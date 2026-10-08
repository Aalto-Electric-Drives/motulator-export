"""
Netlists of the control systems and their layout.

A control system is a hierarchy of subsystems that mirrors the classes of
motulator: each class is a block, either a C-Script block (`CBlock`, an S-function
in Simulink) or a subsystem of blocks (`Subsystem`). The signal vectors (e.g., the
feedback signals `fbk`) are assembled by `Mux` blocks, and their fields are selected
by `Selector` blocks, where motulator accesses the attributes. The netlist is
independent of PLECS and Simulink.

The layout (`layout`) places the blocks of a subsystem in the columns given by its
hints and aligns them vertically, so that the wires are straight, and routes the
other wires orthogonally: a forward wire turns in the channel left of its
destination, and a wire through a lane (e.g., a feedback wire below the blocks)
turns in the channels next to its source and its destination. The vertical segments
in a channel are ordered so that the wires do not cross, and `problems` checks the
result. The ports of the blocks are placed by the geometry of the tool (`PLECS` or
`SIMULINK`).

"""

import re
from collections.abc import Sequence
from dataclasses import dataclass, field
from itertools import combinations

Point = tuple[int, int]


# %%
@dataclass(frozen=True)
class Port:
    """Port of a block: its name and the width of its signal."""

    name: str
    width: int


@dataclass(frozen=True)
class Param:
    """Mask parameter: its variable, its prompt, and its value (an expression)."""

    variable: str
    prompt: str
    value: str


@dataclass
class Mask:
    """
    Mask of a block (a class of motulator): its type, its description, its
    parameters, and its initialization commands (MATLAB syntax, also understood by
    PLECS), which compute the variables passed to the blocks inside it.
    """

    type: str
    description: str
    params: list[Param]
    init: str = ""


@dataclass
class CBlock:
    """
    C-Script block of PLECS or S-function of Simulink, running the C code.

    The parameters are expressions in the workspace of its mask, or of the masks of
    the subsystems containing it. A block with a mask is a masked subsystem with the
    C-Script block in PLECS and a masked S-function in Simulink.
    """

    name: str
    inputs: list[Port]
    outputs: list[Port]
    code: dict[str, str]  # Code sections of the C-Script
    params: list[str]  # Parameters, expressions in the mask workspace
    sfunction: str  # Name of the S-function in Simulink
    feedthrough: list[int] | None = None  # Direct feedthrough, all inputs by default
    sample_time: str | None = "T_s"  # Expression of the sampling period, or inherited
    mask: Mask | None = None


@dataclass
class Selector:
    """Selection of the elements `indices` (zero-based) of a signal vector."""

    name: str
    width: int
    indices: list[int]

    @property
    def inputs(self) -> list[Port]:
        return [Port("u", self.width)]

    @property
    def outputs(self) -> list[Port]:
        return [Port("y", len(self.indices))]


@dataclass
class Mux:
    """Concatenation of the input signals into a signal vector."""

    name: str
    inputs: list[Port]

    @property
    def outputs(self) -> list[Port]:
        return [Port("y", sum(p.width for p in self.inputs))]


@dataclass
class Tag:
    """Goto block (`goto` True) or From block of a signal connected by a tag."""

    name: str
    tag: str
    width: int
    goto: bool

    @property
    def inputs(self) -> list[Port]:
        return [Port("u", self.width)] if self.goto else []

    @property
    def outputs(self) -> list[Port]:
        return [] if self.goto else [Port("y", self.width)]


@dataclass
class Subsystem:
    """
    Subsystem of blocks.

    The connections are pairs (source, destination) of ports, given as "block:port",
    or by the name of an input or output of the subsystem. The inputs in `tags` are
    distributed to their destinations by Goto and From blocks instead of wires
    (e.g., the input `enable` of the stateful blocks), which are added in the
    layout. Other signals can be connected by tags with `Tag` blocks.

    The layout hints are the columns of the blocks, the inputs, and the outputs
    (`columns`, from left to right, each from top to bottom), the alignments
    (`align`: the port, given as "block:port" or by the name of a block with one
    port, is aligned with the port given as the value, optionally with an offset in
    the gaps between the blocks, e.g., "block:port+1"), and the lanes of the wires
    routed around the blocks (`lanes`: the destination port to the lane, see
    `layout`). A block without an alignment is placed below the previous block of
    its column.

    """

    name: str
    inputs: list[Port]
    outputs: list[Port]
    blocks: list["Block"]
    connections: list[tuple[str, str]]
    columns: list[list[str]]
    align: dict[str, str] = field(default_factory=dict)
    lanes: dict[str, str] = field(default_factory=dict)
    tags: list[str] = field(default_factory=list)
    mask: Mask | None = None

    def block(self, name: str) -> "Block":
        """Block by its name."""
        for b in self.blocks:
            if b.name == name:
                return b
        raise KeyError(name)


Block = CBlock | Selector | Mux | Tag | Subsystem


def leaves(sub: Subsystem, path: str = "") -> list[tuple[str, CBlock]]:
    """C blocks of a subsystem and its subsystems, with their paths."""
    out: list[tuple[str, CBlock]] = []
    for b in sub.blocks:
        name = f"{path}/{b.name}" if path else b.name
        if isinstance(b, Subsystem):
            out += leaves(b, name)
        elif isinstance(b, CBlock):
            out.append((name, b))
    return out


def port_ref(sub: Subsystem, ref: str) -> str:
    """Port reference "block:port", also of a block with one port given by its name."""
    if ":" in ref or ref in [p.name for p in [*sub.inputs, *sub.outputs]]:
        return ref
    b = sub.block(ref)
    ports = [*b.inputs, *b.outputs]
    if len(ports) != 1:
        raise ValueError(f"{sub.name}: {ref}: the port name is needed")
    return f"{ref}:{ports[0].name}"


def check(sub: Subsystem) -> None:
    """Check the names, the ports, and the connections of a subsystem (recursively)."""
    names = [p.name for p in [*sub.inputs, *sub.outputs]] + [b.name for b in sub.blocks]
    if len(set(names)) != len(names):
        raise ValueError(f"{sub.name}: duplicate names")
    widths = {p.name: p.width for p in [*sub.inputs, *sub.outputs]}
    widths |= {f"{b.name}:{p.name}": p.width for b in sub.blocks for p in b.inputs}
    widths |= {f"{b.name}:{p.name}": p.width for b in sub.blocks for p in b.outputs}
    connections = [(port_ref(sub, s), port_ref(sub, d)) for s, d in sub.connections]
    dsts = [dst for _, dst in connections]
    if len(set(dsts)) != len(dsts):
        raise ValueError(f"{sub.name}: a port with several sources")
    for src, dst in connections:
        if src not in widths or widths[src] != widths.get(dst):
            raise ValueError(f"{sub.name}: {src} -> {dst}: unknown port or width")
    inputs = [f"{b.name}:{p.name}" for b in sub.blocks for p in b.inputs]
    missing = [p for p in inputs + [p.name for p in sub.outputs] if p not in dsts]
    if missing:
        raise ValueError(f"{sub.name}: not connected: {missing}")
    tags = [b for b in sub.blocks if isinstance(b, Tag)]
    for b in tags:
        if [t.goto for t in tags if t.tag == b.tag].count(True) != 1:
            raise ValueError(f"{sub.name}: tag {b.tag} needs one Goto block")
    for b in sub.blocks:
        if isinstance(b, Subsystem):
            check(b)


# %%
@dataclass(frozen=True)
class Geometry:
    """
    Geometry of the blocks of a tool: their sizes and the positions of their ports.

    The ports of a block of a kind in `spacing` are spaced by its value and centered
    at the block (as in the C-Script blocks of PLECS), and else they are placed
    evenly over the height of the block (as in Simulink), whose height is `pitch`
    per port. The kinds in `sizes` have a fixed size (width, height).

    """

    sizes: dict[str, tuple[int, int]]
    widths: dict[str, int]
    spacing: dict[str, int]
    pitch: dict[str, int]
    stub: int  # Distance of the output terminals from the frame
    grid: int  # Grid of the positions
    channel: int  # Width of a channel without vertical segments
    slot: int  # Distance between the vertical segments in a channel
    gap: int  # Vertical gap between the blocks in a column
    lane: int  # Distance between the lanes
    margin: int  # Distance of the lanes from the blocks

    def size(self, kind: str, n_in: int, n_out: int) -> tuple[int, int]:
        """Size (width, height) of a block."""
        if kind in self.sizes:
            return self.sizes[kind]
        n = max(n_in, n_out, 1)
        if kind in self.spacing:
            return self.widths[kind], self.spacing[kind] * n + 4 * self.grid
        return self.widths[kind], self.pitch[kind] * max(n, 2 if kind != "mux" else 1)

    def port_dy(self, kind: str, n: int, k: int, h: int) -> float:
        """Vertical offset of the port k (zero-based) of n from the center."""
        s = self.spacing[kind] if kind in self.spacing else h / n
        return s * (k - (n - 1) / 2)


PLECS = Geometry(
    sizes={
        "input": (20, 10),
        "output": (20, 10),
        "selector": (30, 20),
        "goto": (40, 10),
        "from": (40, 10),
    },
    widths={"cblock": 100, "subsystem": 100, "mux": 10},
    spacing={"cblock": 10, "mux": 10, "subsystem": 20},
    pitch={},
    stub=4,
    grid=5,
    channel=30,
    slot=10,
    gap=30,
    lane=10,
    margin=25,
)

SIMULINK = Geometry(
    sizes={
        "input": (30, 14),
        "output": (30, 14),
        "selector": (50, 30),
        "goto": (70, 14),
        "from": (70, 14),
    },
    widths={"cblock": 140, "subsystem": 140, "mux": 5},
    spacing={},
    pitch={"cblock": 30, "subsystem": 40, "mux": 30},
    stub=5,
    grid=5,
    channel=40,
    slot=15,
    gap=40,
    lane=15,
    margin=40,
)


def kind(b: Block) -> str:
    """Kind of a block in the geometry."""
    if isinstance(b, CBlock):
        return "cblock"
    if isinstance(b, Subsystem):
        return "subsystem"
    if isinstance(b, Selector):
        return "selector"
    if isinstance(b, Tag):
        return "goto" if b.goto else "from"
    return "mux"


# %%
@dataclass
class Placed:
    """Block placed in a layout: its center and size, and its ports."""

    name: str
    kind: str
    inputs: list[str]
    outputs: list[str]
    w: int
    h: int
    in_dy: list[float]
    out_dy: list[float]
    stub: int
    x: int = 0
    y: int = 0
    column: int = 0

    def port(self, name: str, out: bool) -> Point:
        """Position of a port."""
        if out:
            dy = self.out_dy[self.outputs.index(name)]
            return self.x + self.w // 2 + self.stub, round(self.y + dy)
        dy = self.in_dy[self.inputs.index(name)]
        return self.x - self.w // 2, round(self.y + dy)

    @property
    def rect(self) -> tuple[int, int, int, int]:
        """Frame (left, top, right, bottom)."""
        return (
            self.x - self.w // 2,
            self.y - self.h // 2,
            self.x + self.w - self.w // 2,
            self.y + self.h - self.h // 2,
        )


@dataclass
class Wire:
    """Wire of a connection: the points from the source to the destination."""

    src: str
    dst: str
    points: list[Point]


# Alignment of a port with another: (block, port, output) of both, and the offset
Alignment = tuple[tuple[str, str, bool], tuple[str, str, bool], int]


@dataclass
class Layout:
    """
    Layout of a subsystem: the placed blocks (including the ports), the wires, and the
    alignments in the order of the placement (for the tools that place the ports
    themselves, as Simulink).
    """

    blocks: dict[str, Placed]
    wires: list[Wire]
    froms: dict[str, tuple[str, str]]  # From block -> (tag, destination port)
    gotos: dict[str, str]  # Goto block -> tag
    aligned: list[Alignment] = field(default_factory=list)

    @property
    def size(self) -> Point:
        """Size (width, height) of the schematic."""
        xs = [b.rect[2] for b in self.blocks.values()]
        ys = [b.rect[3] for b in self.blocks.values()]
        ys += [p[1] for w in self.wires for p in w.points]
        return max(xs) + 40, max(ys) + 40


def _snap(v: float, grid: int) -> int:
    return int(round(v / grid)) * grid


class _Refs:
    """Resolution of the port references of a subsystem to (block, port, output)."""

    def __init__(self, blocks: dict[str, Placed]) -> None:
        self.blocks = blocks

    def __call__(self, ref: str) -> tuple[str, str, bool]:
        ref = _offset(ref)[0]
        if ":" in ref:
            name, port = ref.split(":", 1)
            b = self.blocks[name]
            return name, port, port in b.outputs and port not in b.inputs
        b = self.blocks[ref]
        ports = [(p, False) for p in b.inputs] + [(p, True) for p in b.outputs]
        if len(ports) != 1:
            raise ValueError(f"{ref}: the port name is needed")
        return ref, *ports[0]

    def point(self, ref: str) -> Point:
        name, port, out = self(ref)
        return self.blocks[name].port(port, out)


def _offset(ref: str) -> tuple[str, float]:
    """Split a port reference with an offset ("block:port+1") into the two."""
    match = re.fullmatch(r"(.*?)([+-]\d+(?:\.\d+)?)?", ref)
    assert match is not None
    return match.group(1), float(match.group(2) or 0)


def layout(sub: Subsystem, geo: Geometry) -> Layout:
    """
    Layout of a subsystem in the geometry of a tool.

    The lane of a wire is "below:k" or "above:k" (the k-th lane below or above all
    blocks), or "under:<block>:k" or "over:<block>:k" (the k-th lane below or above
    the block). A wire with the lane "early" turns in the channel right of its source
    instead of the channel left of its destination. The destinations of the inputs in
    `tags` get From blocks in front of them, and the inputs get Goto blocks next to
    them.

    """
    check(sub)
    blocks, froms, gotos, connections = _blocks(sub, geo)
    refs = _Refs(blocks)
    for c, names in enumerate(sub.columns):
        for name in names:
            blocks[name].column = c
    columned = {n for names in sub.columns for n in names}
    missing = [n for n in blocks if n not in columned and n not in froms | gotos]
    if missing:
        raise ValueError(f"{sub.name}: not in the columns: {missing}")

    aligned = _place_vertically(sub, blocks, refs, geo)
    for name, (_, dst) in froms.items():
        d_name, d_port, _ = refs(dst)
        blocks[name].y = blocks[d_name].port(d_port, False)[1]
        blocks[name].column = blocks[d_name].column
        aligned.append(((name, "y", True), (d_name, d_port, False), 0))
    for name, tag in gotos.items():
        blocks[name].y = blocks[tag].y
        blocks[name].column = blocks[tag].column
        aligned.append(((name, "u", False), (tag, "y", True), 0))
    lay = Layout(blocks, [], froms, gotos, aligned)
    lay.wires = _place_horizontally(sub, lay, connections, refs, geo)

    # Lanes above the blocks give negative coordinates: shift everything down
    ys = [b.rect[1] for b in blocks.values()] + [
        p[1] for w in lay.wires for p in w.points
    ]
    dy = _snap(geo.gap - min(ys), geo.grid)
    for b in blocks.values():
        b.y += dy
    for w in lay.wires:
        w.points = [(x, y + dy) for x, y in w.points]
    return lay


def _blocks(
    sub: Subsystem, geo: Geometry
) -> tuple[dict[str, Placed], dict[str, tuple[str, str]], dict[str, str], list]:
    """
    Blocks of a layout (including the inputs and outputs, and the Goto and From blocks
    of the tags), the From blocks, the Goto blocks, and the connections.
    """
    blocks: dict[str, Placed] = {}

    def add(name: str, k: str, inputs: list[str], outputs: list[str]) -> None:
        w, h = geo.size(k, len(inputs), len(outputs))
        n_i, n_o = len(inputs), len(outputs)
        blocks[name] = Placed(
            name,
            k,
            inputs,
            outputs,
            w,
            h,
            [geo.port_dy(k, n_i, i, h) for i in range(n_i)],
            [geo.port_dy(k, n_o, i, h) for i in range(n_o)],
            geo.stub,
        )

    for p in sub.inputs:
        add(p.name, "input", [], ["y"])
    for p in sub.outputs:
        add(p.name, "output", ["u"], [])
    for b in sub.blocks:
        add(b.name, kind(b), [p.name for p in b.inputs], [p.name for p in b.outputs])
    froms: dict[str, tuple[str, str]] = {}
    gotos: dict[str, str] = {}
    connections: list[tuple[str, str]] = []
    for src, dst in sub.connections:
        if src in sub.tags:
            name = f"From {dst.replace(':', '.')}"
            froms[name] = (src, dst)
            add(name, "from", [], ["y"])
            connections.append((name, dst))
        else:
            connections.append((src, dst))
    for tag in sub.tags:
        gotos[f"Goto {tag}"] = tag
        add(f"Goto {tag}", "goto", ["u"], [])
        connections.append((tag, f"Goto {tag}"))
    return blocks, froms, gotos, connections


def _place_horizontally(
    sub: Subsystem,
    lay: Layout,
    connections: list[tuple[str, str]],
    refs: _Refs,
    geo: Geometry,
) -> list[Wire]:
    """
    Place the blocks horizontally and route the wires. The columns are separated by
    channels, whose widths depend on the numbers of the vertical segments in them
    (from a first routing).
    """
    blocks, froms, gotos = lay.blocks, lay.froms, lay.gotos
    n_col = len(sub.columns)
    col_w = [0] * n_col
    from_w = [0] * n_col  # Space of the From blocks in front of the columns
    for name, b in blocks.items():
        if name in froms:
            from_w[b.column] = b.w + geo.channel // 2
        elif name not in gotos:
            col_w[b.column] = max(col_w[b.column], b.w + b.stub)
    for name, tag in gotos.items():
        c = blocks[tag].column
        col_w[c] = max(col_w[c], blocks[tag].w + geo.channel + blocks[name].w)
    slots = dict.fromkeys(range(-1, n_col), 0)
    wires: list[Wire] = []
    for _ in range(2):
        x = geo.channel + geo.slot * slots[-1]
        col_x = []
        for c in range(n_col):
            x += from_w[c]
            col_x.append(x)
            x += col_w[c] + geo.channel + geo.slot * slots[c]
        for name, b in blocks.items():
            if name not in froms | gotos:
                b.x = _snap(col_x[b.column] + b.w / 2, geo.grid)
        for name, tag in gotos.items():
            t = blocks[tag]
            blocks[name].x = t.x + t.w // 2 + geo.channel + blocks[name].w // 2
        for name, (_, dst) in froms.items():
            b = blocks[name]
            b.x = refs.point(dst)[0] - geo.channel // 2 - b.w // 2 - b.stub
        plans = _plan(sub, connections, blocks, refs, geo)
        segs = _segments(plans)
        slots = {c: len([s for s in segs if s.channel == c]) for c in range(-1, n_col)}
        wires = _route(plans, segs, col_x, col_w, geo)
    return wires


def _place_vertically(
    sub: Subsystem, blocks: dict[str, Placed], refs: _Refs, geo: Geometry
) -> list[Alignment]:
    """
    Place the blocks vertically: the aligned ones by their alignments, and the others
    below the previous blocks of their columns. A block is placed when its target
    and the previous block of its column have been placed. An aligned block is
    pushed down if it would overlap the previous block. Returns the alignments that
    hold, in the order of the placement.
    """
    aligned: list[Alignment] = []
    previous: dict[str, str | None] = {}
    for names in sub.columns:
        for k, name in enumerate(names):
            previous[name] = names[k - 1] if k > 0 else None
    anchors = {refs(key)[0]: key for key in sub.align}
    placed: set[str] = set()
    pending = [n for names in sub.columns for n in names]
    while pending:
        progress = False
        for name in list(pending):
            b = blocks[name]
            prev = previous[name]
            key = anchors.get(name)
            if prev is not None and prev not in placed:
                continue
            alignment = None
            if key is not None:
                target, offset = _offset(sub.align[key])
                if refs(target)[0] not in placed:
                    continue
                dy = round(offset * (geo.gap + geo.grid))
                alignment = (refs(key), refs(target), dy)
                b.y = 0
                b.y = refs.point(target)[1] + dy - refs.point(key)[1]
                # Below the previous block, leaving space for its name
                top = blocks[prev].rect[3] + geo.gap // 2 if prev is not None else None
            else:
                top = blocks[prev].rect[3] + geo.gap if prev is not None else 0
                b.y = _snap(top + b.h / 2, geo.grid)
            if top is not None and b.rect[1] < top:
                b.y += _snap(top - b.rect[1] + geo.grid / 2, geo.grid)
            elif alignment is not None:
                aligned.append(alignment)
            placed.add(name)
            pending.remove(name)
            progress = True
        if not progress:
            raise ValueError(f"{sub.name}: circular alignments: {pending}")
    return aligned


# %%
@dataclass
class _Segment:
    """Vertical segment of a wire in a channel, with its horizontal attachments."""

    net: str  # Source of the wire
    part: int  # 0: forward jog, 1: out of the source to a lane, 2: from a lane
    channel: int
    ys: list[int]  # Ends of the segment
    left: list[int] = field(default_factory=list)  # Horizontals attached on the left
    right: list[int] = field(default_factory=list)  # Horizontals on the right
    x: int = 0

    @property
    def span(self) -> tuple[int, int]:
        return min(self.ys), max(self.ys)


_Plan = tuple[str, str, Point, Point, tuple]


def _lane_y(lane: str, blocks: dict[str, Placed], geo: Geometry) -> int:
    """Height of a lane."""
    parts = lane.split(":")
    k = int(parts[-1])
    rects = [b.rect for b in blocks.values()]
    if parts[0] == "below":
        y = max(r[3] for r in rects) + geo.margin + geo.lane * k
    elif parts[0] == "above":
        y = min(r[1] for r in rects) - geo.margin - geo.lane * k
    elif parts[0] == "under":
        y = blocks[parts[1]].rect[3] + geo.margin + geo.lane * k
    elif parts[0] == "over":
        y = blocks[parts[1]].rect[1] - geo.margin - geo.lane * k
    else:
        raise ValueError(f"Unknown lane: {lane}")
    return _snap(y, geo.grid)


def _plan(
    sub: Subsystem,
    connections: list[tuple[str, str]],
    blocks: dict[str, Placed],
    refs: _Refs,
    geo: Geometry,
) -> list[_Plan]:
    """
    Plan the routes: (src, dst, p, q, plan) with the terminal points p and q, and
    the plan ("straight",), ("jog", channel), or ("lane", y, channel_1, channel_2).
    """
    plans: list[_Plan] = []
    for src, dst in connections:
        p, q = refs.point(src), refs.point(dst)
        c_s, c_d = blocks[refs(src)[0]].column, blocks[refs(dst)[0]].column
        lane = sub.lanes.get(dst)
        if lane == "early":
            plans.append((src, dst, p, q, ("jog", c_s)))
        elif lane is not None:
            y = _lane_y(lane, blocks, geo)
            plans.append((src, dst, p, q, ("lane", y, c_s, c_d - 1)))
        elif q[0] <= p[0]:
            raise ValueError(f"{sub.name}: {src} -> {dst} needs a lane")
        elif p[1] == q[1]:
            plans.append((src, dst, p, q, ("straight",)))
        else:
            plans.append((src, dst, p, q, ("jog", c_d - 1)))
    return plans


def _segments(plans: list[_Plan]) -> list[_Segment]:
    """Vertical segments of the planned routes, merged per net, part, and channel."""
    segs: dict[tuple[str, int, int], _Segment] = {}

    def seg(net: str, part: int, channel: int) -> _Segment:
        key = (net, part, channel)
        if key not in segs:
            segs[key] = _Segment(net, part, channel, [])
        return segs[key]

    for src, _, p, q, plan in plans:
        if plan[0] == "jog":
            s = seg(src, 0, plan[1])
            s.ys += [p[1], q[1]]
            s.left.append(p[1])
            s.right.append(q[1])
        elif plan[0] == "lane":
            _, y, c1, c2 = plan
            s1, s2 = seg(src, 1, c1), seg(src, 2, c2)
            s1.ys += [p[1], y]
            s1.left.append(p[1])
            s2.ys += [y, q[1]]
            s2.right.append(q[1])
            # The lane between the segments
            if c2 < c1:
                s1.left.append(y)
                s2.right.append(y)
            elif c2 > c1:
                s1.right.append(y)
                s2.left.append(y)
    return list(segs.values())


def _order(segs: list[_Segment]) -> list[_Segment]:
    """Order the vertical segments of a channel from left to right."""

    def inside(y: int, span: tuple[int, int]) -> bool:
        return span[0] <= y <= span[1]

    def ok(a: _Segment, b: _Segment) -> bool:
        """Whether a can be left of b without crossings."""
        if any(inside(y, b.span) for y in a.right):
            return False
        return not any(inside(y, a.span) for y in b.left)

    n = len(segs)
    before: dict[int, set[int]] = {i: set() for i in range(n)}  # Must be left of i
    for i, j in combinations(range(n), 2):
        a, b = segs[i], segs[j]
        if a.net == b.net:
            continue
        ij, ji = ok(a, b), ok(b, a)
        if ij and not ji:
            before[j].add(i)
        elif ji and not ij:
            before[i].add(j)
    order: list[int] = []
    remaining = set(range(n))
    while remaining:
        free = [i for i in remaining if not before[i] & remaining] or list(remaining)
        # Ties: the segments from the left first, the shorter ones first
        i = min(
            free,
            key=lambda i: (
                not segs[i].left,
                segs[i].span[1] - segs[i].span[0],
                segs[i].net,
            ),
        )
        order.append(i)
        remaining.remove(i)
    return [segs[i] for i in order]


def _route(
    plans: list[_Plan],
    segs: list[_Segment],
    col_x: list[int],
    col_w: list[int],
    geo: Geometry,
) -> list[Wire]:
    """Route the wires: the points of each connection."""
    for c in range(-1, len(col_x)):
        x0 = col_x[c] + col_w[c] + geo.channel // 2 if c >= 0 else geo.channel // 2
        in_channel = [s for s in segs if s.channel == c]
        for k, s in enumerate(_order(in_channel)):
            s.x = _snap(x0 + geo.slot * k, geo.grid)
    x = {(s.net, s.part, s.channel): s.x for s in segs}
    wires = []
    for src, dst, p, q, plan in plans:
        if plan[0] == "straight":
            pts = [p, q]
        elif plan[0] == "jog":
            x0 = x[(src, 0, plan[1])]
            pts = [p, (x0, p[1]), (x0, q[1]), q]
        else:
            _, y, c1, c2 = plan
            x1, x2 = x[(src, 1, c1)], x[(src, 2, c2)]
            pts = [p, (x1, p[1]), (x1, y), (x2, y), (x2, q[1]), q]
        wires.append(Wire(src, dst, _simplify(pts)))
    return wires


def _simplify(pts: Sequence[Point]) -> list[Point]:
    """Remove the repeated points and the points in the middle of straight runs."""
    out: list[Point] = []
    for p in pts:
        if out and p == out[-1]:
            continue
        if len(out) >= 2:
            a, b = out[-2], out[-1]
            if a[0] == b[0] == p[0] or a[1] == b[1] == p[1]:
                out[-1] = p
                continue
        out.append(p)
    return out


# %%
def _pairs(points: Sequence[Point]) -> list[tuple[Point, Point]]:
    return [(points[k], points[k + 1]) for k in range(len(points) - 1)]


def _touch(s: tuple[Point, Point], t: tuple[Point, Point], inset: int = 0) -> bool:
    """Whether two orthogonal segments (or a segment and a box) intersect."""
    (ax0, ay0), (ax1, ay1) = s
    (bx0, by0), (bx1, by1) = t
    return max(min(ax0, ax1), min(bx0, bx1) + inset) <= min(
        max(ax0, ax1), max(bx0, bx1) - inset
    ) and max(min(ay0, ay1), min(by0, by1) + inset) <= min(
        max(ay0, ay1), max(by0, by1) - inset
    )


def problems(lay: Layout) -> list[str]:
    """
    Crossings of the wires of different nets, and the wires through blocks.

    The wires from the same source may overlap. Returns the descriptions of the
    problems, empty if there are none.
    """
    out = []
    nets: dict[str, list[tuple[Point, Point]]] = {}
    for w in lay.wires:
        nets.setdefault(w.src, []).extend(_pairs(w.points))
    for (n1, s1), (n2, s2) in combinations(nets.items(), 2):
        hits = [(a, b) for a in s1 for b in s2 if _touch(a, b)]
        if hits:
            out.append(f"{n1} crosses {n2} at {hits[0][0]}")
    for w in lay.wires:
        ends = {w.src.split(":")[0], w.dst.split(":")[0]}
        for b in lay.blocks.values():
            x0, y0, x1, y1 = b.rect
            # The wires may touch the frames only at their own terminals
            inset = 1 if b.name in ends else 0
            for s in _pairs(w.points):
                if _touch(s, ((x0, y0), (x1, y1)), inset=inset):
                    out.append(f"{w.src} -> {w.dst} through {b.name}")
    return out


# %%
@dataclass
class Tree:
    """Branched wire: the points from its start, and the branches or a destination."""

    points: list[Point]
    branches: list["Tree"]
    dst: str = ""


def _direction(a: Point, b: Point) -> tuple[int, int]:
    return (b[0] > a[0]) - (b[0] < a[0]), (b[1] > a[1]) - (b[1] < a[1])


def _common(polys: list[list[Point]]) -> list[Point]:
    """Common path of polylines from the same start (its corner points)."""
    cur = polys[0][0]
    path = [cur]
    idx = [1] * len(polys)
    while all(i < len(p) for i, p in zip(idx, polys, strict=True)):
        dirs = {_direction(cur, p[i]) for i, p in zip(idx, polys, strict=True)}
        if len(dirs) != 1:
            break
        d = dirs.pop()
        step = min(
            abs(p[i][0] - cur[0]) + abs(p[i][1] - cur[1])
            for i, p in zip(idx, polys, strict=True)
        )
        cur = (cur[0] + d[0] * step, cur[1] + d[1] * step)
        path.append(cur)
        for k, p in enumerate(polys):
            if p[idx[k]] == cur:
                idx[k] += 1
    return _simplify(path)


def _rest(poly: list[Point], at: Point) -> list[Point]:
    """Remainder of a polyline from a point on it."""
    for k in range(len(poly) - 1):
        if _touch((poly[k], poly[k + 1]), (at, at)):
            return [at, *poly[k + 1 :]] if poly[k + 1] != at else [at, *poly[k + 2 :]]
    raise ValueError(f"{at} not on the polyline")


def tree(wires: Sequence[Wire]) -> Tree:
    """Branched wire of the wires from the same source."""
    return _tree([list(w.points) for w in wires], [w.dst for w in wires])


def _tree(polys: list[list[Point]], dsts: list[str]) -> Tree:
    if len(polys) == 1:
        return Tree(polys[0][1:-1], [], dsts[0])
    path = _common(polys)
    if len(path) == 1:
        return Tree([], _groups(polys, dsts))
    at = path[-1]
    return Tree(path[1:], _groups([_rest(p, at) for p in polys], dsts))


def _groups(polys: list[list[Point]], dsts: list[str]) -> list[Tree]:
    """Branches from a common start, grouped by their first directions."""
    groups: dict[tuple[int, int], list[int]] = {}
    for k, p in enumerate(polys):
        groups.setdefault(_direction(p[0], p[1]), []).append(k)
    trees = []
    for ks in groups.values():
        t = _tree([polys[k] for k in ks], [dsts[k] for k in ks])
        if len(ks) == 1:
            t.points = polys[ks[0]][1:-1]
        trees.append(t)
    return trees


# %%
def preview(lay: Layout, path: str, title: str = "") -> None:
    """Draw a layout into an image file, for checking it (needs matplotlib)."""
    import matplotlib.pyplot as plt  # noqa: PLC0415
    from matplotlib.patches import Rectangle  # noqa: PLC0415

    w, h = lay.size
    fig, ax = plt.subplots(figsize=(w / 60, h / 60))
    for b in lay.blocks.values():
        x0, y0, x1, y1 = b.rect
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, lw=0.8))
        if b.kind in ("goto", "from"):
            tag = lay.froms[b.name][0] if b.name in lay.froms else b.name.split()[1]
            ax.text(b.x, b.y, f"[{tag}]", ha="center", va="center", fontsize=4)
        elif b.kind != "mux":
            ax.text(b.x, y1 + 2, b.name, ha="center", va="top", fontsize=5)
    colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]
    nets = sorted({wi.src for wi in lay.wires})
    for wi in lay.wires:
        xs, ys = zip(*wi.points, strict=True)
        ax.plot(xs, ys, lw=0.8, color=colors[nets.index(wi.src) % len(colors)])
    ax.set_xlim(0, w)
    ax.set_ylim(h, 0)
    ax.set_aspect("equal")
    ax.set_title(title, fontsize=7)
    ax.axis("off")
    fig.savefig(path, dpi=200, bbox_inches="tight")
    plt.close(fig)
