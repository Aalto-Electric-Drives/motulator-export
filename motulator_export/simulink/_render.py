"""
Simulink export of the netlists of the control systems (see `plecs._netlist`).

The C blocks are S-functions (`code` "c") or MATLAB Function blocks (`code`
"matlab", see `_mfunction`), and the subsystems, the selectors, the multiplexers,
and the tags are the corresponding Simulink blocks. The netlist is laid out in the
geometry of Simulink, and the builder `blocks.add_netlist` adds the blocks at their
positions, aligns them with the ports given by the alignments of the layout (since
Simulink places the ports itself), and draws the lines through the corners of the
wires. The contents of all subsystems are given as rows with the paths of the
blocks relative to the control-system subsystem.

A MATLAB Function block has the mask of its C block, or a mask of its parameters,
whose initialization collects the parameters into the struct p of the function. A
block with inputs without direct feedthrough is a masked subsystem containing the
MATLAB Function block and the unit delays of these inputs, laid out as a netlist.

"""

from dataclasses import dataclass, fields, replace
from typing import Any

from motulator_export.plecs._netlist import (
    SIMULINK,
    CBlock,
    Delay,
    Layout,
    Mask,
    Mux,
    Point,
    Port,
    Selector,
    Subsystem,
    Tag,
    layout,
    leaves,
)
from motulator_export.simulink._mfunction import (
    PARAMS_MASK,
    MFunction,
    block_mask,
    block_mfunction,
    function_name,
)
from motulator_export.simulink._sfunction import SFunction, block_sfunction

# ASCII replacements in the mask prompts
ASCII = str.maketrans({"Ω": "Ohm", "²": "^2"})

# Output of the monitored signals of the control-system subsystem
SIGNALS = "signals"

# Codes of the blocks of the control systems: the C port in S-functions, or the
# MATLAB port in MATLAB Function blocks
CODES = ("c", "matlab")


@dataclass
class Wrapped(CBlock):
    """
    MATLAB Function block of a block with delayed inputs, in the subsystem of the
    block (`block`), which has the mask.
    """

    block: CBlock | None = None


def with_signals(top: Subsystem, monitor: str) -> Subsystem:
    """
    Add the output `signals` to the top level: the outputs of the C block `monitor`
    (the monitored signals) combined by a multiplexer.
    """
    b = top.block(monitor)
    mux = Mux("Mux signals", [Port(p.name, p.width) for p in b.outputs])
    return replace(
        top,
        outputs=[*top.outputs, Port(SIGNALS, sum(p.width for p in b.outputs))],
        blocks=[*top.blocks, mux],
        connections=[
            *top.connections,
            *((f"{monitor}:{p.name}", f"Mux signals:{p.name}") for p in b.outputs),
            ("Mux signals:y", SIGNALS),
        ],
        columns=[*top.columns, ["Mux signals"], [SIGNALS]],
        align={
            **top.align,
            f"Mux signals:{b.outputs[0].name}": f"{monitor}:{b.outputs[0].name}",
            SIGNALS: "Mux signals:y",
        },
    )


def sfunctions(top: Subsystem) -> list[SFunction]:
    """S-functions of the C blocks."""
    return [block_sfunction(b) for _, b in leaves(top)]


def mfunctions(top: Subsystem) -> list[MFunction]:
    """MATLAB functions of the C blocks, each once."""
    out: dict[str, MFunction] = {}
    for _, b in leaves(top):
        f = block_mfunction(b)
        if f.name in out and out[f.name].source != f.source:
            raise ValueError(f"Different functions named {f.name}")
        out[f.name] = f
    return list(out.values())


def wrapper(b: CBlock, fun: MFunction) -> Subsystem:
    """
    Subsystem of a block with delayed inputs: the MATLAB Function block, named by
    its function, and the unit delays of the inputs without direct feedthrough.
    """
    name = fun.name
    inner = Wrapped(
        **{f.name: getattr(b, f.name) for f in fields(CBlock)}
        | {"name": name, "mask": None, "feedthrough": None},
        block=b,
    )
    sample_time = f"p.{fun.sample_time}"
    delays = [
        Delay(f"Delay {b.inputs[k].name}", b.inputs[k].width, sample_time)
        for k in fun.delayed
    ]
    connections: list[tuple[str, str]] = []
    align: dict[str, str] = {}
    for k, p in enumerate(b.inputs):
        if k in fun.delayed:
            d = f"Delay {p.name}"
            connections += [(p.name, f"{d}:u"), (f"{d}:y", f"{name}:{p.name}")]
            align |= {f"{d}:y": f"{name}:{p.name}", p.name: f"{d}:u"}
        else:
            connections.append((p.name, f"{name}:{p.name}"))
            align[p.name] = f"{name}:{p.name}"
    for p in b.outputs:
        connections.append((f"{name}:{p.name}", p.name))
        align[p.name] = f"{name}:{p.name}"
    return Subsystem(
        b.name,
        b.inputs,
        b.outputs,
        [inner, *delays],
        connections,
        columns=[
            [p.name for p in b.inputs],
            [d.name for d in delays],
            [name],
            [p.name for p in b.outputs],
        ],
        align=align,
    )


def _kind(lay: Layout, name: str, port: str, out: bool) -> list[Any]:
    """Port as {block, kind, number} of Simulink."""
    b = lay.blocks[name]
    if b.kind in ("input", "from"):
        return [name, "Outport", 1.0]
    if b.kind in ("output", "goto"):
        return [name, "Inport", 1.0]
    ports = b.outputs if out else b.inputs
    return [name, "Outport" if out else "Inport", float(ports.index(port) + 1)]


def _port(lay: Layout, ref: str, out: bool) -> str:
    """Port as 'block/number' of Simulink."""
    name, port = ref.split(":", 1) if ":" in ref else (ref, "")
    b = lay.blocks[name]
    if b.kind in ("input", "output", "from", "goto"):
        return f"{name}/1"
    ports = b.outputs if out else b.inputs
    return f"{name}/{ports.index(port) + 1}"


def _corners(points: list[Point]) -> list[Any]:
    """
    Corners of a wire as the arguments of `blocks.route`: 'x', x for a horizontal
    and 'y', y for a vertical segment, except the last vertical one, which the route
    adds to the destination.
    """
    out: list[Any] = []
    for a, b in zip(points[:-2], points[1:-1], strict=True):
        if a[1] == b[1]:
            out += ["x", float(b[0])]
        else:
            out += ["y", float(b[1])]
    if len(out) >= 2 and out[-2] == "y" and points[-2][1] == points[-1][1]:
        out = out[:-2]
    return out


def netlist_rows(
    sub: Subsystem, path: str = "", code: str = "c"
) -> dict[str, list[list[Any]]]:
    """
    Rows of the blocks, the alignments, the lines, and the masks of a subsystem and
    its subsystems (see `blocks.add_netlist`), the paths relative to the subsystem.
    The C blocks are S-functions (`code` "c") or MATLAB Function blocks ("matlab").
    """
    if code not in CODES:
        raise ValueError(f"Unknown code {code}, not in {CODES}")
    lay = layout(sub, SIMULINK)
    rows: dict[str, list[list[Any]]] = {
        "blocks": [],
        "aligns": [],
        "lines": [],
        "masks": [],
    }
    prefix = f"{path}/" if path else ""
    n_in = {p.name: k + 1 for k, p in enumerate(sub.inputs)}
    n_out = {p.name: k + 1 for k, p in enumerate(sub.outputs)}
    children: list[tuple[Subsystem, str]] = []
    for name, b in lay.blocks.items():
        x0, y0, x1, y1 = b.rect
        pos = [float(x0), float(y0), float(x1), float(y1)]
        if name in lay.froms:
            typ, params = _block(Tag(name, lay.froms[name][0], 1, goto=False))
        elif name in lay.gotos:
            typ, params = _block(Tag(name, lay.gotos[name], 1, goto=True))
        elif b.kind == "input":
            typ, params = "Inport", ["Port", str(n_in[name])]
        elif b.kind == "output":
            typ, params = "Outport", ["Port", str(n_out[name])]
        else:
            block = sub.block(name)
            typ, params, mask = _netlist_block(block, prefix + name, code, children)
            if mask is not None:
                assert isinstance(block, (CBlock, Subsystem))
                rows["masks"].append(_mask(prefix + name, block, mask))
        rows["blocks"].append([prefix + name, typ, pos, params])
    for (b, p, out), (t, tp, t_out), dy in lay.aligned:
        rows["aligns"].append(
            [path, *_kind(lay, b, p, out), *_kind(lay, t, tp, t_out), float(dy)]
        )
    for w in lay.wires:
        src = _port(lay, w.src, True)
        dst = _port(lay, w.dst, False)
        rows["lines"].append([path, src, dst, _corners(w.points)])
    for block, child in children:
        inner = netlist_rows(block, child, code)
        for key in rows:
            rows[key] += inner[key]
    return rows


def _netlist_block(
    block: Any, path: str, code: str, children: list[tuple[Subsystem, str]]
) -> tuple[str, list[Any], Mask | None]:
    """
    Type, parameters, and mask of a block of a netlist. Its subsystem (if any) is
    added to `children`.
    """
    if code == "matlab" and isinstance(block, CBlock):
        return _mfunction_block(block, children, path)
    typ, params = _block(block)
    if isinstance(block, Subsystem):
        children.append((block, path))
    mask = block.mask if isinstance(block, (CBlock, Subsystem)) else None
    return typ, params, mask


def _mfunction_block(
    b: CBlock, children: list[tuple[Subsystem, str]], path: str
) -> tuple[str, list[Any], Mask | None]:
    """
    Type, parameters, and mask of the MATLAB Function block of a C block, or of its
    subsystem with the unit delays, which is added to `children`. The parameters of
    a MATLAB Function block are its function, its sample time, and the widths of
    its outputs (see `blocks.add_mfunction`).
    """
    if isinstance(b, Wrapped):
        assert b.block is not None
        fun = block_mfunction(b.block)
        mask = None
    else:
        fun = block_mfunction(b)
        mask = block_mask(b)
        if fun.delayed:
            children.append((wrapper(b, fun), path))
            return "Subsystem", ["ContentPreviewEnabled", "off"], mask
    sample_time = "" if fun.sample_time is None else f"p.{fun.sample_time}"
    widths = [float(w) for w in fun.output_widths]
    return "MATLAB Function", [fun.name, sample_time, widths], mask


def _mask(path: str, b: CBlock | Subsystem, mask: Mask) -> list[Any]:
    """
    Mask of a block: {path, type, description, initialization, parameters
    {variable, prompt, value}, display (the name and the port labels)}.
    """
    labels = [
        f"port_label('{kind}', {k + 1}, '{p.name}');"
        for kind, ports in (("input", b.inputs), ("output", b.outputs))
        for k, p in enumerate(ports)
    ]
    display = "\n".join([f"disp('{b.name}');", *labels])
    if mask.type == PARAMS_MASK and isinstance(b, CBlock):
        # The name of the function, as an S-function shows its name
        display = f"disp('{function_name(b)}');"
    params = [[m.variable, m.prompt.translate(ASCII), m.value] for m in mask.params]
    init = mask.init + mask.simulink_init
    return [path, mask.type, mask.description, init, params, display]


def _block(b: Any) -> tuple[str, list[str]]:
    """Type and parameters of a block of the netlist."""
    if isinstance(b, CBlock):
        # Dummy parameters until the mask exists (see blocks.add_netlist)
        dummy = ", ".join(["1"] * len(b.params))
        return "S-Function", ["FunctionName", b.sfunction, "Parameters", dummy]
    if isinstance(b, Subsystem):
        return "Subsystem", ["ContentPreviewEnabled", "off"]
    if isinstance(b, Selector):
        indices = " ".join(str(k + 1) for k in b.indices)
        return "Selector", [
            "NumberOfDimensions",
            "1",
            "IndexOptions",
            "Index vector (dialog)",
            "Indices",
            f"[{indices}]",
            "InputPortWidth",
            str(b.width),
        ]
    if isinstance(b, Delay):
        sample_time = "-1" if b.sample_time is None else b.sample_time
        return "UnitDelay", ["SampleTime", sample_time]
    # The names of the tags and the multiplexers are not shown
    if isinstance(b, Tag):
        params = ["GotoTag", b.tag, "ShowName", "off"]
        visibility = ["TagVisibility", "local"] if b.goto else []
        return ("Goto" if b.goto else "From"), [*params, *visibility]
    widths = " ".join(str(p.width) for p in b.inputs)
    return "Mux", ["Inputs", f"[{widths}]", "ShowName", "off"]


def sfunction_params(top: Subsystem) -> list[list[str]]:
    """Rows {path, parameters} of the S-functions, set after the mask exists."""
    return [
        [path, ", ".join(f"double({p})" for p in b.params)] for path, b in leaves(top)
    ]
