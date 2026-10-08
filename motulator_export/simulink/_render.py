"""
Simulink export of the netlists of the control systems (see `plecs._netlist`).

The C blocks are S-functions, and the subsystems, the selectors, the multiplexers,
and the tags are the corresponding Simulink blocks. The netlist is laid out in the
geometry of Simulink, and the builder `blocks.add_netlist` adds the blocks at their
positions, aligns them with the ports given by the alignments of the layout (since
Simulink places the ports itself), and draws the lines through the corners of the
wires. The contents of all subsystems are given as rows with the paths of the
blocks relative to the control-system subsystem.

"""

from dataclasses import replace
from typing import Any

from motulator_export.plecs._netlist import (
    SIMULINK,
    CBlock,
    Layout,
    Mux,
    Point,
    Port,
    Selector,
    Subsystem,
    Tag,
    layout,
    leaves,
)
from motulator_export.simulink._sfunction import SFunction, block_sfunction

# ASCII replacements in the mask prompts
ASCII = str.maketrans({"Ω": "Ohm", "²": "^2"})

# Output of the monitored signals of the control-system subsystem
SIGNALS = "signals"


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


def netlist_rows(sub: Subsystem, path: str = "") -> dict[str, list[list[Any]]]:
    """
    Rows of the blocks, the alignments, the lines, and the masks of a subsystem and
    its subsystems (see `blocks.add_netlist`), the paths relative to the subsystem.
    """
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
    children = []
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
            typ, params = _block(block)
            if isinstance(block, Subsystem):
                children.append((block, prefix + name))
            if isinstance(block, (CBlock, Subsystem)) and block.mask is not None:
                rows["masks"].append(_mask(prefix + name, block))
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
        inner = netlist_rows(block, child)
        for key in rows:
            rows[key] += inner[key]
    return rows


def _mask(path: str, b: CBlock | Subsystem) -> list[Any]:
    """
    Mask of a block: {path, type, description, initialization, parameters
    {variable, prompt, value}, display (the name and the port labels)}.
    """
    mask = b.mask
    assert mask is not None
    labels = [
        f"port_label('{kind}', {k + 1}, '{p.name}');"
        for kind, ports in (("input", b.inputs), ("output", b.outputs))
        for k, p in enumerate(ports)
    ]
    display = "\n".join([f"disp('{b.name}');", *labels])
    params = [[m.variable, m.prompt.translate(ASCII), m.value] for m in mask.params]
    return [path, mask.type, mask.description, mask.init, params, display]


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
    # The names of the tags and the multiplexers are not shown
    if isinstance(b, Tag):
        params = ["GotoTag", b.tag, "ShowName", "off"]
        if b.goto:
            return "Goto", [*params, "TagVisibility", "local"]
        return "From", params
    widths = " ".join(str(p.width) for p in b.inputs)
    return "Mux", ["Inputs", f"[{widths}]", "ShowName", "off"]


def sfunction_params(top: Subsystem) -> list[list[str]]:
    """Rows {path, parameters} of the S-functions, set after the mask exists."""
    return [
        [path, ", ".join(f"double({p})" for p in b.params)] for path, b in leaves(top)
    ]
