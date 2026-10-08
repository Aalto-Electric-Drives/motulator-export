"""
PLECS rendering of the netlists of the control systems (see `_netlist`).

The C blocks are C-Script blocks, the subsystems are PLECS subsystems, and the
selectors and multiplexers are the Signal Selector and Signal Multiplexer blocks.
The tags are distributed by Goto and From blocks. A block with a mask is a masked
subsystem; a C block with a mask contains the C-Script block, whose parameters
refer to the variables of the mask.

"""

from dataclasses import replace

from motulator_export.plecs._netlist import (
    PLECS,
    CBlock,
    Mask,
    Mux,
    Placed,
    Point,
    Selector,
    Subsystem,
    Tag,
    Tree,
    layout,
    tree,
)
from motulator_export.plecs._schematic import (
    Branches,
    Terminal,
    _cscript,
    _inner_schematic,
    _mask_param,
    _q,
    _Schematic,
    _terminals,
)


def settings(mask: Mask | None) -> str:
    """Settings of a subsystem, as saved by PLECS, and its mask."""
    text = (
        '      SampleTime    "-1"\n'
        '      CodeGenDiscretizationMethod "2"\n'
        '      CodeGenTarget "Generic"\n'
    )
    if mask is not None:
        text += (
            f"      MaskType      {_q(mask.type)}\n"
            f"      MaskDescription {_q(mask.description)}\n"
        )
        text += f"      MaskInit      {_q(mask.init)}\n" if mask.init else ""
    text += (
        "      MaskIconFrame on\n      MaskIconOpaque off\n      MaskIconRotates on\n"
    )
    if mask is not None:
        text += "".join(_mask_param(m.variable, m.prompt, m.value) for m in mask.params)
    return text


def frame(b: Placed) -> str:
    """Frame of a block, centered at its position."""
    x0, y0, x1, y1 = (v - c for v, c in zip(b.rect, (b.x, b.y, b.x, b.y), strict=True))
    return f"      Frame         [{x0}, {y0}; {x1}, {y1}]\n"


def terminals(b: Placed) -> str:
    """Terminals of a subsystem: the inputs on the left and the outputs on the right."""
    x0, _, x1, _ = (v - c for v, c in zip(b.rect, (b.x, b.y, b.x, b.y), strict=True))
    return _terminals(
        [("Input", x0, round(dy), "left") for dy in b.in_dy]
        + [("Output", x1 + b.stub, round(dy), "right") for dy in b.out_dy]
    )


def _ints(values: list[int]) -> str:
    """Vector of integers."""
    return "[" + " ".join(str(v) for v in values) + "]"


def schematic(sub: Subsystem) -> tuple[_Schematic, Point]:
    """Schematic of a subsystem (the contents), and its size."""
    lay = layout(sub, PLECS)
    sch = _Schematic()
    n_in = len(sub.inputs)
    for name, b in lay.blocks.items():
        pos = (b.x, b.y)
        if name in lay.froms or name in lay.gotos:
            tag = lay.froms[name][0] if name in lay.froms else lay.gotos[name]
            typ = "From" if name in lay.froms else "Goto"
            params = {"Tag": tag, "Visibility": "1"}
            sch.component(typ, name, pos, params, show=False)
        elif b.kind == "input":
            k = [p.name for p in sub.inputs].index(name)
            params = {"Index": str(k + 1), "Width": "-1"}
            sch.component("Input", name, pos, params)
        elif b.kind == "output":
            k = [p.name for p in sub.outputs].index(name)
            params = {"Index": str(n_in + k + 1), "Width": "-1"}
            sch.component("Output", name, pos, params)
        else:
            _block(sch, sub.block(name), b)
    nets: dict[str, list] = {}
    for w in lay.wires:
        nets.setdefault(w.src, []).append(w)

    def terminal(ref: str, out: bool) -> Terminal:
        name, port = ref.split(":", 1) if ":" in ref else (ref, "")
        b = lay.blocks[name]
        if b.kind in ("input", "output", "from", "goto"):
            return name, 1
        k = b.outputs.index(port) if out else b.inputs.index(port)
        if b.kind == "selector":
            return name, 2 if out else 1
        if b.kind == "mux":
            return name, 1 if out else k + 2
        return name, k + 1 + (len(b.inputs) if out else 0)

    def convert(t: Tree) -> Branches:
        branches = [convert(b) for b in t.branches]
        return t.points, branches, terminal(t.dst, False) if t.dst else None

    for src, wires in nets.items():
        sch.tree(terminal(src, True), "Signal", convert(tree(wires)))
    return sch, lay.size


def _block(
    sch: _Schematic, b: CBlock | Selector | Mux | Tag | Subsystem, p: Placed
) -> None:
    """Add a block of the netlist."""
    pos = (p.x, p.y)
    if isinstance(b, CBlock) and b.mask is None:
        feedthrough = b.feedthrough or [1] * len(b.inputs)
        cscript = _cscript(
            b.code,
            _ints([q.width for q in b.inputs]),
            _ints([q.width for q in b.outputs]),
            ", ".join(b.params),
            b.sample_time or "-1",  # Inherited
            feedthrough=_ints(feedthrough),
        )
        sch.component("CScript", b.name, pos, cscript, direction="up", extra=frame(p))
    elif isinstance(b, (CBlock, Subsystem)):
        inner, size = schematic(b if isinstance(b, Subsystem) else _masked(b))
        sch.component(
            "Subsystem",
            b.name,
            pos,
            direction="up",
            extra=frame(p) + settings(b.mask),
            trailer=terminals(p) + _inner_schematic(inner, size),
        )
    elif isinstance(b, Tag):
        params = {"Tag": b.tag, "Visibility": "1"}
        sch.component("Goto" if b.goto else "From", b.name, pos, params, show=False)
    elif isinstance(b, Selector):
        params = {
            "InputWidth": str(b.width),
            "OutputIndices": _ints([k + 1 for k in b.indices]),
        }
        sch.component("SignalSelector", b.name, pos, params)
    else:
        params = {"Width": _ints([q.width for q in b.inputs])}
        sch.component("SignalMux", b.name, pos, params, show=False)


def _masked(b: CBlock) -> Subsystem:
    """Contents of the masked subsystem of a C block: the C-Script block."""
    cscript = replace(b, name="C-Script", mask=None)
    first = b.outputs[0].name
    return Subsystem(
        b.name,
        b.inputs,
        b.outputs,
        [cscript],
        [(p.name, f"C-Script:{p.name}") for p in b.inputs]
        + [(f"C-Script:{p.name}", p.name) for p in b.outputs],
        columns=[[p.name for p in b.inputs], ["C-Script"], [p.name for p in b.outputs]],
        align={first: f"C-Script:{first}"},
    )
