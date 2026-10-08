"""
PLECS file format: formatting of values and the schematic writer.

The functions in this module know nothing about motulator; they write components,
connections, probes, scopes, masks, and C-Script parameters in the syntax of a PLECS
model file.

"""

import base64
from math import isinf
from typing import Any, cast

import numpy as np


# %%
def _fmt(value: Any) -> str:
    """Format a value for the PLECS (Octave) workspace."""
    if value is None:
        return "[]"
    if isinstance(value, str):
        return value
    if isinstance(value, np.ndarray) and value.ndim == 2:
        rows = "; ".join(" ".join(_fmt(v) for v in row) for row in value)
        return f"[{rows}]"
    if isinstance(value, (list, tuple, np.ndarray)):
        return "[" + " ".join(_fmt(v) for v in np.asarray(value).ravel()) + "]"
    value = float(value)
    if isinf(value):
        return "inf" if value > 0 else "-inf"
    return repr(value)


def _q(text: str) -> str:
    """Quote a string for the PLECS file format."""
    text = text.replace("\\", "\\\\").replace('"', '\\"')
    return '"' + text.replace("\n", "\\n").replace("\t", "\\t") + '"'


Point = tuple[int, int]
Terminal = tuple[str, int]
Tap = tuple[list[Point], list[tuple[Terminal, list[Point]]]]
# Branched connection: (points, branches, destination), see _Schematic.tree
Branches = tuple[list[Point], list[Any], Terminal | None]


def _points(points: list[Point], indent: str) -> str:
    if not points:
        return ""
    xy = "; ".join(f"{x}, {y}" for x, y in points)
    return f"{indent}Points        [{xy}]\n"


class _Schematic:
    """
    Minimal writer for the components and connections of a PLECS schematic.

    Connections from the same source terminal are written as branches. The points
    of a connection are the corner points of the wire, excluding the terminals. For
    a branched connection, the trunk points lead from the source to the branching
    point, and the branch points from the branching point to the destination. A bus
    is a connection with several branching points in a row, see `bus`.

    """

    def __init__(self) -> None:
        self.items: list[str] = []
        self.connections: dict[
            tuple[Terminal, str], list[tuple[Terminal, list[Point]]]
        ] = {}
        self.trunks: dict[tuple[Terminal, str], list[Point]] = {}
        self.buses: list[tuple[Terminal, str, list[Tap]]] = []
        self.trees: list[tuple[Terminal, str, Branches]] = []
        self.dx = 0  # Horizontal shift of the components and points added afterwards

    def _shift(self, points: list[Point] | None) -> list[Point]:
        return [(x + self.dx, y) for x, y in points or []]

    def component(
        self,
        typ: str,
        name: str,
        pos: Point,
        params: dict[str, str] | None = None,
        direction: str = "right",
        flipped: bool = False,
        extra: str = "",
        show: bool = True,
        label: str = "south",
        src_component: str | None = None,
        trailer: str = "",
    ) -> None:
        """Add a component."""
        s = f"    Component {{\n      Type          {typ}\n"
        if src_component is not None:
            s += f"      SrcComponent  {_q(src_component)}\n"
        s += (
            f"      Name          {_q(name)}\n"
            f"      Show          {'on' if show else 'off'}\n"
            f"      Position      [{pos[0] + self.dx}, {pos[1]}]\n"
            f"      Direction     {direction}\n"
            f"      Flipped       {'on' if flipped else 'off'}\n"
            f"      LabelPosition {label}\n"
        )
        s += extra
        for var, val in (params or {}).items():
            s += (
                "      Parameter {\n"
                f"        Variable      {_q(var)}\n"
                f"        Value         {_q(val)}\n"
                "        Show          off\n"
                "      }\n"
            )
        self.items.append(s + trailer + "    }\n")

    def connect(
        self, src: Terminal, dst: Terminal, typ: str, points: list[Point] | None = None
    ) -> None:
        """Add a connection. Connections from the same source become branches."""
        self.connections.setdefault((src, typ), []).append((dst, self._shift(points)))

    def trunk(self, src: Terminal, typ: str, points: list[Point]) -> None:
        """Set the trunk points of a branched connection."""
        self.trunks[(src, typ)] = self._shift(points)

    def bus(self, src: Terminal, typ: str, taps: list[Tap]) -> None:
        """
        Add a connection with several branching points (a bus).

        Each tap is `(points, dsts)`: the points lead from the previous tap (or from
        the source terminal) to the branching point, and `dsts` lists the terminals
        branching from it, each with the points leading towards the terminal (empty
        for a straight line). The bus continues from each tap to the next one.

        """
        taps = [
            (self._shift(pts), [(dst, self._shift(lead)) for dst, lead in dsts])
            for pts, dsts in taps
        ]
        self.buses.append((src, typ, taps))

    def tree(self, src: Terminal, typ: str, tree: "Branches") -> None:
        """
        Add a branched connection given as a tree.

        The tree is `(points, branches, dst)`: the points from the start (the source
        terminal or the branching point), and the branches (trees from the end of
        the points) or the destination terminal (if there are no branches).

        """
        self.trees.append((src, typ, tree))

    def _tree(self, tree: "Branches", ind: str) -> str:
        points, branches, dst = tree
        text = _points(points, ind)
        if not branches:
            return text + _dst(cast(Terminal, dst), ind)
        for b in branches:
            text += f"{ind}Branch {{\n{self._tree(b, ind + '  ')}{ind}}}\n"
        return text

    def _bus(self, taps: list[Tap], i: int, ind: str) -> str:
        points, dsts = taps[i]
        if len(dsts) == 1 and i + 1 == len(taps):
            dst, lead = dsts[0]
            return _points(points + lead, ind) + _dst(dst, ind)
        text = _points(points, ind)
        for dst, lead in dsts:
            branch = _points(lead, ind + "  ") + _dst(dst, ind + "  ")
            text += f"{ind}Branch {{\n{branch}{ind}}}\n"
        if i + 1 < len(taps):
            text += f"{ind}Branch {{\n{self._bus(taps, i + 1, ind + '  ')}{ind}}}\n"
        return text

    def render(self) -> str:
        """Render the components and connections."""
        text = "".join(self.items)
        for (src, typ), dsts in self.connections.items():
            trunk = self.trunks.get((src, typ), [])
            text += _connection(src, typ)
            if len(dsts) == 1:
                (dst, points) = dsts[0]
                text += _points(trunk + points, "      ") + _dst(dst, "      ")
            else:
                text += _points(trunk, "      ")
                for dst, points in dsts:
                    branch = _points(points, "        ") + _dst(dst, "        ")
                    text += f"      Branch {{\n{branch}      }}\n"
            text += "    }\n"
        for src, typ, taps in self.buses:
            text += _connection(src, typ) + self._bus(taps, 0, "      ") + "    }\n"
        for src, typ, tree in self.trees:
            text += _connection(src, typ) + self._tree(tree, "      ") + "    }\n"
        return text

    def signal(
        self, src: Terminal, dst: Terminal, points: list[Point] | None = None
    ) -> None:
        """Add a signal connection."""
        self.connect(src, dst, "Signal", points)

    def wire(
        self, src: Terminal, dst: Terminal, points: list[Point] | None = None
    ) -> None:
        """Add an electrical connection."""
        self.connect(src, dst, "Wire", points)


def _connection(src: Terminal, typ: str) -> str:
    return (
        "    Connection {\n"
        f"      Type          {typ}\n"
        f"      SrcComponent  {_q(src[0])}\n"
        f"      SrcTerminal   {src[1]}\n"
    )


def _dst(dst: Terminal, ind: str) -> str:
    return f"{ind}DstComponent  {_q(dst[0])}\n{ind}DstTerminal   {dst[1]}\n"


# %%
def _probe(component: str, signals: list[str], path: str = "") -> str:
    sig = ", ".join(_q(s) for s in signals)
    return (
        "      Probe {\n"
        f"        Component     {_q(component)}\n"
        f"        Path          {_q(path)}\n"
        f"        Signals       {{{sig}}}\n"
        "      }\n"
    )


def _scope(axes: list[tuple[str, str]]) -> str:
    extra = (
        "      Location      [100, 100; 800, 700]\n"
        f'      Axes          "{len(axes)}"\n'
        '      TimeRange     "0"\n'
        '      ScrollingMode "1"\n'
        '      SingleTimeAxis "1"\n'
        '      Open          "0"\n'
        '      Ts            "-1"\n'
        '      SampleLimit   "0"\n'
        '      XAxisLabel    "Time (s)"\n'
        '      ShowLegend    "1"\n'
    )
    for name, label in axes:
        extra += (
            "      Axis {\n"
            f"        Name          {_q(name)}\n"
            "        AutoScale     1\n"
            "        MinValue      0\n"
            "        MaxValue      1\n"
            "        Signals       {}\n"
            "        SignalTypes   [ ]\n"
            f"        AxisLabel     {_q(label)}\n"
            "        Untangle      0\n"
            "        KeepBaseline  off\n"
            "        BaselineValue 0\n"
            "      }\n"
        )
    return extra


def _cscript(
    code: dict[str, str],
    num_inputs: str,
    num_outputs: str,
    parameters: str,
    ts: str,
    num_disc_states: int = 0,
    num_cont_states: int = 0,
    feedthrough: str = "1",
) -> dict[str, str]:
    """Parameters of a C-Script block."""
    return {
        "DialogGeometry": "",
        "NumInputs": num_inputs,
        "NumOutputs": num_outputs,
        "NumContStates": str(num_cont_states),
        "NumDiscStates": str(num_disc_states),
        "NumZCSignals": "0",
        "DirectFeedthrough": feedthrough,
        "Ts": ts,
        "Parameters": parameters,
        "LangStandard": "2",
        "GnuExtensions": "2",
        "RuntimeCheck": "2",
        "Declarations": code.get("Declarations", ""),
        "StartFcn": code.get("StartFcn", ""),
        "OutputFcn": code.get("OutputFcn", ""),
        "UpdateFcn": code.get("UpdateFcn", ""),
        "DerivativeFcn": code.get("DerivativeFcn", ""),
        "TerminateFcn": "",
        "StoreCustomStateFcn": "",
        "RestoreCustomStateFcn": "",
    }


def _terminals(terminals: list[tuple[str, int, int, str]]) -> str:
    """Terminals of a component: (type, x, y, direction)."""
    return "".join(
        "      Terminal {\n"
        f"        Type          {typ}\n"
        f"        Position      [{x}, {y}]\n"
        f"        Direction     {d}\n"
        "      }\n"
        for typ, x, y, d in terminals
    )


def _inner_schematic(sub: _Schematic, size: Point) -> str:
    """Schematic block of a subsystem."""
    return (
        "      Schematic {\n"
        f"        Location      [0, 0; {size[0]}, {size[1]}]\n"
        "        ZoomFactor    1\n"
        "        SliderPosition [0, 0]\n"
        "        ShowBrowser   off\n"
        "        BrowserWidth  100\n" + sub.render() + "      }\n"
    )


def _mask_probes(probes: list[tuple[str, str, str]], path: str = "") -> str:
    """
    Probe signals of a masked subsystem: (name, component, signal), the components
    in the subsystem `path` of the masked subsystem.
    """
    return "".join(
        "      MaskProbe {\n"
        f"        Name          {_q(name)}\n"
        "        Probe {\n"
        f"          Component     {_q(comp)}\n"
        f"          Path          {_q(path)}\n"
        f"          Signals       {{{_q(signal)}}}\n"
        "        }\n"
        "      }\n"
        for name, comp, signal in probes
    )


def _mask_param(variable: str, prompt: str, value: str, tab: str = "") -> str:
    """Mask parameter definition of a subsystem (a non-ASCII prompt in base64)."""
    text = (
        _q(prompt)
        if prompt.isascii()
        else "base64 " + _q(base64.b64encode(prompt.encode()).decode())
    )
    return (
        "      Parameter {\n"
        f"        Variable      {_q(variable)}\n"
        f"        Prompt        {text}\n"
        "        Type          FreeText\n"
        f"        Value         {_q(value)}\n"
        "        Show          off\n"
        # Tunable, since non-tunable parameters are inlined as constants, which
        # makes accessing an empty parameter (None) a compilation error
        "        Tunable       on\n"
        f"        TabName       {_q(tab)}\n"
        "      }\n"
    )


def _mask(mask_type: str, description: str, display: str) -> str:
    """Mask of a subsystem, including the motulator label in the icon."""
    return (
        '      SampleTime    "-1"\n'
        '      CodeGenDiscretizationMethod "2"\n'
        '      CodeGenTarget "Generic"\n'
        f"      MaskType      {_q(mask_type)}\n"
        f"      MaskDescription {_q(description)}\n"
        '      MaskDisplayLang "2"\n'
        f"      MaskDisplay   {_q(display)}\n"
        "      MaskIconFrame off\n"
        "      MaskIconOpaque off\n"
        "      MaskIconRotates on\n"
    )
