"""
Building blocks of the PLECS models: the schematic writer, the control-system
block, the converter, the DC bus, and the simulation via the RPC interface.

"""

import base64
import os
import xmlrpc.client
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from math import isinf, pi
from pathlib import Path
from typing import Any

import numpy as np
from motulator.common.model._converter import (
    CapacitiveDCBusConverter,
    FrequencyConverter,
)
from motulator.drive.model import Drive

# Directory of the C sources, written into the models as a path relative to the
# model file (the placeholder C_DIR in the C-Script code)
C_SOURCES = Path(__file__).parent / "c"
C_DIR = "$(C_DIR)"


# %%
@dataclass
class StepSignal:
    """
    Step signal: `before` at first and `after` at `t > time`.

    Several steps are given as sequences: `after[k]` is the value after the step at
    `time[k]`. The signal is callable, as the reference functions of motulator.

    """

    time: float | Sequence[float]
    after: float | Sequence[float]
    before: float = 0.0

    def __call__(self, t: Any) -> Any:
        """Value at the time `t` (a float or an array)."""
        levels = [self.before, *np.atleast_1d(self.after)]
        times = np.atleast_1d(self.time)
        return self.before + sum(
            (t > t_k) * (levels[k + 1] - levels[k]) for k, t_k in enumerate(times)
        )


def sampled_step(sig: StepSignal, T_s: float) -> StepSignal:
    """
    Step signal that switches at the same sample as a reference of motulator.

    The controller of motulator evaluates the reference `(t > sig.time)` at its time
    `t`, which is accumulated by adding the sampling period. Due to rounding, the
    switching sample may differ from that of a PLECS Step block at `sig.time`. This
    returns a step switching halfway between the samples, as in motulator.

    """
    times = []
    for time in np.atleast_1d(sig.time):
        t, k = 0.0, 0
        while not t > time:
            t = (t + T_s) % 1e9  # As in ControlSystem.update of motulator
            k += 1
        times.append((k - 0.5) * T_s)
    time = times[0] if np.isscalar(sig.time) else times
    return StepSignal(time=time, after=sig.after, before=sig.before)


@dataclass
class MaskParam:
    """Mask parameter of the control system."""

    variable: str  # Variable name in the mask workspace
    prompt: str  # Prompt in the mask dialog
    tab: str  # Tab in the mask dialog
    target: str  # C assignment target, or "" if handled separately
    required: bool = False  # Whether an empty value is an error


@dataclass
class ControlBlock:
    """
    Control-system block: a masked subsystem containing a C-Script block.

    The mask parameters are passed to the C-Script, whose inputs are the references
    and the measurements. The first output of the C-Script, the duty ratios, is the
    only output of the block. The monitored signals are the other outputs of the
    C-Script, available as mask probes of the block.

    """

    name: str  # Name of the block (the control method)
    mask_type: str
    description: str
    mask_params: list[MaskParam]
    inputs: list[str]  # Input ports
    input_widths: list[int]
    outputs: dict[str, list[str]]  # Monitored signals: probe name -> signal names
    code: Callable[[], dict[str, str]]  # Code sections of the C-Script
    mask_init: str = ""  # Mask initialization commands
    cscript_params: list[str] | None = None  # Defaults to the mask variables

    @property
    def signals(self) -> list[str]:
        """Names of the monitored signals."""
        return [n for v in self.outputs.values() for n in v]


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


# Common C-Script declarations
C_PARAMS = (
    "#define P(i, j) ParamRealData(i, j)\n"
    "#define PDIM(i) (ParamDim(i, 0) * ParamDim(i, 1))\n"
    "\n"
    "/* Parameter value, or NAN for an empty parameter (None in motulator) */\n"
    "#define PARAM(i) (PDIM(i) > 0 ? P(i, 0) : NAN)\n"
    "\n"
    "/* Read a GradNet from the parameters i0, i0 + 1, ... (GRADNET_FIELDS) */\n"
    "#define READ_GRADNET(net, i0) \\\n"
    "    do { \\\n"
    "        (net).in_dim = (int)P((i0), 0); \\\n"
    "        (net).mu_dim = (int)P((i0) + 1, 0); \\\n"
    "        (net).embed_dim = PDIM((i0) + 3); \\\n"
    "        for (int j_ = 0; j_ < (net).embed_dim; j_++) { \\\n"
    "            for (int k_ = 0; k_ < (net).in_dim; k_++) { \\\n"
    "                (net).W[j_][k_] = P((i0) + 2, j_ + (net).embed_dim * k_); \\\n"
    "            } \\\n"
    "            (net).b[j_] = P((i0) + 3, j_); \\\n"
    "        } \\\n"
    "        for (int k_ = 0; k_ < (net).in_dim; k_++) { \\\n"
    "            (net).mu_log[k_] = (k_ < (net).mu_dim) ? P((i0) + 4, k_) : 0.0; \\\n"
    "            (net).bias[k_] = P((i0) + 5, k_); \\\n"
    "        } \\\n"
    "        (net).activation = (GradNetActivation)(int)P((i0) + 6, 0); \\\n"
    "        (net).beta_log = P((i0) + 7, 0); \\\n"
    "        (net).p = (int)P((i0) + 8, 0); \\\n"
    "        (net).in_base = P((i0) + 9, 0); \\\n"
    "        (net).out_base = P((i0) + 10, 0); \\\n"
    "    } while (0)\n"
)


def parameter_checks(mask_params: list[MaskParam]) -> str:
    """C code checking that the required mask parameters are scalars."""
    return "".join(
        f"if (PDIM({k}) != 1) {{\n"
        f'    SetErrorMessage("{m.variable} must be a scalar.");\n'
        "    return;\n"
        "}\n"
        for k, m in enumerate(mask_params)
        if m.required
    )


def cfg_assignments(mask_params: list[MaskParam]) -> str:
    """C code assigning the non-empty mask parameters to the configuration."""
    return "".join(
        f"if (PDIM({k}) > 0) {{\n    {m.target} = P({k}, 0);\n}}\n"
        for k, m in enumerate(mask_params)
        if m.target.startswith("cfg.")
    )


# Positions of the control system (inputs at x - 50, output at x + 54), the
# converter, and the machine (all centered at the same height), and the DC rails
# and the riser of the converter control signal
CS = (250, 100)
CONV = (620, 100)
MACH = (740, 100)
Y_TOP, Y_BOT = 40, 160
X_RISER = 500
SRC = {"DiscretizationBehavior": "2", "StateSpaceInlining": "1"}  # Controlled sources


def _jogs(
    y_src: list[int], y_dst: list[int], x0: int, step: int, fan_in: bool = True
) -> list[int]:
    """
    Positions of the vertical segments of wires from `y_src` to `y_dst` (same order).

    The wires do not cross when the longest vertical segment is the outermost for a
    fan-in (sources spread wider than the destinations) and the innermost for a
    fan-out. Among equal lengths, the wire farther from the destinations is outer.

    """

    def key(k: int) -> tuple[int, int]:
        up = y_dst[k] < y_src[k]
        return abs(y_dst[k] - y_src[k]), y_src[k] if up else -y_src[k]

    order = sorted(range(len(y_src)), key=key, reverse=not fan_in)
    x = [0] * len(y_src)
    for rank, k in enumerate(order):
        x[k] = x0 + step * rank
    return x


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


def _mask_probes(probes: list[tuple[str, str, str]]) -> str:
    """Probe signals of a masked subsystem: (name, component, signal)."""
    return "".join(
        "      MaskProbe {\n"
        f"        Name          {_q(name)}\n"
        "        Probe {\n"
        f"          Component     {_q(comp)}\n"
        '          Path          ""\n'
        f"          Signals       {{{_q(signal)}}}\n"
        "        }\n"
        "      }\n"
        for name, comp, signal in probes
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


# Converter: phases a, b, and c on the right, the gate input on the top, and the DC
# terminals on the top and bottom
CONVERTER_FRAME = "      Frame         [-25, -25; 25, 25]\n"


def _add_pwm(sch: _Schematic, T_s: float) -> None:
    """
    Add the carrier comparison (PLECS Symmetrical PWM), fed by the delayed duty ratios.

    The symmetrical triangular carrier with the period 2*T_s, starting from its
    maximum, and the regular sampling at both the maximum and the minimum correspond
    to CarrierComparison in motulator. The counter quantization of motulator is not
    modeled. The output values -1 and 1 are the gate signals of the ideal converter.

    """
    params = {
        "sampling": "3",  # Regular (double update)
        "fc": _fmt(0.5 / T_s),
        "carrier_phaseshift": "0",
        "carrier_limits": "[0 1]",
        "output_values": "[-1 1]",
    }
    sch.component(
        "Reference",
        "PWM",
        (450, CS[1]),
        params,
        direction="up",  # The native orientation of the library block
        src_component="Components/Control/Modulators/Symmetrical PWM",
        extra="      Frame         [-20, -20; 20, 20]\n",
        trailer=_terminals([("Output", 24, 0, "right"), ("Input", -20, 0, "left")]),
    )
    sch.signal(("Delay", 2), ("PWM", 2))


def _add_delay(sch: _Schematic, T_s: float, block: ControlBlock) -> None:
    """Add the computational delay of one sampling period (PLECS Delay block)."""
    sch.component(
        "Delay", "Delay", (370, CS[1]), {"N": "1", "X0": "0", "Ts": _fmt(T_s)}
    )
    sch.signal((block.name, len(block.inputs) + 1), ("Delay", 1))


def _add_converter(sch: _Schematic) -> None:
    """Add the ideal two-level converter (PLECS), fed by the gate signals."""
    sch.component(
        "Reference",
        "Converter",
        CONV,
        # Mirrored so that the AC terminals are on the right, as in the machine
        direction="down",
        flipped=True,
        label="south",
        # Ideal converter: the phase is connected to the positive DC terminal upon a
        # positive gate signal and else to the negative terminal, as in motulator
        src_component="Components/Electrical/Converters/2-Level\\nConv.",
        extra=CONVERTER_FRAME,
        trailer=_terminals(
            [
                ("Port", -30, -10, "left"),
                ("Port", -30, 0, "left"),
                ("Port", -30, 10, "left"),
                ("Input", -15, -25, "up"),
                ("Port", 0, -30, "up"),
                ("Port", 0, 30, "down"),
            ]
        ),
    )
    # The gate signals are routed above the DC bus, the riser staying next to the PWM
    x = X_RISER - sch.dx
    sch.signal(("PWM", 1), ("Converter", 4), [(x, CS[1]), (x, 20), (CONV[0] + 15, 20)])


def _add_dc_bus(sch: _Schematic, mdl: Drive) -> None:
    """
    Add the DC bus: a stiff voltage, a capacitor, or a diode bridge and LC.

    The circuits are not grounded (as in motulator, there is no zero sequence).

    """
    conv = mdl.converter
    x_c, y_c = CONV
    sch.component("Voltmeter", "u_dc", (580, y_c), direction="up", label="west")
    if isinstance(conv, FrequencyConverter):
        _add_diode_bridge(sch)
        return
    if isinstance(conv, CapacitiveDCBusConverter):
        dc, typ, params = "C_dc", "Capacitor", {"C": "converter.C_dc"}
        params["v_init"] = "converter.u_dc"
    else:
        dc, typ, params = "V_dc", "DCVoltageSource", {"V": "converter.u_dc"}
    sch.component(typ, dc, (540, y_c), params, direction="up", label="west")
    # Positive and negative rails via the voltmeter to the converter
    for terminal, y in ((1, Y_TOP), (2, Y_BOT)):
        taps: list[Tap] = [([(540, y), (580, y)], [(("u_dc", terminal), [])])]
        taps += [([(x_c, y)], [(("Converter", 4 + terminal), [])])]
        sch.bus((dc, terminal), "Wire", taps)


def _add_diode_bridge(sch: _Schematic) -> None:
    """Add the grid, the diode bridge, the DC inductor, and the DC capacitor."""
    x_c, y_c = CONV
    x_legs = [330, 370, 410]
    for k, ph in enumerate("abc"):
        x = x_legs[k]
        for sign, y in (("+", y_c - 30), ("-", y_c + 30)):
            sch.component(
                "Diode",
                f"D{ph}{sign}",
                (x, y),
                {"Vf": "0", "Ron": "0"},
                direction="up",
                flipped=True,
                show=False,
            )
        # Grid phase voltage u_g*cos(w_g*t - k*2*pi/3), i.e., exp_j_theta_g(0) = 1
        sch.component(
            "ACVoltageSource",
            f"u_g{ph}",
            (240, y_c - 40 + 40 * k),
            {"V": "converter.u_g", "w": "converter.w_g", "phi": f"pi/2 - {k}*2*pi/3"},
            label="north",
            show=k == 0,
        )
        # The midpoint of the leg, at a different height for each phase
        y = y_c - 10 + 10 * k
        lead = [[(300, y_c - 40), (300, y)], [], [(310, y_c + 40), (310, y)]][k]
        dsts = [((f"D{ph}+", 1), []), ((f"D{ph}-", 2), [])]
        sch.bus((f"u_g{ph}", 1), "Wire", [(lead + [(x, y)], dsts)])
    # Star point of the grid (not grounded)
    y_a, y_b, y_g = y_c - 40, y_c, y_c + 40
    taps: list[Tap] = [([(200, y_a), (200, y_b)], [(("u_gb", 2), [])])]
    sch.bus(("u_ga", 2), "Wire", taps + [([(200, y_g)], [(("u_gc", 2), [])])])
    # DC bus: the cathodes of the upper diodes to the inductor, the anodes of the
    # lower diodes to the negative rail
    sch.component(
        "Inductor",
        "L_dc",
        (450, Y_TOP),
        {"L": "converter.L_dc", "i_init": "0"},
        direction="left",
    )
    sch.component(
        "Capacitor",
        "C_dc",
        (540, y_c),
        {"C": "converter.C_dc", "v_init": "converter.u_dc"},
        direction="up",
        label="west",
    )
    legs = [
        ([(x, Y_TOP)], [((f"D{ph}+", 2), [])])
        for x, ph in zip(x_legs, "abc", strict=True)
    ]
    sch.bus(("L_dc", 1), "Wire", legs[::-1])
    taps = [
        ([(540, Y_TOP)], [(("C_dc", 1), [])]),
        ([(580, Y_TOP)], [(("u_dc", 1), [])]),
    ]
    sch.bus(("L_dc", 2), "Wire", taps + [([(x_c, Y_TOP)], [(("Converter", 5), [])])])
    taps = [([(x_c, Y_BOT), (580, Y_BOT)], [(("u_dc", 2), [])])]
    taps += [([(540, Y_BOT)], [(("C_dc", 2), [])])]
    taps += [
        ([(x, Y_BOT)], [((f"D{ph}-", 1), [])])
        for x, ph in zip(x_legs[::-1], "cba", strict=True)
    ]
    sch.bus(("Converter", 6), "Wire", taps)


def _step(sig: StepSignal) -> dict[str, str]:
    """Parameters of a Step block; several steps as vectors, to be summed."""
    if np.isscalar(sig.time):
        params = {"Time": sig.time, "Before": sig.before, "After": sig.after}
    else:
        levels = [sig.before, *np.atleast_1d(sig.after)]
        params = {
            "Time": sig.time,
            "Before": [sig.before] + [0.0] * (len(levels) - 2),
            "After": np.diff(levels),
        }
    return {k: _fmt(v) for k, v in params.items()} | {"DataType": "10"}


# Layout of the schematic (x to the right, y downwards). The control system is on
# the left, the converter and the machine on the right, and the scope at the bottom.
def _fmt_mask(value: Any) -> str:
    """Format a mask value, writing multiples of 2*pi as in motulator."""
    if isinstance(value, float) and value > 0 and not isinf(value):
        k = value / (2 * pi)
        if k == round(k, 6) and 2 * pi * round(k, 6) == value:
            return f"2*pi*{round(k, 6):g}"
    return _fmt(value)


def _mask_parameter(m: MaskParam, value: Any) -> str:
    """Mask parameter definition of a subsystem."""
    prompt = (
        _q(m.prompt)
        if m.prompt.isascii()
        else "base64 " + _q(base64.b64encode(m.prompt.encode()).decode())
    )
    return (
        "      Parameter {\n"
        f"        Variable      {_q(m.variable)}\n"
        f"        Prompt        {prompt}\n"
        "        Type          FreeText\n"
        f"        Value         {_q(_fmt_mask(value))}\n"
        "        Show          off\n"
        # Tunable, since non-tunable parameters are inlined as constants, which
        # makes accessing an empty parameter (None) a compilation error
        "        Tunable       on\n"
        f"        TabName       {_q(m.tab)}\n"
        "      }\n"
    )


def _control_subsystem(block: ControlBlock, values: dict[str, Any]) -> tuple[str, str]:
    """Mask and contents of the control-system subsystem."""
    n_in = len(block.inputs)
    header = (
        "      Frame         [-50, -30; 50, 30]\n"
        '      SampleTime    "-1"\n'
        '      CodeGenDiscretizationMethod "2"\n'
        '      CodeGenTarget "Generic"\n'
        f"      MaskType      {_q(block.mask_type)}\n"
        f"      MaskDescription {_q(block.description)}\n"
        + (f"      MaskInit      {_q(block.mask_init)}\n" if block.mask_init else "")
        + "      MaskIconFrame on\n"
        "      MaskIconOpaque off\n"
        "      MaskIconRotates on\n"
        + "".join(_mask_parameter(m, values[m.variable]) for m in block.mask_params)
    )
    # The duty ratios are the only output; the monitored signals are mask probes
    terminals = _terminals(
        [
            *[("Input", -50, 10 * k - 5 * (n_in - 1), "left") for k in range(n_in)],
            ("Output", 54, 0, "right"),
        ]
    )

    # Contents: input ports, C-Script, and the output port
    sub = _Schematic()
    y_cs = 100  # C-Script position, inputs spaced by 10 around it
    y_src = [40 + 30 * k for k in range(n_in)]
    y_in = [y_cs + 10 * k - 5 * (n_in - 1) for k in range(n_in)]
    x_jog = _jogs(y_src, y_in, 100, 8)
    for k, name in enumerate(block.inputs):
        sub.component(
            "Input", name, (60, y_src[k]), {"Index": str(k + 1), "Width": "-1"}
        )
        points = [(x_jog[k], y_src[k]), (x_jog[k], y_in[k])]
        sub.signal((name, 1), ("C-Script", k + 1), points)
    params = block.cscript_params or [m.variable for m in block.mask_params]
    widths = " ".join(str(w) for w in block.input_widths)
    n_out = " ".join(str(len(v)) for v in block.outputs.values())
    cscript = _cscript(
        block.code(), f"[{widths}]", f"[3 {n_out}]", ", ".join(params), "T_s"
    )
    sub.component(
        "CScript",
        "C-Script",
        (200, y_cs),
        cscript,
        direction="up",
        extra="      Frame         [-50, -60; 50, 60]\n",
    )
    # The port index runs over both the inputs and the outputs
    y_out = y_cs - 5 * len(block.outputs)  # First output of the C-Script
    sub.component(
        "Output", "d_abc", (340, y_out), {"Index": str(n_in + 1), "Width": "-1"}
    )
    sub.signal(("C-Script", n_in + 1), ("d_abc", 1))
    # Probe signals of the masked subsystem (the internals cannot be probed directly)
    probes = [("Duty ratios (d_abc)", "C-Script", "Output 1")]
    probes += [(n, "C-Script", f"Output {k + 2}") for k, n in enumerate(block.outputs)]
    return header, terminals + _inner_schematic(sub, (500, 250)) + _mask_probes(probes)


def _add_control_system(
    sch: _Schematic,
    block: ControlBlock,
    sources: list[tuple[str, StepSignal | float | str]],
    values: dict[str, Any],
) -> None:
    """
    Add the control system and its inputs (references and measurements).

    Each source is a `(name, signal)` pair in the order of the block inputs, the
    signal being a `StepSignal`, a constant, or a probe definition (`_probe`). The
    steps of a `StepSignal` with several steps are summed by a Gain block.

    """
    n_in = len(sources)
    y_src = [CS[1] - 20 * (n_in - 1) + 40 * k for k in range(n_in)]
    y_in = [CS[1] + 10 * k - 5 * (n_in - 1) for k in range(n_in)]
    x_jog = _jogs(y_src, y_in, 140, 10)
    for k, (name, src) in enumerate(sources):
        out: Terminal = (name, 1)
        if isinstance(src, StepSignal):
            sch.component("Step", name, (60, y_src[k]), _step(src))
            if not np.isscalar(src.time):
                ones = _fmt([1.0] * len(np.atleast_1d(src.time)))
                gain = {"K": ones, "Multiplication": "2"}  # Matrix K*u
                sch.component("Gain", f"{name} sum", (110, y_src[k]), gain, show=False)
                sch.signal((name, 1), (f"{name} sum", 1))
                out = (f"{name} sum", 2)
        elif isinstance(src, (int, float)):
            sch.component("Constant", name, (60, y_src[k]), {"Value": _fmt(src)})
        else:
            sch.component("PlecsProbe", name, (60, y_src[k]), extra=src)
        x = x_jog[k]
        points = [] if y_src[k] == y_in[k] else [(x, y_src[k]), (x, y_in[k])]
        sch.signal(out, (block.name, k + 1), points)
    header, trailer = _control_subsystem(block, values)
    sch.component(
        "Subsystem", block.name, CS, direction="up", extra=header, trailer=trailer
    )


def _add_ctrl_output(sch: _Schematic, block: ControlBlock) -> None:
    """Add the output port of the controller signals (mask probes of the block)."""
    probe = _probe(block.name, list(block.outputs))
    sch.component("PlecsProbe", "Controller signals", (400, 340), extra=probe)
    sch.component("Output", "ctrl", (480, 340), {"Index": "2", "Width": "-1"})
    sch.signal(("Controller signals", 1), ("ctrl", 1))


def _write_model(
    path: Path,
    block: ControlBlock,
    variables: list[tuple[str, Any]],
    t_stop: float,
    T_s: float,
    sch: _Schematic,
    size: Point,
    outputs: bool,
) -> Path:
    """Write the model file: settings, workspace variables, and the schematic."""
    init = (
        "% Generated from motulator. The parameters of the control system are in\n"
        f"% the mask of the subsystem '{block.name}'.\n"
        + "".join(f"{n} = {_fmt(v)};\n" for n, v in variables)
    )
    text = (
        "Plecs {\n"
        f"  Name          {_q(path.stem)}\n"
        '  Version       "5.0"\n'
        '  CircuitModel  "ContStateSpace"\n'
        '  StartTime     "0.0"\n'
        f"  TimeSpan      {_q(_fmt(t_stop))}\n"
        '  Solver        "dopri"\n'
        f"  MaxStep       {_q(_fmt(T_s))}\n"
        '  InitStep      "-1"\n'
        '  RelTol        "1e-6"\n'
        '  AbsTol        "-1"\n'
        f"  InitializationCommands {_q(init)}\n"
        + (OUTPUT_TERMINALS if outputs else "")
        + "  Schematic {\n"
        f"    Location      [0, 0; {size[0]}, {size[1]}]\n"
        "    ZoomFactor    1\n"
        "    SliderPosition [0, 0]\n"
        "    ShowBrowser   off\n"
        "    BrowserWidth  100\n" + sch.render() + "  }\n}\n"
    )
    try:
        c_dir = Path(os.path.relpath(C_SOURCES, path.resolve().parent)).as_posix()
    except ValueError:  # Different drives (Windows)
        c_dir = C_SOURCES.as_posix()
    path.write_text(text.replace(C_DIR, c_dir))
    return path


# %%
# Output ports of the model for scripted simulations
OUTPUT_TERMINALS = "".join(
    f'  Terminal {{\n    Type          Output\n    Index         "{k}"\n  }}\n'
    for k in (1, 2)
)


def simulate_plecs(
    path: str | Path,
    t_eval: np.ndarray,
    model_vars: dict[str, float] | None = None,
    url: str = "http://localhost:1080/RPC2",
    mdl_outputs: list[str] | None = None,
    ctrl_outputs: list[str] | None = None,
) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
    """
    Simulate the PLECS model via the XML-RPC interface of PLECS Standalone.

    Parameters
    ----------
    path : str | Path
        Path of the model file.
    t_eval : ndarray
        Output times (s).
    model_vars : dict[str, float], optional
        Workspace variables to be overridden.
    url : str, optional
        URL of the RPC interface, defaults to "http://localhost:1080/RPC2".
    mdl_outputs, ctrl_outputs : list[str]
        Names of the signals in the output ports "mdl" and "ctrl".

    Returns
    -------
    mdl : dict[str, ndarray]
        System-model signals and the time "t".
    ctrl : dict[str, ndarray]
        Controller signals and the time "t".

    """
    path = Path(path).resolve()
    server = xmlrpc.client.ServerProxy(url)
    # Close the model first, since loading a model that is already open (e.g., in
    # the PLECS window) does not reload it from the file
    try:
        server.plecs.close(path.stem)
    except xmlrpc.client.Fault:
        pass
    server.plecs.load(str(path))
    opts: dict[str, Any] = {"SolverOpts": {"OutputTimes": [float(t) for t in t_eval]}}
    if model_vars:
        opts["ModelVars"] = model_vars
    try:
        res: Any = server.plecs.simulate(path.stem, opts)
    finally:
        server.plecs.close(path.stem)
    t = np.array(res["Time"])
    values = np.array(res["Values"])
    mdl_names, ctrl_names = mdl_outputs or [], ctrl_outputs or []
    n_mdl = len(mdl_names)
    if values.shape[0] != n_mdl + len(ctrl_names):
        raise RuntimeError(f"Unexpected number of output signals: {values.shape}")
    mdl = {"t": t, **dict(zip(mdl_names, values[:n_mdl], strict=True))}
    ctrl = {"t": t, **dict(zip(ctrl_names, values[n_mdl:], strict=True))}
    return mdl, ctrl
