"""
Building blocks of the PLECS models: the control-system block and its C-Script
code helpers, the converter, the DC bus, and the model file.

The PLECS file format primitives are in `_schematic`, and the simulation via the
RPC interface is in `_rpc`.

"""

import os
import re
from collections.abc import Callable
from dataclasses import dataclass
from math import isinf, pi
from pathlib import Path
from typing import Any

import numpy as np
from motulator.common.model._base import Model
from motulator.common.model._converter import DCBus, DiodeBridgeDCBus

from motulator_export._common import C_SOURCES, StepSignal, c_define
from motulator_export.plecs._netlist import Subsystem
from motulator_export.plecs._render import schematic
from motulator_export.plecs._schematic import (
    Point,
    Tap,
    Terminal,
    _fmt,
    _inner_schematic,
    _mask_param,
    _mask_probes,
    _probe,
    _q,
    _Schematic,
    _terminals,
)

# Placeholder of the directory of the C sources (C_SOURCES) in the C-Script code,
# written into the models as a path relative to the model file
C_DIR = "$(C_DIR)"


# Array sizes of the GradNet structure of the C port
GRADNET_MAX_EMBED_DIM = c_define("gradnet.h", "GRADNET_MAX_EMBED_DIM")
GRADNET_MAX_IN_DIM = c_define("gradnet.h", "GRADNET_MAX_IN_DIM")

# GradNet parameters, as fields of a workspace struct and in the C-Script parameters
GRADNET_FIELDS = [
    "in_dim",
    "mu_dim",
    "W",
    "b",
    "mu_log",
    "bias",
    "activation",
    "beta_log",
    "p",
    "in_base",
    "out_base",
]


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
    Control-system block: a masked subsystem containing the blocks of the classes of
    the control system (see `_control`).

    The inputs are `enable`, the gate enable of motulator (see `ENABLE_DESCRIPTION`),
    the references, and the measurements, including the line-to-line converter
    voltages used while the gates are blocked. The duty ratios of the C block
    "PWM/compute_output" are the only output of the block. The monitored signals are
    the outputs of the C block "Monitored signals", available as mask probes of the
    block.

    """

    name: str  # Name of the block (the control method)
    mask_type: str
    description: str
    mask_params: list[MaskParam]
    inputs: list[str]  # Input ports
    input_widths: list[int]
    outputs: dict[str, list[str]]  # Monitored signals: probe name -> signal names
    netlist: Callable[[], Subsystem]  # Contents of the subsystem
    mask_init: str = ""  # Mask initialization commands

    @property
    def signals(self) -> list[str]:
        """Names of the monitored signals."""
        return [n for v in self.outputs.values() for n in v]


# %%
# Common C-Script declarations
C_PARAMS = (
    "#define P(i, j) ParamRealData(i, j)\n"
    "#define PDIM(i) (ParamDim(i, 0) * ParamDim(i, 1))\n"
    "\n"
    "/* Parameter value, or NAN for an empty parameter (None in motulator) */\n"
    "#define PARAM(i) (PDIM(i) > 0 ? P(i, 0) : NAN)\n"
)

# C-Script declaration of the lower limit of the measured DC-bus voltage
C_U_DC_MIN = (
    "/* Lower limit (V) of the measured DC-bus voltage, which avoids the division by\n"
    " * zero in the PWM if the DC bus is not charged */\n"
    "#define U_DC_MIN 1.0\n"
)

# C-Script declarations for reading a GradNet from the parameters (_sm.py only)
C_GRADNET_PARAMS = (
    "/* Check that a GradNet in the parameters i0, i0 + 1, ... (GRADNET_FIELDS) fits\n"
    " * in the arrays of the C port (in the start function only, returns on error) */\n"
    "#define CHECK_GRADNET(i0) \\\n"
    "    do { \\\n"
    "        if ((int)P((i0), 0) > GRADNET_MAX_IN_DIM \\\n"
    "            || PDIM((i0) + 3) > GRADNET_MAX_EMBED_DIM) { \\\n"
    '            SetErrorMessage("GradNet too large: in_dim must be at most " \\\n'
    f'                            "{GRADNET_MAX_IN_DIM} and embed_dim at most " \\\n'
    f'                            "{GRADNET_MAX_EMBED_DIM}."); \\\n'
    "            return; \\\n"
    "        } \\\n"
    "    } while (0)\n"
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

# The first input of the control systems
ENABLE = "enable"
# C block of the monitored signals of a control system
MONITOR = "Monitored signals"
ENABLE_DESCRIPTION = (
    " The input enable is the gate enable (ControlSystem.set_enable): while it is "
    "zero, the control system runs in standby for the blocked gates, computing its "
    "standby output without measuring the converter voltage, as in motulator."
)


def monitored_code(
    outputs: dict[str, list[str]],
    monitored: dict[str, str],
    comment: str = "Monitored signals",
    prelude: str = "",
    first: int = 1,
) -> str:
    """
    C code writing the monitored signals to the outputs of the C-Script from the
    output `first` on.

    The code starts with the comment, followed by the prelude (e.g., a coordinate
    transformation used in the expressions of `monitored`) and the assignments.

    """
    return f"/* {comment} */\n{prelude}" + "".join(
        f"OutputSignal({i_out + first}, {j}) = {monitored[name]};\n"
        for i_out, names in enumerate(outputs.values())
        for j, name in enumerate(names)
    )


# %%
# Layout of the schematic (x to the right, y downwards): the control system is on
# the left, the converter and the machine on the right, and the scope at the bottom.
# Positions of the control system (inputs at x - 50, output at x + 54), the
# converter, and the machine (all centered at the same height), and the DC rails
# and the riser of the converter control signal
CS = (250, 100)
CONV = (620, 100)
MACH = (740, 100)
Y_TOP, Y_BOT = 40, 160
X_RISER = 500
# The DC source (or capacitor) and the DC-bus voltmeter, spaced so that their labels on
# the left fit between them, and the shift of the converter and the components after
# it (sch.dx), which makes the same space between the riser and the DC source
X_DC, X_U_DC = CONV[0] - 120, CONV[0] - 60
DC_DX = 60
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


# Tag of the gate enable (the input enable of the control system), and its distance
# above the source
TAG = {"Tag": ENABLE, "Visibility": "2"}  # Schematic
Y_TAG = 15
# Space for the gate enable between the PWM (or the blanking time) and the DC bus
# (horizontal shift of the converter and the components after it)
GATE_DX = 60

# Converter: phases a, b, and c on the right, the gate input on the top, and the DC
# terminals on the top and bottom
CONVERTER_FRAME = "      Frame         [-25, -25; 25, 25]\n"


def _check_delay(mdl: Model) -> None:
    """
    Raise an error if the computational delay is not one sampling period (one half
    carrier period with double update, two with single update).
    """
    if len(mdl.delay.data) != (2 if mdl.single_update else 1):
        raise NotImplementedError(
            "Only the computational delay of one sampling period supported (delay=1, "
            "or delay=2 with single_update)"
        )


def _check_initial_states(
    states: list[tuple[str, complex, complex]],
    exported: list[tuple[str, complex]] | None = None,
) -> None:
    """
    Raise an error if a state of the system model, given as (name, value, initial
    value), is not at its initial value: the exported models start from the initial
    values of motulator (e.g., after a simulation, the states are the final ones),
    except the exported states (name, value), e.g., the speed and the angle of the
    mechanics and the capacitor voltage of the LCL filter. All the states must be
    finite (e.g., not NaN after a failed simulation).
    """
    for name, value in [(n, v) for n, v, _ in states] + (exported or []):
        if not np.isfinite(value):
            raise ValueError(f"The state {name} is not finite: {value}")
    for name, value, initial in states:
        if abs(value - initial) > 1e-9 * max(abs(initial), 1.0):
            raise NotImplementedError(
                f"Only the initial value {initial:.6g} of {name} supported, not "
                f"{value:.6g}"
            )


def _add_pwm(sch: _Schematic, T_s: float | str, single_update: bool = False) -> None:
    """
    Add the carrier comparison (PLECS Symmetrical PWM), fed by the delayed duty ratios.

    The symmetrical triangular carrier with the period 2*T_s (T_s with
    `single_update`), starting from its maximum, and the regular sampling at both the
    maximum and the minimum correspond to CarrierComparison in motulator. With single
    update, the delayed duty ratios change only at the maxima, so that sampling also
    at the minima gives the single update. The counter quantization of motulator is
    not modeled. The output values -1 and 1 are the gate signals of the ideal
    converter.

    """
    k = 1.0 if single_update else 0.5  # Carrier frequency times the sampling period
    params = {
        "sampling": "3",  # Regular (double update)
        "fc": f"{k:g}/{T_s}" if isinstance(T_s, str) else _fmt(k / T_s),
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


def _add_delay(sch: _Schematic, T_s: float | str, block: ControlBlock) -> None:
    """Add the computational delay of one sampling period (PLECS Delay block)."""
    sch.component(
        "Delay", "Delay", (370, CS[1]), {"N": "1", "X0": "0", "Ts": _fmt(T_s)}
    )
    sch.signal((block.name, len(block.inputs) + 1), ("Delay", 1))


# Space for the blanking time between the PWM and the DC bus (horizontal shift of the
# converter and the components after it)
BLANKING_DX = 80


def _add_blanking_time(sch: _Schematic) -> None:
    """
    Add the Blanking Time block of PLECS after the PWM.

    The block delays the turn-on of the switches by the dead time `converter.t_d`. Its
    output values are 1 (upper switch on), -1 (lower switch on), and 0 (both off).

    """
    sch.component(
        "Reference",
        "Blanking time",
        (530, CS[1]),
        {"tb": "converter.t_d"},
        direction="up",
        src_component="Components/Control/Modulators/Blanking Time",
        extra="      Frame         [-20, -20; 20, 20]\n",
        trailer=_terminals([("Output", 24, 0, "right"), ("Input", -20, 0, "left")]),
    )
    sch.signal(("PWM", 1), ("Blanking time", 2))


def _add_converter(
    sch: _Schematic, T_s: float | str, t_d: float = 0.0, u_th: float = 0.0
) -> None:
    """
    Add the two-level converter (PLECS), fed by the gate signals, which are blocked
    while the gate enable is zero.

    The converter is the IGBT converter of PLECS with ideal IGBTs and diodes: the
    upper IGBT is turned on with a positive, the lower one with a negative gate
    signal, and both are off with a zero gate signal, when the current direction
    determines the conducting diode, as in motulator with `sign=np.sign`. With the
    dead time `t_d`, the converter is fed by the blanking time (`_add_blanking_time`,
    added before). With the threshold voltage `u_th`, the IGBT converter with
    parasitics is used, the forward voltages of the IGBTs and the diodes being
    `converter.u_th` (without on-resistances). The gate signals are multiplied by the
    gate enable, the input enable of the control system (from the tag of
    `_add_control_system`) sampled at the sampling instants (`T_s`), as the control
    system evaluates the gate enable in motulator (`ControlSystem.get_enable`), so
    that the gates are blocked (all switches off) without the computational delay.

    """
    params: dict[str, str] = {}
    if u_th > 0:
        src = "Components/Electrical/Converters/Detailed\\n2-Level\\nIGBT\\nConv."
        params = {"Vf_igbt": "converter.u_th", "Ron_igbt": "0"}
        params |= {"Vf_d": "converter.u_th", "Ron_d": "0"}
    else:
        src = "Components/Electrical/Converters/2-Level\\nIGBT\\nConv."
    sch.component(
        "Reference",
        "Converter",
        CONV,
        params,
        # Mirrored so that the AC terminals are on the right, as in the machine
        direction="down",
        flipped=True,
        label="south",
        src_component=src,
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
    # The gate signals rise next to the PWM (or the blanking time) to the product
    # with the sampled gate enable, from which they are routed above the DC bus. The
    # gate enable comes from its tag above them. The coordinates are absolute (the
    # components after the PWM are shifted by sch.dx).
    gate: Terminal = ("Blanking time", 1) if t_d > 0 else ("PWM", 1)
    x_r = X_RISER + (BLANKING_DX if t_d > 0 else 0)  # The riser of the gate signals
    y_p, y_e = 25, 25 - 4 * 5  # The product and the gate enable above it
    dx = sch.dx
    sch.component("From", "From enable", (x_r - 90 - dx, y_e), TAG, show=False)
    ts = T_s if isinstance(T_s, str) else _fmt(T_s)
    sch.component(
        "ZeroOrderHold", "Sampled enable", (x_r - 30 - dx, y_e), {"Ts": ts}, show=False
    )
    sch.signal(("From enable", 1), ("Sampled enable", 1))
    product = {"Inputs": "2"}
    sch.component(
        "Product",
        "Gate enable",
        (x_r + 45 - dx, y_p),
        product,
        direction="up",
        show=False,
    )
    # The gate enable to the upper input, the gate signals to the lower one
    x_e = x_r + 15
    sch.signal(
        ("Sampled enable", 2),
        ("Gate enable", 2),
        [(x_e - dx, y_e), (x_e - dx, y_p - 5)],
    )
    sch.signal(gate, ("Gate enable", 3), [(x_r - dx, CS[1]), (x_r - dx, y_p + 5)])
    x_g = CONV[0] + 15  # The gate input of the converter (shifted by sch.dx)
    sch.signal(("Gate enable", 1), ("Converter", 4), [(x_g, y_p)])


def _add_dc_bus(sch: _Schematic, dc_bus: DCBus | None) -> None:
    """
    Add the DC bus: a stiff voltage (without a DC-bus model), a capacitor (`DCBus`),
    or a diode bridge and LC (`DiodeBridgeDCBus`).

    The circuits are not grounded (as in motulator, there is no zero sequence).

    """
    x_c, y_c = CONV
    sch.component("Voltmeter", "u_dc", (X_U_DC, y_c), direction="up", label="west")
    if isinstance(dc_bus, DiodeBridgeDCBus):
        _add_diode_bridge(sch)
        return
    if dc_bus is not None:
        dc, typ, params = "C_dc", "Capacitor", {"C": "dc_bus.C_dc"}
        params["v_init"] = "dc_bus.u_dc0"
    else:
        dc, typ, params = "U_dc", "DCVoltageSource", {"V": "converter.u_dc"}
    sch.component(typ, dc, (X_DC, y_c), params, direction="up", label="west")
    # Positive and negative rails via the voltmeter to the converter
    for terminal, y in ((1, Y_TOP), (2, Y_BOT)):
        taps: list[Tap] = [([(X_DC, y), (X_U_DC, y)], [(("u_dc", terminal), [])])]
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
            {"V": "dc_bus.u_g", "w": "dc_bus.w_g", "phi": f"pi/2 - {k}*2*pi/3"},
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
        {"L": "dc_bus.L_dc", "i_init": "0"},
        direction="left",
    )
    sch.component(
        "Capacitor",
        "C_dc",
        (X_DC, y_c),
        {"C": "dc_bus.C_dc", "v_init": "dc_bus.u_dc0"},
        direction="up",
        label="west",
    )
    legs = [
        ([(x, Y_TOP)], [((f"D{ph}+", 2), [])])
        for x, ph in zip(x_legs, "abc", strict=True)
    ]
    sch.bus(("L_dc", 1), "Wire", legs[::-1])
    taps = [
        ([(X_DC, Y_TOP)], [(("C_dc", 1), [])]),
        ([(X_U_DC, Y_TOP)], [(("u_dc", 1), [])]),
    ]
    sch.bus(("L_dc", 2), "Wire", taps + [([(x_c, Y_TOP)], [(("Converter", 5), [])])])
    taps = [([(x_c, Y_BOT), (X_U_DC, Y_BOT)], [(("u_dc", 2), [])])]
    taps += [([(X_DC, Y_BOT)], [(("C_dc", 2), [])])]
    taps += [
        ([(x, Y_BOT)], [((f"D{ph}-", 1), [])])
        for x, ph in zip(x_legs[::-1], "cba", strict=True)
    ]
    sch.bus(("Converter", 6), "Wire", taps)


# %%
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


def _add_step(sch: _Schematic, name: str, sig: StepSignal, pos: Point) -> Terminal:
    """
    Add the Step block of a step signal, and with several steps, the Gain block
    summing them 50 to the right of it. Returns the terminal of the signal.
    """
    sch.component("Step", name, pos, _step(sig))
    if np.isscalar(sig.time):
        return (name, 1)
    ones = _fmt([1.0] * len(np.atleast_1d(sig.time)))
    gain = {"K": ones, "Multiplication": "2"}  # Matrix K*u
    sch.component("Gain", f"{name} sum", (pos[0] + 50, pos[1]), gain, show=False)
    sch.signal((name, 1), (f"{name} sum", 1))
    return (f"{name} sum", 2)


def _fmt_mask(value: Any) -> str:
    """Format a mask value, writing multiples of 2*pi as in motulator."""
    if isinstance(value, float) and value > 0 and not isinf(value):
        k = value / (2 * pi)
        if 2 * pi * round(k, 6) == value:  # Exact, as 2*pi*k in MATLAB
            return f"2*pi*{round(k, 6):g}"
    return _fmt(value)


def _mask_parameter(m: MaskParam, value: Any) -> str:
    """Mask parameter definition of a subsystem."""
    return _mask_param(m.variable, m.prompt, _fmt_mask(value), m.tab)


# Prompts of the variables of the model initialization in the mask of the control
# system
WORKSPACE_PROMPTS = {
    "par": "Machine model of the control system (struct)",
    "cfg": "Configuration of the controller (struct)",
    "speed_ctrl": "Arguments of SpeedController (struct)",
    "slew_rate": "Slew rate of the speed reference (mechanical rad/s²)",
    "pwm": "Arguments of the PWM (struct)",
}


def workspace_variables(init: str) -> list[str]:
    """
    Variables of the control system in the model initialization (after the heading
    "%% Control system"), which the mask of the control system passes to the blocks
    inside it, since the components inside a masked subsystem of PLECS see only the
    variables of its mask.
    """
    control = init.split("%% Control system", 1)[1]
    return list(dict.fromkeys(re.findall(r"^(\w+)", control, re.M)))


def _workspace_params(init: str, tab: str) -> str:
    """Mask parameters of the variables of the control system (see
    `workspace_variables`)."""
    return "".join(
        _mask_param(n, f"{n}: {WORKSPACE_PROMPTS[n]}", n, tab)
        for n in workspace_variables(init)
    )


def _control_subsystem(
    block: ControlBlock, values: dict[str, Any], init: str
) -> tuple[str, str]:
    """Mask and contents of the control-system subsystem."""
    n_in = len(block.inputs)
    h = max(30, 5 * n_in)  # Half of the height, the inputs inside the frame
    header = (
        f"      Frame         [-50, -{h}; 50, {h}]\n"
        '      SampleTime    "-1"\n'
        '      CodeGenDiscretizationMethod "2"\n'
        '      CodeGenTarget "Generic"\n'
        f"      MaskType      {_q(block.mask_type)}\n"
        f"      MaskDescription {_q(block.description)}\n"
        + (f"      MaskInit      {_q(block.mask_init)}\n" if block.mask_init else "")
        + "      MaskIconFrame on\n"
        "      MaskIconOpaque off\n"
        "      MaskIconRotates on\n"
        + _workspace_params(init, block.mask_params[0].tab)
        + "".join(_mask_parameter(m, values[m.variable]) for m in block.mask_params)
    )
    # The duty ratios are the only output; the monitored signals are mask probes
    terminals = _terminals(
        [
            *[("Input", -50, 10 * k - 5 * (n_in - 1), "left") for k in range(n_in)],
            ("Output", 54, 0, "right"),
        ]
    )

    inner, size = schematic(block.netlist())
    # The probe signal of the only output of a C-Script block is "Output", and those
    # of several outputs are "Output 1", "Output 2", ...
    probes = [("Duty ratios (d_abc)", "compute_output", "Output")]
    n_out = len(block.outputs)
    monitored = [
        (n, MONITOR, "Output" if n_out == 1 else f"Output {k + 1}")
        for k, n in enumerate(block.outputs)
    ]
    return header, (
        terminals
        + _inner_schematic(inner, size)
        + _mask_probes(probes, "PWM")
        + _mask_probes(monitored)
    )


def _add_control_system(
    sch: _Schematic,
    block: ControlBlock,
    sources: list[tuple[str, StepSignal | float | str]],
    values: dict[str, Any],
    init: str,
) -> int:
    """
    Add the control system and its inputs (references and measurements).

    Each source is a `(name, signal)` pair in the order of the block inputs, the
    signal being a `StepSignal`, a constant, or a probe definition (`_probe`). The
    steps of a `StepSignal` with several steps are summed by a Gain block. The mask
    of the control system passes the variables of the control system in the model
    initialization `init` to the blocks inside it. Returns the y-coordinate of the
    last source.

    """
    n_in = len(sources)
    # The sources are centered at the control system, but not above the top margin,
    # spaced so that their labels below them do not touch the next source
    y_0 = max(CS[1] - 25 * (n_in - 1), 20)
    y_src = [y_0 + 50 * k for k in range(n_in)]
    y_in = [CS[1] + 10 * k - 5 * (n_in - 1) for k in range(n_in)]
    # The vertical segments 10 apart, the last at least 10 left of the frame
    x_jog = _jogs(y_src, y_in, min(140, CS[0] - 50 - 10 * n_in), 10)
    for k, (name, src) in enumerate(sources):
        out: Terminal = (name, 1)
        if isinstance(src, StepSignal):
            out = _add_step(sch, name, src, (60, y_src[k]))
        elif isinstance(src, (int, float)):
            sch.component("Constant", name, (60, y_src[k]), {"Value": _fmt(src)})
        else:
            sch.component("PlecsProbe", name, (60, y_src[k]), extra=src)
        x = x_jog[k]
        points = [] if y_src[k] == y_in[k] else [(x, y_src[k]), (x, y_in[k])]
        sch.signal(out, (block.name, k + 1), points)
        if name == ENABLE:
            # The gate enable also to the converter (`_add_converter`), by a tag above
            x_b, y_tag = x_jog[k] - 5, y_src[k] - Y_TAG
            sch.component("Goto", "Goto enable", (x_b + 40, y_tag), TAG, show=False)
            sch.trunk(out, "Signal", [(x_b, y_src[k])])
            sch.signal(out, ("Goto enable", 1), [(x_b, y_tag)])
    header, trailer = _control_subsystem(block, values, init)
    sch.component(
        "Subsystem", block.name, CS, direction="up", extra=header, trailer=trailer
    )
    return y_src[-1]


def _add_ctrl_output(sch: _Schematic, block: ControlBlock) -> None:
    """Add the output port of the controller signals (mask probes of the block)."""
    probe = _probe(block.name, list(block.outputs))
    sch.component("PlecsProbe", "Controller signals", (400, 340), extra=probe)
    sch.component("Output", "ctrl", (480, 340), {"Index": "2", "Width": "-1"})
    sch.signal(("Controller signals", 1), ("ctrl", 1))


# %%
# Output ports of the model for scripted simulations
OUTPUT_TERMINALS = "".join(
    f'  Terminal {{\n    Type          Output\n    Index         "{k}"\n  }}\n'
    for k in (1, 2)
)


def _write_model(
    path: Path,
    init: str,
    t_stop: float,
    T_s: float | str,
    sch: _Schematic,
    size: Point,
    outputs: bool,
) -> Path:
    """
    Write the model file: the settings, the model initialization `init` (the
    parameters), and the schematic.
    """
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
    path.write_text(text.replace(C_DIR, c_dir), encoding="utf-8")
    return path
