"""
Export motulator grid converter systems to PLECS.

This module converts a grid converter system with grid-following current-vector
control or disturbance-observer-based grid-forming control into a PLECS Standalone
model, in the same way as `motulator_plecs.sm` does for drive systems. The control
system is a masked subsystem, whose parameters are those of
`CurrentVectorControllerCfg` or `ObserverBasedGridFormingControllerCfg`, and
contains a C-Script block with the C port in `c/gfl_current_vector.c` or
`c/gfm_observer.c`. The system model consists of PLECS blocks.

Currently supported:

- Grid-following control (`CurrentVectorController`) in the power-control mode, with
  the active and reactive power references given as steps
- Grid-forming control (`ObserverBasedGridFormingController`) in the power-control
  mode, with the active power reference given as a step and a constant converter
  voltage reference
- Converter: carrier comparison (`pwm=True`) with the Symmetrical PWM block and the
  ideal two-level converter of PLECS, a stiff DC bus (`VoltageSourceConverter`), and
  the computational delay of one sampling period as a PLECS Delay block
- AC filter: LCL filter (`LCLFilter`) without resistances and grid impedance, or L
  filter (`LFilter`) with its series resistance and the grid inductance
- Grid: balanced three-phase voltage source (`ThreePhaseSource`) with constant
  magnitude and frequency

"""

from pathlib import Path
from typing import Any, cast

import numpy as np
from motulator.common.model._converter import VoltageSourceConverter
from motulator.common.model._pwm import CarrierComparison
from motulator.common.utils import complex2abc
from motulator.grid.control import (
    CurrentVectorController,
    GridConverterControlSystem,
    ObserverBasedGridFormingController,
)
from motulator.grid.model import (
    GridConverterSystem,
    LCLFilter,
    LFilter,
    ThreePhaseSource,
)

from motulator_plecs._common import (
    C_DIR,
    C_PARAMS,
    CONV,
    ControlBlock,
    MaskParam,
    StepSignal,
    Tap,
    _add_control_system,
    _add_converter,
    _add_ctrl_output,
    _add_dc_bus,
    _add_delay,
    _add_pwm,
    _probe,
    _Schematic,
    _scope,
    _write_model,
    cfg_assignments,
    parameter_checks,
    simulate_plecs,
)

# %%
# Grid-following control: inputs, monitored signals (mask probes), and mask parameters
GFL_INPUTS = ["p_g_ref", "q_g_ref", "i_c_abc", "u_g_line", "u_dc"]
GFL_OUTPUTS = {
    "Power (p_g_ref, p_g, q_g_ref, q_g)": ["p_g_ref", "p_g", "q_g_ref", "q_g"],
    "Current (i_c_d_ref, i_c_d, i_c_q_ref, i_c_q)": [
        "i_c_d_ref",
        "i_c_d",
        "i_c_q_ref",
        "i_c_q",
    ],
    "PLL (u_g, w_g, theta_c)": ["u_g", "w_g", "theta_c"],
}
TAB_GFL = "Grid-following control (CurrentVectorControllerCfg)"
GFL_MASK_PARAMS = [
    MaskParam("i_max", "Maximum current (A), peak value: i_max", TAB_GFL, "", True),
    MaskParam("L", "Filter inductance (H): L", TAB_GFL, "", True),
    MaskParam(
        "alpha_c", "Current-control bandwidth (rad/s): alpha_c", TAB_GFL, "cfg.alpha_c"
    ),
    MaskParam(
        "alpha_i",
        "Integral-action bandwidth (rad/s), [] = alpha_c: alpha_i",
        TAB_GFL,
        "",
    ),
    MaskParam(
        "u_nom", "Nominal grid voltage (V), peak phase: u_nom", TAB_GFL, "cfg.u_nom"
    ),
    MaskParam(
        "w_nom", "Nominal grid angular frequency (rad/s): w_nom", TAB_GFL, "cfg.w_nom"
    ),
    MaskParam(
        "alpha_pll", "PLL bandwidth (rad/s): alpha_pll", TAB_GFL, "cfg.alpha_pll"
    ),
    MaskParam("T_s", "Sampling period (s): T_s", TAB_GFL, "cfg.T_s", True),
]

# Grid-forming control
GFM_INPUTS = ["p_g_ref", "v_c_ref", "i_c_abc", "u_dc"]
GFM_OUTPUTS = {
    "Power (p_g_ref, p_g, q_g)": ["p_g_ref", "p_g", "q_g"],
    "Voltage (v_c_ref, v_c)": ["v_c_ref", "v_c"],
    "Current (i_c_d_ref, i_c_d, i_c_q_ref, i_c_q)": [
        "i_c_d_ref",
        "i_c_d",
        "i_c_q_ref",
        "i_c_q",
    ],
    "Angle (theta_c)": ["theta_c"],
}
TAB_GFM = "Grid-forming control (ObserverBasedGridFormingControllerCfg)"
GFM_MASK_PARAMS = [
    MaskParam("i_max", "Maximum current (A), peak value: i_max", TAB_GFM, "", True),
    MaskParam("L", "Total inductance estimate (H): L", TAB_GFM, "", True),
    MaskParam("R", "Total series resistance estimate (Ω): R", TAB_GFM, "cfg.R"),
    MaskParam(
        "R_a", "Active resistance (Ω), [] = 0.25*u_nom/i_max: R_a", TAB_GFM, "cfg.R_a"
    ),
    MaskParam(
        "k_v", "Voltage control gain, [] = alpha_o/w_nom: k_v", TAB_GFM, "cfg.k_v"
    ),
    MaskParam("alpha_o", "Observer gain (rad/s): alpha_o", TAB_GFM, "cfg.alpha_o"),
    MaskParam(
        "alpha_c", "Current-control bandwidth (rad/s): alpha_c", TAB_GFM, "cfg.alpha_c"
    ),
    MaskParam(
        "u_nom", "Nominal grid voltage (V), peak phase: u_nom", TAB_GFM, "cfg.u_nom"
    ),
    MaskParam(
        "w_nom", "Nominal grid angular frequency (rad/s): w_nom", TAB_GFM, "cfg.w_nom"
    ),
    MaskParam("T_s", "Sampling period (s): T_s", TAB_GFM, "cfg.T_s", True),
    MaskParam(
        "i_d_max",
        "Maximum active current (A), [] = no power limitation: i_d_max",
        TAB_GFM,
        "cfg.i_d_max",
    ),
    MaskParam(
        "alpha_l", "Power-limitation bandwidth (rad/s): alpha_l", TAB_GFM, "cfg.alpha_l"
    ),
]


# %%
def _check_supported(
    mdl: GridConverterSystem, ctrl: GridConverterControlSystem
) -> None:
    """Raise an error if the grid converter system is not supported."""
    if type(mdl.converter) is not VoltageSourceConverter:
        raise NotImplementedError("Only VoltageSourceConverter supported")
    f = mdl.ac_filter
    if isinstance(f, LCLFilter):
        if f.R_fc or f.R_fg or f.L_g or f.R_g:
            raise NotImplementedError("Only LCLFilter without resistances and L_g")
    elif not isinstance(f, LFilter) or f.R_g:
        raise NotImplementedError("Only LFilter and LCLFilter without R_g supported")
    src = mdl.ac_source
    if type(src) is not ThreePhaseSource or callable(src.w_g) or callable(src.e_g):
        raise NotImplementedError("Only ThreePhaseSource with constant w_g and e_g")
    if src.phi != 0 or src.e_g_neg != 0:
        raise NotImplementedError("Phase shift and negative sequence not supported")
    if not isinstance(mdl.pwm, CarrierComparison):
        raise NotImplementedError("Only CarrierComparison (pwm=True) supported")
    if len(mdl.delay.data) != 1:
        raise NotImplementedError("Only the computational delay of one sample")
    inner = ctrl.inner_ctrl
    if not isinstance(
        inner, (CurrentVectorController, ObserverBasedGridFormingController)
    ):
        raise NotImplementedError("Only CurrentVectorController and DO-GFM supported")
    if ctrl.dc_bus_voltage_ctrl is not None:
        raise NotImplementedError("DC-bus voltage control not supported")


def _plant_variables(mdl: GridConverterSystem) -> list[tuple[str, Any]]:
    """Workspace variables of the system model."""
    f = mdl.ac_filter
    src = cast(ThreePhaseSource, mdl.ac_source)
    variables: list[tuple[str, Any]] = [("converter.u_dc", mdl.converter.u_dc)]
    if isinstance(f, LCLFilter):
        variables += [
            ("ac_filter.L_fc", f.L_fc),
            ("ac_filter.L_fg", f.L_fg),
            ("ac_filter.C_f", f.C_f),
            ("ac_filter.u_f0_abc", complex2abc(f.state.u_f_ab)),
        ]
    else:
        f = cast(LFilter, f)
        variables += [
            ("ac_filter.L_f", f.L_f),
            ("ac_filter.R_f", f.R_f),
            ("ac_filter.L_g", f.L_g),
        ]
    return variables + [("ac_source.e_g", src.e_g), ("ac_source.w_g", src.w_g)]


# %%
DUTY_RATIO_CODE = (
    "/* Duty ratios, delayed by the Delay block outside the subsystem */\n"
    "for (int k = 0; k < 3; k++) {\n"
    "    OutputSignal(0, k) = ctrl.ref.d_abc[k];\n"
    "}\n"
    "\n"
    "/* Monitored signals */\n"
)


def _monitored_code(outputs: dict[str, list[str]], monitored: dict[str, str]) -> str:
    """C code writing the monitored signals to the other outputs of the C-Script."""
    return "".join(
        f"OutputSignal({i_out + 1}, {j}) = {monitored[name]};\n"
        for i_out, names in enumerate(outputs.values())
        for j, name in enumerate(names)
    )


def _gfl_cscript_code() -> dict[str, str]:
    """Code sections of the C-Script block of grid-following control."""
    declarations = (
        "/* Generated by motulator_plecs.grid. The control algorithms are in the\n"
        " * included C files. The parameters come from the mask of the subsystem. */\n"
        f'#include "{C_DIR}/common.c"\n'
        f'#include "{C_DIR}/gfl_current_vector.c"\n'
        "\n" + C_PARAMS + "\n"
        "static GFLControlSystem ctrl;\n"
    )
    i = {m.variable: k for k, m in enumerate(GFL_MASK_PARAMS)}
    start = (
        "/* Check the parameters of the mask */\n"
        + parameter_checks(GFL_MASK_PARAMS)
        + "\n"
        "/* Configuration (CurrentVectorControllerCfg), the defaults are used for\n"
        " * empty parameters */\n"
        f"GFLControllerCfg cfg = gfl_controller_cfg(P({i['i_max']}, 0), "
        f"P({i['L']}, 0));\n"
        + cfg_assignments(GFL_MASK_PARAMS)
        + f"cfg.alpha_i = PDIM({i['alpha_i']}) > 0 ? P({i['alpha_i']}, 0) : "
        "cfg.alpha_c;\n"
        "gfl_control_system_init(&ctrl, &cfg);\n"
    )
    output = (
        "/* Measurements and references */\n"
        "double i_c_abc[3] = {InputSignal(2, 0), InputSignal(2, 1),\n"
        "                     InputSignal(2, 2)};\n"
        "/* Line-to-line PCC voltages u_ab and u_bc */\n"
        "double u_g_line[2] = {InputSignal(3, 0), InputSignal(3, 1)};\n"
        "GridMeasurements meas = {abc2complex(i_c_abc), line2complex(u_g_line),\n"
        "                         InputSignal(4, 0)};\n"
        "gfl_control_system_compute_output(&ctrl, &meas, InputSignal(0, 0),\n"
        "                                  InputSignal(1, 0));\n"
        "\n" + DUTY_RATIO_CODE
    )
    monitored = {
        "p_g_ref": "ctrl.ref.p_g",
        "p_g": "ctrl.fbk.p_g",
        "q_g_ref": "ctrl.ref.q_g",
        "q_g": "ctrl.fbk.q_g",
        "i_c_d_ref": "creal(ctrl.ref.i_c)",
        "i_c_d": "creal(ctrl.fbk.i_c)",
        "i_c_q_ref": "cimag(ctrl.ref.i_c)",
        "i_c_q": "cimag(ctrl.fbk.i_c)",
        "u_g": "ctrl.fbk.u_g",
        "w_g": "ctrl.fbk.w_g",
        "theta_c": "ctrl.fbk.theta_c",
    }
    output += _monitored_code(GFL_OUTPUTS, monitored)
    return {
        "Declarations": declarations,
        "StartFcn": start,
        "OutputFcn": output,
        "UpdateFcn": "gfl_control_system_update(&ctrl);\n",
    }


def _gfm_cscript_code() -> dict[str, str]:
    """Code sections of the C-Script block of grid-forming control."""
    declarations = (
        "/* Generated by motulator_plecs.grid. The control algorithms are in the\n"
        " * included C files. The parameters come from the mask of the subsystem. */\n"
        f'#include "{C_DIR}/common.c"\n'
        f'#include "{C_DIR}/gfm_observer.c"\n'
        "\n" + C_PARAMS + "\n"
        "static GFMControlSystem ctrl;\n"
    )
    i = {m.variable: k for k, m in enumerate(GFM_MASK_PARAMS)}
    start = (
        "/* Check the parameters of the mask */\n"
        + parameter_checks(GFM_MASK_PARAMS)
        + "\n"
        "/* Configuration (ObserverBasedGridFormingControllerCfg), the defaults are\n"
        " * used for empty parameters */\n"
        f"GFMControllerCfg cfg = gfm_controller_cfg(P({i['i_max']}, 0), "
        f"P({i['L']}, 0));\n"
        + cfg_assignments(GFM_MASK_PARAMS)
        + "gfm_control_system_init(&ctrl, &cfg);\n"
    )
    output = (
        "/* Measurements and references */\n"
        "double i_c_abc[3] = {InputSignal(2, 0), InputSignal(2, 1),\n"
        "                     InputSignal(2, 2)};\n"
        "GFMMeasurements meas = {abc2complex(i_c_abc), InputSignal(3, 0)};\n"
        "gfm_control_system_compute_output(&ctrl, &meas, InputSignal(0, 0),\n"
        "                                  InputSignal(1, 0));\n"
        "\n" + DUTY_RATIO_CODE
    )
    monitored = {
        "p_g_ref": "ctrl.ref.p_g",
        "p_g": "ctrl.fbk.p_g",
        "q_g": "ctrl.fbk.q_g",
        "v_c_ref": "ctrl.ref.v_c",
        "v_c": "cabs(ctrl.fbk.v_c)",
        "i_c_d_ref": "creal(ctrl.ref.i_c)",
        "i_c_d": "creal(ctrl.fbk.i_c)",
        "i_c_q_ref": "cimag(ctrl.ref.i_c)",
        "i_c_q": "cimag(ctrl.fbk.i_c)",
        "theta_c": "ctrl.fbk.theta_c",
    }
    output += _monitored_code(GFM_OUTPUTS, monitored)
    return {
        "Declarations": declarations,
        "StartFcn": start,
        "OutputFcn": output,
        "UpdateFcn": "gfm_control_system_update(&ctrl);\n",
    }


GFL_BLOCK = ControlBlock(
    name="Grid-following control",
    mask_type="Grid-following control (motulator)",
    description=(
        "Current-vector grid-following control with a PLL in the power-control mode. "
        "The parameters correspond to the motulator API: CurrentVectorControllerCfg. "
        "Empty parameters ([]) correspond to the defaults of motulator."
    ),
    mask_params=GFL_MASK_PARAMS,
    inputs=GFL_INPUTS,
    input_widths=[1, 1, 3, 2, 1],
    outputs=GFL_OUTPUTS,
    code=_gfl_cscript_code,
)

GFM_BLOCK = ControlBlock(
    name="Grid-forming control",
    mask_type="Grid-forming control (motulator)",
    description=(
        "Disturbance-observer-based grid-forming control in the power-control mode, "
        "with transparent current limitation. The parameters correspond to the "
        "motulator API: ObserverBasedGridFormingControllerCfg. Empty parameters ([]) "
        "correspond to None, i.e., the defaults of motulator."
    ),
    mask_params=GFM_MASK_PARAMS,
    inputs=GFM_INPUTS,
    input_widths=[1, 1, 3, 1],
    outputs=GFM_OUTPUTS,
    code=_gfm_cscript_code,
)


def _control_block(
    ctrl: GridConverterControlSystem,
) -> tuple[ControlBlock, dict[str, Any]]:
    """Get the control-system block and its mask values in the motulator API."""
    inner = ctrl.inner_ctrl
    if isinstance(inner, CurrentVectorController):
        cfg = inner.cfg
        values = {m.variable: getattr(cfg, m.variable) for m in GFL_MASK_PARAMS}
        return GFL_BLOCK, values
    gfm = cast(ObserverBasedGridFormingController, inner)
    obs = gfm.observer  # The configuration is stored only as the resulting gains
    u_nom, w_nom, i_max = obs.state.u_gp, obs.w_g, gfm.current_limiter.i_max
    # R_a and k_v are resolved in ObserverBasedGridFormingControllerCfg.__post_init__
    values = {
        "i_max": i_max,
        "L": obs.L,
        "R": obs.R,
        "R_a": None if gfm.R_a == 0.25 * u_nom / i_max else gfm.R_a,
        "k_v": None if gfm.k_v == obs.alpha_o / w_nom else gfm.k_v,
        "alpha_o": obs.alpha_o,
        "alpha_c": gfm.k_c / obs.L,
        "u_nom": u_nom,
        "w_nom": w_nom,
        "T_s": gfm.T_s,
        "i_d_max": gfm.i_d_max,
        "alpha_l": gfm.alpha_l,
    }
    return GFM_BLOCK, values


# %%
def _abc_probe(comp: str, signal: str) -> str:
    """Probe of the three-phase measurements comp + "a", comp + "b", comp + "c"."""
    return "".join(_probe(f"{comp}{ph}", [signal]) for ph in "abc")


def _grid_source(k: int) -> dict[str, str]:
    """Grid voltage e_g*cos(w_g*t - k*2*pi/3) of the phase k, exp_j_theta_g(0) = 1."""
    return {"V": "ac_source.e_g", "w": "ac_source.w_g", "phi": f"pi/2 - {k}*2*pi/3"}


def _add_converter_current_meters(sch: _Schematic, y_ph: list[int]) -> None:
    """Add the ammeters of the converter currents (measured for the control system)."""
    for k, ph in enumerate("abc"):
        sch.component(
            "Ammeter", f"i_c{ph}", (690, y_ph[k]), direction="left", show=False
        )
        jog = [] if k == 1 else [(660, CONV[1] - 10 + 10 * k), (660, y_ph[k])]
        sch.wire(("Converter", k + 1), (f"i_c{ph}", 1), jog)


def _add_lcl_filter_and_grid(sch: _Schematic) -> None:
    """Add the LCL filter, the grid, and the PCC voltage measurements."""
    y_ph = [60, 100, 140]  # Phase lines a, b, and c
    x_cap = [780, 810, 840]  # Filter capacitors
    x_v = 940  # Line-to-line voltmeters between the phase lines
    _add_converter_current_meters(sch, y_ph)
    for k, ph in enumerate("abc"):
        y = y_ph[k]
        # Converter-side inductor, capacitor, and grid-side inductor
        inductor = {"i_init": "0", "L": "ac_filter.L_fc"}
        sch.component(
            "Inductor",
            f"L_fc{ph}",
            (740, y),
            inductor,
            direction="left",
            label="north",
            show=k == 0,
        )
        sch.wire((f"i_c{ph}", 2), (f"L_fc{ph}", 1))
        capacitor = {"C": "ac_filter.C_f", "v_init": f"ac_filter.u_f0_abc({k + 1})"}
        sch.component(
            "Capacitor",
            f"C_f{ph}",
            (x_cap[k], 175),
            capacitor,
            direction="up",
            label="east",
            show=k == 2,  # The label of the last capacitor does not overlap
        )
        inductor = {"i_init": "0", "L": "ac_filter.L_fg"}
        sch.component(
            "Inductor",
            f"L_fg{ph}",
            (890, y),
            inductor,
            direction="left",
            label="north",
            show=k == 0,
        )
        taps: list[Tap] = [([(x_cap[k], y)], [((f"C_f{ph}", 1), [])])]
        sch.bus((f"L_fc{ph}", 2), "Wire", taps + [([], [((f"L_fg{ph}", 1), [])])])
        sch.component(
            "ACVoltageSource",
            f"e_g{ph}",
            (990, y),
            _grid_source(k),
            direction="left",
            label="north",
            show=k == 0,
        )
    # Star point of the capacitors
    taps = [([(x_cap[0], 200), (x_cap[1], 200)], [(("C_fb", 2), [])])]
    sch.bus(("C_fa", 2), "Wire", taps + [([(x_cap[2], 200)], [(("C_fc", 2), [])])])
    # Line-to-line PCC voltages u_ab and u_bc (measured for the control system), with
    # the voltmeters between the phase lines
    sch.component("Voltmeter", "u_gab", (x_v, 80), direction="up", show=False)
    sch.component("Voltmeter", "u_gbc", (x_v, 120), direction="up", show=False)
    meters = [[("u_gab", 1)], [("u_gab", 2), ("u_gbc", 1)], [("u_gbc", 2)]]
    for k, ph in enumerate("abc"):
        dsts = [(m, []) for m in meters[k]]
        taps = [([(x_v, y_ph[k])], dsts), ([], [((f"e_g{ph}", 1), [])])]
        sch.bus((f"L_fg{ph}", 2), "Wire", taps)
    # Star point of the grid (not grounded, the circuit is floating as in motulator)
    taps = [([(1030, y_ph[0]), (1030, y_ph[1])], [(("e_gb", 2), [])])]
    sch.bus(("e_ga", 2), "Wire", taps + [([(1030, y_ph[2])], [(("e_gc", 2), [])])])


def _add_l_filter_and_grid(sch: _Schematic) -> None:
    """Add the L filter with its resistance, the grid inductance, and the grid."""
    y_ph = [60, 100, 140]  # Phase lines a, b, and c
    _add_converter_current_meters(sch, y_ph)
    for k, ph in enumerate("abc"):
        elements = [
            ("Inductor", "L_f", {"i_init": "0", "L": "ac_filter.L_f"}),
            ("Resistor", "R_f", {"R": "ac_filter.R_f"}),
            ("Inductor", "L_g", {"i_init": "0", "L": "ac_filter.L_g"}),
            ("ACVoltageSource", "e_g", _grid_source(k)),
        ]
        prev = f"i_c{ph}"
        for n, (typ, name, params) in enumerate(elements):
            sch.component(
                typ,
                f"{name}{ph}",
                (740 + 50 * n, y_ph[k]),
                params,
                direction="left",
                label="north",
                show=k == 0,
            )
            sch.wire((prev, 2), (f"{name}{ph}", 1))
            prev = f"{name}{ph}"
    # Star point of the grid (not grounded, the circuit is floating as in motulator)
    taps: list[Tap] = [([(930, y_ph[0]), (930, y_ph[1])], [(("e_gb", 2), [])])]
    sch.bus(("e_ga", 2), "Wire", taps + [([(930, y_ph[2])], [(("e_gc", 2), [])])])


def _mdl_outputs(mdl: GridConverterSystem) -> list[str]:
    """Signals of the output port "mdl": the converter (and grid) currents."""
    names = ["i_c_a", "i_c_b", "i_c_c"]
    if isinstance(mdl.ac_filter, LCLFilter):
        names += ["i_g_a", "i_g_b", "i_g_c"]
    return names


def _add_outputs(
    sch: _Schematic, block: ControlBlock, lcl: bool, outputs: bool
) -> None:
    """Add the scope and its probes, and optionally the output ports."""
    inductor = "L_fc" if lcl else "L_f"
    if (
        outputs
    ):  # Output ports for scripted simulations: currents and controller signals
        probes = _abc_probe(inductor, "Inductor current")
        if lcl:
            probes += _abc_probe("L_fg", "Inductor current")
        sch.component("PlecsProbe", "Currents", (400, 300), extra=probes)
        sch.component("Output", "mdl", (480, 300), {"Index": "1", "Width": "-1"})
        sch.signal(("Currents", 1), ("mdl", 1))
        _add_ctrl_output(sch, block)
    # Scope: powers, converter currents, and the PCC voltages (LCL filter) or the
    # second monitored signal group of the control system
    groups = list(block.outputs)
    if lcl:
        voltages = _probe("u_gab", ["Measured voltage"])
        voltages += _probe("u_gbc", ["Measured voltage"])
        voltage_axis = ("Voltage", "Line-to-line voltage (V)")
    else:
        voltages = _probe(block.name, groups[1:2])
        voltage_axis = ("Voltage", "Voltage (V)")
    probes_scope = [
        ("Powers", _probe(block.name, groups[:1]), 300),
        ("Converter currents", _abc_probe(inductor, "Inductor current"), 340),
        ("Voltages", voltages, 380),
    ]
    for name, extra, y in probes_scope:
        sch.component("PlecsProbe", name, (720, y), extra=extra)
    axes = [("Power", "Power (W, VAr)"), ("Current", "Current (A)"), voltage_axis]
    sch.component("Scope", "Scope", (900, 320), extra=_scope(axes), direction="up")
    sch.signal(("Powers", 1), ("Scope", 1), [(760, 300), (760, 310)])
    sch.signal(("Converter currents", 1), ("Scope", 2), [(770, 340), (770, 320)])
    sch.signal(("Voltages", 1), ("Scope", 3), [(780, 380), (780, 330)])


# %%
def write_model(
    path: str | Path,
    mdl: GridConverterSystem,
    ctrl: GridConverterControlSystem,
    t_stop: float,
    p_g_ref: StepSignal,
    q_g_ref: StepSignal | None = None,
    v_c_ref: float | None = None,
    outputs: bool = False,
) -> Path:
    """
    Write a PLECS model of the grid converter system.

    Parameters
    ----------
    path : str | Path
        Path of the model file (.plecs). The C sources are expected in the `src`
        directory of the package, referred to by a path relative to the model file.
    mdl : GridConverterSystem
        Continuous-time system model.
    ctrl : GridConverterControlSystem
        Discrete-time control system.
    t_stop : float
        Simulation stop time (s).
    p_g_ref : StepSignal
        Active power reference (W).
    q_g_ref : StepSignal, optional
        Reactive power reference (VAr), needed in grid-following control.
    v_c_ref : float, optional
        Converter voltage magnitude reference (V), needed in grid-forming control.
    outputs : bool, optional
        Add the output ports "mdl" and "ctrl" for `simulate`, defaults to
        False.

    Returns
    -------
    Path
        Path of the written model file.

    """
    path = Path(path)
    _check_supported(mdl, ctrl)
    block, values = _control_block(ctrl)
    lcl = isinstance(mdl.ac_filter, LCLFilter)
    sch = _Schematic()
    sources: list[tuple[str, StepSignal | float | str]] = [("p_g_ref", p_g_ref)]
    if block is GFL_BLOCK:
        if q_g_ref is None:
            raise ValueError("q_g_ref is needed in grid-following control")
        sources += [("q_g_ref", q_g_ref)]
    else:
        if v_c_ref is None:
            raise ValueError("v_c_ref is needed in grid-forming control")
        sources += [("v_c_ref", v_c_ref)]
    sources += [("i_c_abc", _abc_probe("i_c", "Measured current"))]
    if block is GFL_BLOCK:
        voltages = _probe("u_gab", ["Measured voltage"])
        voltages += _probe("u_gbc", ["Measured voltage"])
        sources += [("u_g_line", voltages)]
    sources += [("u_dc meas.", _probe("u_dc", ["Measured voltage"]))]
    _add_control_system(sch, block, sources, values)
    _add_delay(sch, values["T_s"], block)
    _add_pwm(sch, values["T_s"])
    _add_converter(sch)
    _add_dc_bus(sch, cast(Any, mdl))
    if lcl:
        _add_lcl_filter_and_grid(sch)
    else:
        _add_l_filter_and_grid(sch)
    _add_outputs(sch, block, lcl, outputs)
    variables = _plant_variables(mdl)
    size = (1200, 440)
    return _write_model(
        path, block, variables, t_stop, values["T_s"], sch, size, outputs
    )


def simulate(
    path: str | Path,
    t_eval: np.ndarray,
    mdl: GridConverterSystem,
    ctrl: GridConverterControlSystem,
) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
    """Simulate the PLECS model of a grid converter system, see `sm.simulate`."""
    block, _ = _control_block(ctrl)
    return simulate_plecs(
        path, t_eval, mdl_outputs=_mdl_outputs(mdl), ctrl_outputs=block.signals
    )
