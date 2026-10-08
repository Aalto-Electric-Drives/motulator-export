"""
Control of grid converters: each class is a block.

The control systems mirror the classes of motulator as a hierarchy of blocks, each
with a mask of the arguments of its class (see `_control`):

- GridConverterControlSystem (the masked subsystem of the control system),
  containing the blocks of its methods (`get_measurement` and `modulate`), the
  monitored signals, the `PWM`, and the inner controller
- Grid-following control: CurrentVectorController(cfg), containing the `PLL`, the
  `CurrentLimiter`, the `CurrentController`, and the block of its `compute_output`
  computing the current reference from the power references
- Grid-forming control: ObserverBasedGridFormingController(cfg), containing the
  `Observer`, the `CurrentLimiter`, and the blocks of its `compute_output` before
  and after the current limiter: the current reference (with the state of the
  active-power limitation, updated in `update`) and the voltage reference

The mask initializations of the inner controllers resolve the defaults of their
configurations. The masks of the top-level classes refer to the variables of the
model initialization (`init_script`): the configuration `cfg` and the PWM `pwm`.
The signal vectors are defined in `gfl_current_vector.h` and `gfm_observer.h`.

"""

from typing import Any

from motulator_export.plecs._common import ENABLE
from motulator_export.plecs._control import (
    SAMPLING,
    Signals,
    abc_input,
    assign,
    checks,
    complex_in,
    complex_out,
    control_block,
    current_limiter,
    get_measurement,
    grid_converter_control_system,
    init_header,
    mask_params,
    monitor,
    pwm_init,
)
from motulator_export.plecs._netlist import (
    CBlock,
    Mask,
    Mux,
    Param,
    Port,
    Subsystem,
    Tag,
)

GFL_SIGNALS = Signals(
    ("common.c", "gfl_current_vector.c"),
    ("gfl_current_vector.h",),
    {
        "meas": ("GridMeasurements", "grid_measurements"),
        "fbk": ("GFLFeedbacks", "gfl_feedbacks"),
        "ref": ("GFLReferences", "gfl_references"),
    },
    "sfun_gfl",
)
GFM_SIGNALS = Signals(
    ("common.c", "gfm_observer.c"),
    ("gfm_observer.h",),
    {
        "meas": ("GFMMeasurements", "gfm_measurements"),
        "fbk": ("GFMFeedbacks", "gfm_feedbacks"),
        "ref": ("GFMReferences", "gfm_references"),
    },
    "sfun_gfm",
)

# Names of the control-system blocks, their inputs, and the monitored signals (mask
# probes)
GFL_NAME = "Grid-following control"
GFL_INPUTS = [
    Port(ENABLE, 1),
    Port("p_g_ref", 1),
    Port("q_g_ref", 1),
    Port("i_c_abc", 3),
    Port("u_g_line", 2),
    Port("u_dc", 1),
]
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
GFM_NAME = "Grid-forming control"
GFM_INPUTS = [
    Port(ENABLE, 1),
    Port("p_g_ref", 1),
    Port("v_c_ref", 1),
    Port("i_c_abc", 3),
    Port("u_dc", 1),
]
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


# %%
# Grid-following control
def _pll() -> CBlock:
    """Phase-locked loop (PLL)."""
    mask = Mask(
        "PLL (motulator)",
        "Phase-locked loop with the filtered PCC voltage magnitude (PLL), giving the "
        "feedback signals fbk in its coordinates from the measurements and the "
        "realized converter voltage.",
        [
            Param(
                "u_nom",
                "u_nom: Nominal grid voltage (V), line-to-neutral peak value",
                "cfg.u_nom",
            ),
            Param(
                "w_nom", "w_nom: Nominal grid angular frequency (rad/s)", "cfg.w_nom"
            ),
            Param("alpha_pll", "alpha_pll: PLL bandwidth (rad/s)", "cfg.alpha_pll"),
            Param("T_s", "T_s: Sampling period (s)", "cfg.T_s"),
        ],
    )
    params = mask_params(mask)
    start = checks(params, params) + "pll_init(&self, P(0, 0), P(1, 0), P(2, 0));\n"
    start += "T_s = P(3, 0);\n"
    output = (
        GFL_SIGNALS.read(1, "meas")
        + f"pll_compute_output(&self, {complex_in(2)}, meas.i_c_ab,\n"
        "                   meas.u_g_ab, &out);\n" + GFL_SIGNALS.write(0, "fbk", "out")
    )
    fbk = GFL_SIGNALS.width("fbk")
    return CBlock(
        "PLL",
        [Port(ENABLE, 1), GFL_SIGNALS.port("meas"), Port("u_c_ab", 2)],
        [Port("out", fbk)],
        GFL_SIGNALS.class_code(
            "PLL",
            start,
            output,
            "pll_update(&self, T_s, &out);\n",
            SAMPLING + "static GFLFeedbacks out;\n",
            (fbk,),
        ),
        params,
        GFL_SIGNALS.sfunction("pll"),
        mask=mask,
    )


def _gfl_current_reference() -> CBlock:
    """
    References ref.p_g and ref.q_g, and the current reference from the power
    references before the current limiter (compute_output).
    """
    output = (
        "OutputSignal(0, 0) = InputSignal(0, 0);\n"
        "OutputSignal(1, 0) = InputSignal(1, 0);\n"
        "/* Current reference from the power references and the PCC voltage */\n"
        "double complex i_c_ref =\n"
        "    gfl_current_reference(InputSignal(0, 0), InputSignal(1, 0), "
        "InputSignal(2, 0));\n" + complex_out(2, "i_c_ref")
    )
    return CBlock(
        "compute_output",
        [Port("p_g_ref", 1), Port("q_g_ref", 1), Port("u_g", 1)],
        [Port("p_g", 1), Port("q_g", 1), Port("i_c", 2)],
        GFL_SIGNALS.function_code(output),
        ["cfg.T_s"],
        GFL_SIGNALS.sfunction("current_reference"),
        sample_time="cfg.T_s",
    )


def _gfl_current_controller() -> CBlock:
    """Current controller (CurrentController)."""
    mask = Mask(
        "CurrentController (motulator)",
        "2DOF PI current controller (CurrentController) with the PCC voltage fbk.u_g "
        "as the feedforward. The inputs are the current reference and the feedback "
        "signals fbk (the current fbk.i_c and the PCC voltage fbk.u_g, and the "
        "realized voltage fbk.u_c and the angular speed fbk.w_c of the coordinates for "
        "the integral state), and the output is the voltage reference.",
        [
            Param("L", "L: Filter inductance (H)", "cfg.L"),
            Param(
                "alpha_c",
                "alpha_c: Reference-tracking bandwidth (rad/s)",
                "cfg.alpha_c",
            ),
            Param(
                "alpha_i",
                "alpha_i: Integral-action bandwidth (rad/s), [] = alpha_c",
                "cfg.alpha_i",
            ),
            Param("T_s", "T_s: Sampling period (s)", "cfg.T_s"),
        ],
    )
    params = mask_params(mask)
    start = (
        checks(params, ["L", "alpha_c", "T_s"])
        + "double alpha_i = PDIM(2) > 0 ? P(2, 0) : P(1, 0);\n"
        "self = gfl_current_controller(P(0, 0), P(1, 0), alpha_i);\n"
        "T_s = P(3, 0);\n"
    )
    output = (
        GFL_SIGNALS.read(2, "fbk", declare=False) + "double complex u_c_ref =\n"
        f"    complex_pi_compute_output(&self, {complex_in(1)}, fbk.i_c, fbk.u_g);\n"
        + complex_out(0, "u_c_ref")
    )
    return CBlock(
        "CurrentController",
        [Port(ENABLE, 1), Port("i_c_ref", 2), GFL_SIGNALS.port("fbk")],
        [Port("u_c", 2)],
        GFL_SIGNALS.class_code(
            "ComplexPIController",
            start,
            output,
            "complex_pi_update(&self, T_s, fbk.u_c, fbk.w_c);\n",
            SAMPLING + "static GFLFeedbacks fbk;\n",
            (2,),
        ),
        params,
        GFL_SIGNALS.sfunction("current_controller"),
        mask=mask,
    )


def _gfl_current_vector_controller() -> Subsystem:
    """Grid-following current-vector controller (CurrentVectorController)."""
    sig = GFL_SIGNALS
    layout = [sig.index("ref", n) for n in ("p_g", "q_g", "i_c", "u_c")]
    if layout != [0, 1, 2, 4]:
        raise RuntimeError("The layout of ref does not match its multiplexer")
    mask = Mask(
        "CurrentVectorController (motulator)",
        "Grid-following current-vector controller (CurrentVectorController) in the "
        "power-control mode. The configuration cfg is a struct with the fields of "
        "CurrentVectorControllerCfg ([] = None, i.e., the default), passed to the "
        "blocks inside it, as CurrentVectorController.",
        [
            Param(
                "cfg",
                "cfg: Configuration (CurrentVectorControllerCfg) as a struct",
                "cfg",
            )
        ],
    )
    return Subsystem(
        "CurrentVectorController",
        [
            Port(ENABLE, 1),
            Port("p_g_ref", 1),
            Port("q_g_ref", 1),
            sig.port("meas"),
            Port("u_c_ab", 2),
        ],
        [sig.port("fbk"), sig.port("ref")],
        [
            _pll(),
            sig.selector("fbk", "u_g"),
            _gfl_current_reference(),
            current_limiter(sig),
            _gfl_current_controller(),
            Mux(
                "Mux ref",
                [Port("p_g", 1), Port("q_g", 1), Port("i_c", 2), Port("u_c", 2)],
            ),
        ],
        [
            (ENABLE, "PLL:enable"),
            (ENABLE, "CurrentController:enable"),
            ("meas", "PLL:meas"),
            ("u_c_ab", "PLL:u_c_ab"),
            ("PLL:out", "fbk.u_g:u"),
            ("PLL:out", "CurrentController:fbk"),
            ("PLL:out", "fbk"),
            ("p_g_ref", "compute_output:p_g_ref"),
            ("q_g_ref", "compute_output:q_g_ref"),
            ("fbk.u_g:y", "compute_output:u_g"),
            ("compute_output:i_c", "CurrentLimiter:i"),
            ("CurrentLimiter:i_lim", "CurrentController:i_c_ref"),
            ("CurrentLimiter:i_lim", "Mux ref:i_c"),
            ("compute_output:p_g", "Mux ref:p_g"),
            ("compute_output:q_g", "Mux ref:q_g"),
            ("CurrentController:u_c", "Mux ref:u_c"),
            ("Mux ref:y", "ref"),
        ],
        columns=[
            ["p_g_ref", "q_g_ref", "meas", "u_c_ab", ENABLE],
            ["PLL"],
            ["fbk.u_g"],
            ["compute_output"],
            ["CurrentLimiter"],
            ["CurrentController"],
            ["Mux ref"],
            ["ref", "fbk"],
        ],
        align={
            "p_g_ref": "compute_output:p_g_ref-2",
            "q_g_ref": "compute_output:q_g_ref-1",
            "fbk.u_g:y": "compute_output:u_g",
            "PLL:out": "fbk.u_g:u+2",
            "meas": "PLL:meas",
            "CurrentLimiter:i": "compute_output:i_c",
            "CurrentController:i_c_ref": "CurrentLimiter:i_lim",
            "Mux ref:u_c": "CurrentController:u_c",
            "ref": "Mux ref:y",
            "fbk": "PLL:out",
        },
        lanes={
            "Mux ref:p_g": "above:1",
            "Mux ref:q_g": "above:0",
            "Mux ref:i_c": "over:CurrentController:0",
        },
        tags=[ENABLE],
        mask=mask,
    )


GFL_MONITORED = {
    "p_g_ref": "ref.p_g",
    "p_g": "fbk.p_g",
    "q_g_ref": "ref.q_g",
    "q_g": "fbk.q_g",
    "i_c_d_ref": "creal(ref.i_c)",
    "i_c_d": "creal(fbk.i_c)",
    "i_c_q_ref": "cimag(ref.i_c)",
    "i_c_q": "cimag(fbk.i_c)",
    "u_g": "fbk.u_g",
    "w_g": "fbk.w_g",
    "theta_c": "fbk.theta_c",
}


def gfl_control_system() -> Subsystem:
    """Contents of the control-system block of grid-following control."""
    measurement = get_measurement(
        GFL_SIGNALS,
        GFL_INPUTS[3:],
        [
            ("i_c_ab", "abc2complex(i_c_abc)"),
            ("u_g_ab", "line2complex(u_g_line)"),
            ("u_dc", "fmax(InputSignal(2, 0), U_DC_MIN)"),
        ],
        abc_input("i_c_abc", 0) + "/* Line-to-line PCC voltages u_ab and u_bc */\n"
        "double u_g_line[2] = {InputSignal(1, 0), InputSignal(1, 1)};\n",
    )
    signals = monitor(GFL_SIGNALS, GFL_OUTPUTS, GFL_MONITORED, [])
    return grid_converter_control_system(
        GFL_SIGNALS,
        GFL_NAME,
        GFL_INPUTS,
        measurement,
        _gfl_current_vector_controller(),
        signals,
    )


GFL_BLOCK = control_block(
    GFL_NAME,
    "Current-vector grid-following control with a PLL in the power-control mode "
    "(GridConverterControlSystem with CurrentVectorController).",
    GFL_INPUTS,
    GFL_OUTPUTS,
    gfl_control_system,
    cls="GridConverterControlSystem",
)


# %%
# Grid-forming control
def _observer() -> CBlock:
    """Disturbance observer (Observer)."""
    mask = Mask(
        "Observer (motulator)",
        "Disturbance observer (Observer), giving the feedback signals fbk in its "
        "coordinates from the measurements and the realized converter voltage.",
        [
            Param(
                "u_nom",
                "u_nom: Nominal grid voltage (V), line-to-neutral peak value",
                "cfg.u_nom",
            ),
            Param(
                "w_nom", "w_nom: Nominal grid angular frequency (rad/s)", "cfg.w_nom"
            ),
            Param("alpha_o", "alpha_o: Observer gain (rad/s)", "cfg.alpha_o"),
            Param("L", "L: Total inductance estimate (H)", "cfg.L"),
            Param("R", "R: Total series resistance estimate (Ω)", "cfg.R"),
            Param("T_s", "T_s: Sampling period (s)", "cfg.T_s"),
        ],
    )
    params = mask_params(mask)
    start = (
        checks(params, params)
        + "gfm_observer_init(&self, P(0, 0), P(1, 0), P(2, 0), P(3, 0), P(4, 0));\n"
        "T_s = P(5, 0);\n"
    )
    output = (
        GFM_SIGNALS.read(1, "meas")
        + f"gfm_observer_compute_output(&self, {complex_in(2)}, meas.i_c_ab, &out);\n"
        + GFM_SIGNALS.write(0, "fbk", "out")
    )
    fbk = GFM_SIGNALS.width("fbk")
    return CBlock(
        "Observer",
        [Port(ENABLE, 1), GFM_SIGNALS.port("meas"), Port("u_c_ab", 2)],
        [Port("out", fbk)],
        GFM_SIGNALS.class_code(
            "GFMObserver",
            start,
            output,
            "gfm_observer_update(&self, T_s, &out);\n",
            SAMPLING + "static GFMFeedbacks out;\n",
            (fbk,),
        ),
        params,
        GFM_SIGNALS.sfunction("observer"),
        mask=mask,
    )


def _gfm_current_reference() -> CBlock:
    """
    References ref.p_g (the limited active-power reference) and ref.v_c, and the
    current reference before the current limiter (compute_output), and the update
    of the active-power limitation with the voltage reference ref.u_c.
    """
    params = ["R_a", "k_v", "k_c", "i_d_max", "alpha_l", "u_nom", "T_s"]
    start = (
        checks(params, ["R_a", "k_v", "k_c", "alpha_l", "u_nom", "T_s"])
        + "self.R_a = P(0, 0);\n"
        "self.k_v = P(1, 0);\n"
        "self.k_c = P(2, 0);\n"
        "self.i_d_max = PARAM(3);\n"
        "self.alpha_l = P(4, 0);\n"
        "/* Initial maximum active power (inf without the power limitation) */\n"
        "self.p_max = isnan(self.i_d_max) ? INFINITY : 1.5 * P(5, 0) * self.i_d_max;\n"
        "T_s = P(6, 0);\n"
    )
    output = (
        GFM_SIGNALS.read(4, "fbk", declare=False) + "GFMReferences ref = {0};\n"
        "gfm_current_reference(&self, InputSignal(1, 0), InputSignal(2, 0), &fbk,\n"
        "                      &ref);\n"
        "OutputSignal(0, 0) = ref.p_g;\n"
        "OutputSignal(1, 0) = ref.v_c;\n" + complex_out(2, "ref.i_c")
    )
    update = (
        "/* Active-power limitation with the voltage reference ref.u_c */\n"
        "GFMReferences ref = {0};\n"
        f"ref.u_c = {complex_in(3)};\n"
        "gfm_power_limit_update(&self, T_s, &ref, &fbk);\n"
    )
    return CBlock(
        "compute_output",
        [
            Port(ENABLE, 1),
            Port("p_g_ref", 1),
            Port("v_c_ref", 1),
            Port("u_c", 2),
            GFM_SIGNALS.port("fbk"),
        ],
        [Port("p_g", 1), Port("v_c", 1), Port("i_c", 2)],
        GFM_SIGNALS.class_code(
            "GFMController",
            start,
            output,
            update,
            SAMPLING + "static GFMFeedbacks fbk;\n",
            (1, 1, 2),
        ),
        params,
        GFM_SIGNALS.sfunction("current_reference"),
        feedthrough=[1, 1, 1, 0, 1],
    )


def _gfm_voltage_reference() -> CBlock:
    """Voltage reference from the limited current reference (in compute_output)."""
    params = ["k_c", "R"]
    start = checks(params, params) + "self.k_c = P(0, 0);\nself.observer.R = P(1, 0);\n"
    output = (
        GFM_SIGNALS.read(1, "fbk")
        + "GFMReferences ref = {0};\n"
        + f"ref.i_c = {complex_in(0)};\n"
        + "gfm_voltage_reference(&self, &fbk, &ref);\n"
        + complex_out(0, "ref.u_c")
    )
    return CBlock(
        "voltage_reference",
        [Port("i_c", 2), GFM_SIGNALS.port("fbk")],
        [Port("u_c", 2)],
        GFM_SIGNALS.class_code("GFMController", start, output, disabled=None),
        params,
        GFM_SIGNALS.sfunction("voltage_reference"),
        sample_time=None,
    )


# Mask initialization of the ObserverBasedGridFormingController:
# ObserverBasedGridFormingControllerCfg.__post_init__ and
# ObserverBasedGridFormingController.__init__
GFM_INIT = (
    "% Defaults of ObserverBasedGridFormingControllerCfg.__post_init__\n"
    "R_a = cfg.R_a;\n"
    "if isempty(R_a)\n"
    "  R_a = 0.25*cfg.u_nom/cfg.i_max;\n"
    "end\n"
    "k_v = cfg.k_v;\n"
    "if isempty(k_v)\n"
    "  k_v = cfg.alpha_o/cfg.w_nom;\n"
    "end\n"
    "% Current-control gain (ObserverBasedGridFormingController.__init__)\n"
    "k_c = cfg.alpha_c*cfg.L;\n"
    "% Other parameters of compute_output\n"
    "R = cfg.R;\n"
    "i_d_max = cfg.i_d_max;\n"
    "alpha_l = cfg.alpha_l;\n"
    "u_nom = cfg.u_nom;\n"
    "T_s = cfg.T_s;\n"
)


def _gfm_controller() -> Subsystem:
    """Grid-forming controller (ObserverBasedGridFormingController)."""
    sig = GFM_SIGNALS
    layout = [sig.index("ref", n) for n in ("p_g", "v_c", "i_c", "u_c")]
    if layout != [0, 1, 2, 4]:
        raise RuntimeError("The layout of ref does not match its multiplexer")
    mask = Mask(
        "ObserverBasedGridFormingController (motulator)",
        "Disturbance-observer-based grid-forming controller "
        "(ObserverBasedGridFormingController) in the power-control mode, with the "
        "transparent current limitation. The configuration cfg is a struct with the "
        "fields of ObserverBasedGridFormingControllerCfg ([] = None, i.e., the "
        "default). The mask initialization resolves the defaults and passes the "
        "parameters to the blocks inside it, as ObserverBasedGridFormingController. "
        "The blocks compute_output and voltage_reference are its compute_output "
        "before and after the current limiter, and compute_output updates the "
        "active-power limitation.",
        [
            Param(
                "cfg",
                "cfg: Configuration (ObserverBasedGridFormingControllerCfg) as a "
                "struct",
                "cfg",
            )
        ],
        init=GFM_INIT,
    )
    return Subsystem(
        "ObserverBasedGridFormingController",
        [
            Port(ENABLE, 1),
            Port("p_g_ref", 1),
            Port("v_c_ref", 1),
            sig.port("meas"),
            Port("u_c_ab", 2),
        ],
        [sig.port("fbk"), sig.port("ref")],
        [
            _observer(),
            Tag("From u_c", "u_c", 2, goto=False),
            _gfm_current_reference(),
            current_limiter(sig),
            _gfm_voltage_reference(),
            Mux(
                "Mux ref",
                [Port("p_g", 1), Port("v_c", 1), Port("i_c", 2), Port("u_c", 2)],
            ),
            Tag("Goto u_c", "u_c", 2, goto=True),
        ],
        [
            (ENABLE, "Observer:enable"),
            (ENABLE, "compute_output:enable"),
            ("meas", "Observer:meas"),
            ("u_c_ab", "Observer:u_c_ab"),
            ("Observer:out", "compute_output:fbk"),
            ("Observer:out", "voltage_reference:fbk"),
            ("Observer:out", "fbk"),
            ("p_g_ref", "compute_output:p_g_ref"),
            ("v_c_ref", "compute_output:v_c_ref"),
            ("From u_c", "compute_output:u_c"),
            ("compute_output:i_c", "CurrentLimiter:i"),
            ("CurrentLimiter:i_lim", "voltage_reference:i_c"),
            ("CurrentLimiter:i_lim", "Mux ref:i_c"),
            ("compute_output:p_g", "Mux ref:p_g"),
            ("compute_output:v_c", "Mux ref:v_c"),
            ("voltage_reference:u_c", "Mux ref:u_c"),
            ("voltage_reference:u_c", "Goto u_c"),
            ("Mux ref:y", "ref"),
        ],
        columns=[
            ["p_g_ref", "v_c_ref", "meas", "u_c_ab", ENABLE],
            ["Observer"],
            ["From u_c"],
            ["compute_output"],
            ["CurrentLimiter"],
            ["voltage_reference"],
            ["Mux ref", "Goto u_c"],
            ["ref", "fbk"],
        ],
        align={
            "p_g_ref": "compute_output:p_g_ref-2",
            "v_c_ref": "compute_output:v_c_ref-1",
            "From u_c": "compute_output:u_c",
            "Observer:out": "compute_output:fbk+2",
            "meas": "Observer:meas",
            "CurrentLimiter:i": "compute_output:i_c",
            "voltage_reference:i_c": "CurrentLimiter:i_lim",
            "Mux ref:u_c": "voltage_reference:u_c",
            "ref": "Mux ref:y",
            "fbk": "Observer:out",
        },
        lanes={
            "Mux ref:p_g": "above:1",
            "Mux ref:v_c": "above:0",
            "Mux ref:i_c": "over:voltage_reference:0",
        },
        tags=[ENABLE],
        mask=mask,
    )


GFM_MONITORED = {
    "p_g_ref": "ref.p_g",
    "p_g": "fbk.p_g",
    "q_g": "fbk.q_g",
    "v_c_ref": "ref.v_c",
    "v_c": "cabs(fbk.v_c)",
    "i_c_d_ref": "creal(ref.i_c)",
    "i_c_d": "creal(fbk.i_c)",
    "i_c_q_ref": "cimag(ref.i_c)",
    "i_c_q": "cimag(fbk.i_c)",
    "theta_c": "fbk.theta_c",
}


def gfm_control_system() -> Subsystem:
    """Contents of the control-system block of grid-forming control."""
    measurement = get_measurement(
        GFM_SIGNALS,
        GFM_INPUTS[3:],
        [
            ("i_c_ab", "abc2complex(i_c_abc)"),
            ("u_dc", "fmax(InputSignal(1, 0), U_DC_MIN)"),
        ],
        abc_input("i_c_abc", 0),
    )
    signals = monitor(GFM_SIGNALS, GFM_OUTPUTS, GFM_MONITORED, [])
    return grid_converter_control_system(
        GFM_SIGNALS, GFM_NAME, GFM_INPUTS, measurement, _gfm_controller(), signals
    )


GFM_BLOCK = control_block(
    GFM_NAME,
    "Disturbance-observer-based grid-forming control in the power-control mode, "
    "with the transparent current limitation (GridConverterControlSystem with "
    "ObserverBasedGridFormingController).",
    GFM_INPUTS,
    GFM_OUTPUTS,
    gfm_control_system,
    cls="GridConverterControlSystem",
)


# %%
def init_script(
    variables: list[tuple[str, Any]], values: dict[str, Any], comment: str, gfl: bool
) -> str:
    """
    Model initialization: the variables of the system model, and the structs of the
    control system (cfg and pwm) from the parameter values of the control system
    (`grid.export_values`), of grid-following control (`gfl`) or grid-forming
    control.
    """
    if gfl:
        cfg = ["i_max", "L", "alpha_c", "alpha_i", "u_nom", "w_nom", "alpha_pll"]
        title = "% Grid-following control (CurrentVectorControllerCfg)\n"
    else:
        cfg = ["i_max", "L", "R", "R_a", "k_v", "alpha_o", "alpha_c", "u_nom"]
        cfg += ["w_nom", "i_d_max", "alpha_l"]
        title = "% Grid-forming control (ObserverBasedGridFormingControllerCfg)\n"
    return (
        init_header(variables, comment)
        + title
        + "".join(assign(f"cfg.{n}", values[n]) for n in [*cfg, "T_s"])
        + pwm_init(values)
    )
