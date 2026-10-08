"""
Current-vector control of induction machine drives: each class is a block.

The control system mirrors the classes of motulator as a hierarchy of blocks, each
with a mask of the arguments of its class (see `_control`):

- VectorControlSystem (the masked subsystem of the control system), containing the
  blocks of its methods (`get_measurement` and `modulate`), the monitored signals,
  the `PWM`, the `SpeedController`, and the `CurrentVectorController`
- CurrentVectorController(par, cfg), containing the `CurrentReferenceGenerator`,
  the `CurrentController`, and the `SpeedFluxObserver`, which contains the
  `SpeedObserver` and the `FluxObserver`

The mask initialization of the CurrentVectorController resolves the defaults of
`CurrentVectorControllerCfg` and `CurrentController`, and that of the
SpeedFluxObserver computes the gains of `SpeedFluxObserver.__init__`. The masks of
the top-level classes refer to the variables of the model initialization
(`init_script`): the machine model `par`, the configuration `cfg`, the speed
controller `speed_ctrl`, and the PWM `pwm`. The signal vectors are defined in
`im_current_vector.h`.

"""

from typing import Any

from motulator_export.plecs._common import ENABLE
from motulator_export.plecs._control import (
    SAMPLING,
    T_S,
    Signals,
    abc_input,
    assign,
    checks,
    complex_in,
    complex_out,
    control_block,
    get_measurement,
    init_header,
    mask_params,
    monitor,
    pwm_init,
    speed_ctrl_init,
    speed_observer,
    vector_control_system,
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

SIGNALS = Signals(
    ("common.c", "im_current_vector.c"),
    ("im_current_vector.h",),
    {
        "meas": ("IMMeasurements", "im_measurements"),
        "fbk": ("IMObserverOutputs", "im_observer_outputs"),
        "ref": ("IMReferences", "im_references"),
    },
    "sfun_cvc",
)

# Name of the control-system block, its inputs, and the monitored signals (mask
# probes)
NAME = "Current-vector control"
CTRL_INPUTS = [
    Port(ENABLE, 1),
    Port("w_M_ref", 1),
    Port("i_s_abc", 3),
    Port("u_dc", 1),
    Port("w_M", 1),
]
CTRL_OUTPUTS = {
    "Speed (w_M_ref, w_M)": ["w_M_ref", "w_M"],
    "Torque (tau_M_ref, tau_M)": ["tau_M_ref", "tau_M"],
    "Flux linkage (psi_s, psi_R)": ["psi_s", "psi_R"],
    "Current (i_sd_ref, i_sd, i_sq_ref, i_sq)": [
        "i_sd_ref",
        "i_sd",
        "i_sq_ref",
        "i_sq",
    ],
}

# The machine model (par) as the parameters of the C-Script: the mask
# initialization takes its fields (InductionMachineInvGammaPars)
PAR_FIELDS = ["n_p", "R_s", "R_R", "L_sgm", "L_M"]
PAR = {"par": [f"par_{f}" for f in PAR_FIELDS]}
PAR_INIT = (
    "% Fields of the machine model par (InductionMachineInvGammaPars) for the\n"
    "% C-Script\n" + "".join(f"par_{f} = par.{f};\n" for f in PAR_FIELDS)
)
PAR_PROMPT = "par: Machine model (InductionMachineInvGammaPars) as a struct"
PAR_DESCRIPTION = (
    " The machine model par is a struct with the fields n_p, R_s, R_R, L_sgm, and "
    "L_M of InductionMachineInvGammaPars."
)


def _par_code(params: list[str]) -> str:
    """C code checking the machine model par and reading it from the parameters."""
    i = params.index("par_n_p")
    return (
        checks(params, PAR["par"])
        + "/* Machine model (InductionMachineInvGammaPars) */\n"
        f"InductionMachineInvGammaPars par = {{P({i}, 0), P({i + 1}, 0), "
        f"P({i + 2}, 0),\n"
        f"                                    P({i + 3}, 0), P({i + 4}, 0)}};\n"
    )


# %%
# Blocks of the classes
def _flux_observer() -> CBlock:
    """Flux observer (FluxObserver)."""
    mask = Mask(
        "FluxObserver (motulator)",
        "Reduced-order flux observer in synchronous coordinates (FluxObserver). In "
        "the sensored mode (h = 1), the speed error signal is computed from the "
        "measured rotor speed (CurrentVectorController.get_feedback)."
        + PAR_DESCRIPTION,
        [
            Param("par", PAR_PROMPT, "par"),
            Param(
                "k_o",
                "k_o: Observer gain k_o(w_m), the default of "
                "create_speed_flux_observer in the sensorless (1) or sensored (0) mode",
                "k_o",
            ),
            Param(
                "h",
                "h: Weight of the measured rotor speed, 0 = sensorless, 1 = sensored",
                "h",
            ),
            T_S,
        ],
        init=PAR_INIT,
    )
    params = mask_params(mask, PAR)
    i = {p: k for k, p in enumerate(params)}
    start = (
        checks(params, ["k_o", "h", "T_s"])
        + _par_code(params)
        + f"im_flux_observer_init(&self, par, (int)P({i['k_o']}, 0));\n"
        f"T_s = P({i['T_s']}, 0);\n"
        f"h = P({i['h']}, 0);\n"
    )
    output = (
        SIGNALS.read(1, "meas") + f"double complex u_s_ab = {complex_in(2)};\n"
        "double w_M = InputSignal(3, 0);\n"
        "/* Speed error from the measured rotor speed in the sensored mode */\n"
        "double eps = (h == 0.0) ? 0.0 : meas.w_M - w_M;\n"
        "im_flux_observer_compute_output(&self, u_s_ab, meas.i_c_ab, w_M, eps, h,\n"
        "                                &out);\n" + SIGNALS.write(0, "fbk", "out")
    )
    return CBlock(
        "FluxObserver",
        [Port(ENABLE, 1), SIGNALS.port("meas"), Port("u_s_ab", 2), Port("w_M", 1)],
        [Port("out", SIGNALS.width("fbk"))],
        SIGNALS.class_code(
            "IMFluxObserver",
            start,
            output,
            "im_flux_observer_update(&self, T_s, &out);\n",
            workspace=SAMPLING + "static double h;\nstatic IMObserverOutputs out;\n",
            outputs=(SIGNALS.width("fbk"),),
        ),
        params,
        SIGNALS.sfunction("flux_observer"),
        mask=mask,
    )


# Mask initialization of the SpeedFluxObserver: create_speed_flux_observer and
# SpeedFluxObserver.__init__
SFO_INIT = (
    "% Gains for critically damped dynamics (SpeedFluxObserver.__init__)\n"
    "if isempty(J)\n"
    "  k_w = alpha_o;\n"
    "  k_tau = 0;\n"
    "else\n"
    "  k_w = 2*alpha_o;\n"
    "  k_tau = J*alpha_o^2;\n"
    "end\n"
    "% Default observer gain of create_speed_flux_observer, and the weight of the\n"
    "% measured rotor speed in the FluxObserver\n"
    "k_o = sensorless;\n"
    "h = 1 - sensorless;\n"
)


def _speed_flux_observer() -> Subsystem:
    """Speed and flux observer (SpeedFluxObserver)."""
    mask = Mask(
        "SpeedFluxObserver (motulator)",
        "Flux observer with speed estimation, created by create_speed_flux_observer "
        "with the default observer gain k_o. The mask initialization computes the "
        "gains of the SpeedObserver and the FluxObserver inside it, as in "
        "SpeedFluxObserver." + PAR_DESCRIPTION,
        [
            Param("par", PAR_PROMPT, "par"),
            Param("alpha_o", "alpha_o: Speed estimation pole (rad/s)", "alpha_o"),
            Param(
                "sensorless",
                "sensorless: Sensorless mode (1) or sensored mode (0)",
                "cfg.sensorless",
            ),
            Param("J", "J: Inertia (kgm²), [] = the model not used", "cfg.J"),
            Param("T_s", "T_s: Sampling period (s)", "cfg.T_s"),
        ],
        init=SFO_INIT,
    )
    return Subsystem(
        "SpeedFluxObserver",
        [Port(ENABLE, 1), SIGNALS.port("meas"), Port("u_s_ab", 2)],
        [Port("out", SIGNALS.width("fbk"))],
        [
            _flux_observer(),
            SIGNALS.selector("fbk", "eps"),
            SIGNALS.selector("fbk", "tau_M"),
            speed_observer(SIGNALS),
        ],
        [
            (ENABLE, "FluxObserver:enable"),
            (ENABLE, "SpeedObserver:enable"),
            ("meas", "FluxObserver:meas"),
            ("u_s_ab", "FluxObserver:u_s_ab"),
            ("SpeedObserver:w_M", "FluxObserver:w_M"),
            ("FluxObserver:out", "out"),
            ("FluxObserver:out", "fbk.eps:u"),
            ("FluxObserver:out", "fbk.tau_M:u"),
            ("fbk.eps:y", "SpeedObserver:eps"),
            ("fbk.tau_M:y", "SpeedObserver:tau_M"),
        ],
        columns=[
            [ENABLE, "meas", "u_s_ab"],
            ["FluxObserver"],
            ["fbk.eps", "fbk.tau_M"],
            ["SpeedObserver"],
            ["out"],
        ],
        align={
            "u_s_ab": "FluxObserver:u_s_ab",
            "out": "FluxObserver:out",
            "fbk.eps:y": "FluxObserver:w_M+1",
            "SpeedObserver:eps": "fbk.eps:y",
        },
        lanes={"FluxObserver:w_M": "below:0"},
        tags=[ENABLE],
        mask=mask,
    )


def _current_reference_generator() -> CBlock:
    """Current reference generator (CurrentReferenceGenerator)."""
    mask = Mask(
        "CurrentReferenceGenerator (motulator)",
        "Current reference generator with field weakening (CurrentReferenceGenerator). "
        "The inputs are the torque reference, the unlimited voltage reference ref.u_s "
        "and the DC-bus voltage meas.u_dc for the field weakening, and the rotor flux "
        "estimate fbk.psi_R. The outputs are the limited torque reference and "
        "the current reference, transformed to the coordinates of the feedback signals "
        "(CurrentVectorController.compute_output)." + PAR_DESCRIPTION,
        [
            Param("par", PAR_PROMPT, "par"),
            Param(
                "psi_s_nom",
                "psi_s_nom: Nominal stator flux linkage (Vs)",
                "cfg.psi_s_nom",
            ),
            Param("i_s_max", "i_s_max: Maximum stator current (A)", "cfg.i_s_max"),
            Param(
                "w_s_nom",
                "w_s_nom: Nominal stator angular frequency (rad/s)",
                "cfg.w_s_nom",
            ),
            Param("k_u", "k_u: Voltage utilization factor", "cfg.k_u"),
            Param(
                "k_fw",
                "k_fw: Field-weakening gain (1/H), [] = 2*R_R/(w_s_nom*L_sgm^2)",
                "cfg.k_fw",
            ),
            Param("T_s", "T_s: Sampling period (s)", "cfg.T_s"),
        ],
        init=PAR_INIT,
    )
    params = mask_params(mask, PAR)
    i = {p: k for k, p in enumerate(params)}
    start = (
        checks(params, ["psi_s_nom", "i_s_max", "w_s_nom", "k_u", "T_s"])
        + _par_code(params)
        + f"im_reference_gen_init(&self, par, P({i['psi_s_nom']}, 0), "
        f"P({i['i_s_max']}, 0),\n"
        f"                      P({i['w_s_nom']}, 0), P({i['k_u']}, 0), "
        f"PARAM({i['k_fw']}));\n"
        f"T_s = P({i['T_s']}, 0);\n"
    )
    output = (
        f"double complex psi_R = {complex_in(4)};\n"
        "double tau_M_ref;\n"
        "double complex i_s_ref;\n"
        "im_reference_gen_compute_output(&self, InputSignal(1, 0), cabs(psi_R),\n"
        "                                &i_s_ref, &tau_M_ref);\n"
        "/* Transform the reference to the same coordinates as the feedback */\n"
        "i_s_ref = cexp(I * carg(psi_R)) * i_s_ref;\n"
        "OutputSignal(0, 0) = tau_M_ref;\n" + complex_out(1, "i_s_ref")
    )
    update = (
        "/* Field weakening based on the unlimited voltage reference */\n"
        f"im_reference_gen_update(&self, T_s, {complex_in(2)},\n"
        "                        InputSignal(3, 0));\n"
    )
    return CBlock(
        "CurrentReferenceGenerator",
        [
            Port(ENABLE, 1),
            Port("tau_M_ref", 1),
            Port("u_s", 2),
            Port("u_dc", 1),
            Port("psi_R", 2),
        ],
        [Port("tau_M", 1), Port("i_s", 2)],
        SIGNALS.class_code(
            "IMCurrentReferenceGenerator", start, output, update, SAMPLING, (1, 2)
        ),
        params,
        SIGNALS.sfunction("current_reference_generator"),
        feedthrough=[1, 1, 0, 0, 1],
        mask=mask,
    )


def _current_controller() -> CBlock:
    """Current controller (CurrentController)."""
    mask = Mask(
        "CurrentController (motulator)",
        "2DOF PI current controller (CurrentController). The inputs are the current "
        "reference and the feedback signals fbk (the current fbk.i_s, and the realized "
        "voltage fbk.u_s and the angular speed fbk.w_c of the coordinates for the "
        "integral state), and the output is the voltage reference." + PAR_DESCRIPTION,
        [
            Param("par", PAR_PROMPT, "par"),
            Param(
                "alpha_c",
                "alpha_c: Reference-tracking bandwidth (rad/s)",
                "cfg.alpha_c",
            ),
            Param("alpha_i", "alpha_i: Integral-action bandwidth (rad/s)", "alpha_i"),
            Param("T_s", "T_s: Sampling period (s)", "cfg.T_s"),
        ],
        init=PAR_INIT,
    )
    params = mask_params(mask, PAR)
    i = {p: k for k, p in enumerate(params)}
    start = (
        checks(params, ["alpha_c", "alpha_i", "T_s"])
        + _par_code(params)
        + f"self = im_current_controller(&par, P({i['alpha_c']}, 0), "
        f"P({i['alpha_i']}, 0));\n"
        f"T_s = P({i['T_s']}, 0);\n"
    )
    output = (
        SIGNALS.read(2, "fbk", declare=False) + "double complex u_s_ref =\n"
        f"    complex_pi_compute_output(&self, {complex_in(1)}, fbk.i_s, 0.0);\n"
        + complex_out(0, "u_s_ref")
    )
    return CBlock(
        "CurrentController",
        [Port(ENABLE, 1), Port("i_s_ref", 2), SIGNALS.port("fbk")],
        [Port("u_s", 2)],
        SIGNALS.class_code(
            "ComplexPIController",
            start,
            output,
            "complex_pi_update(&self, T_s, fbk.u_s, fbk.w_c);\n",
            workspace=SAMPLING + "static IMObserverOutputs fbk;\n",
            outputs=(2,),
        ),
        params,
        SIGNALS.sfunction("current_controller"),
        mask=mask,
    )


# Mask initialization of the CurrentVectorController: CurrentVectorControllerCfg and
# CurrentController.__init__
CVC_INIT = (
    "% Default of alpha_o (CurrentVectorControllerCfg.__post_init__)\n"
    "alpha_o = cfg.alpha_o;\n"
    "if isempty(alpha_o)\n"
    "  alpha_o = 2*pi*60;\n"
    "  if ~isempty(cfg.J)\n"
    "    alpha_o = 0.5*alpha_o;\n"
    "  end\n"
    "end\n"
    "% Default of alpha_i (CurrentController.__init__)\n"
    "alpha_i = cfg.alpha_i;\n"
    "if isempty(alpha_i)\n"
    "  alpha_i = cfg.alpha_c;\n"
    "end\n"
)


def _current_vector_controller() -> Subsystem:
    """Current-vector controller (CurrentVectorController)."""
    layout = [SIGNALS.index("ref", n) for n in ("tau_M", "i_s", "u_s")]
    if layout != [0, 1, 3]:
        raise RuntimeError("The layout of ref does not match its multiplexer")
    mask = Mask(
        "CurrentVectorController (motulator)",
        "Current-vector controller (CurrentVectorController). The configuration cfg is "
        "a struct with the fields of CurrentVectorControllerCfg ([] = None, i.e., the "
        "default; the default observer gain k_o and the continuous-time current "
        "controller only). The mask initialization resolves the defaults and passes "
        "the parameters to the blocks inside it, as CurrentVectorController."
        + PAR_DESCRIPTION,
        [
            Param("par", PAR_PROMPT, "par"),
            Param(
                "cfg",
                "cfg: Configuration (CurrentVectorControllerCfg) as a struct",
                "cfg",
            ),
        ],
        init=CVC_INIT,
    )
    crg = "CurrentReferenceGenerator"
    return Subsystem(
        "CurrentVectorController",
        [
            Port(ENABLE, 1),
            Port("tau_M_ref", 1),
            SIGNALS.port("meas"),
            Port("u_c_ab", 2),
        ],
        [SIGNALS.port("fbk"), SIGNALS.port("ref")],
        [
            _speed_flux_observer(),
            Tag("From u_s", "u_s", 2, goto=False),
            SIGNALS.selector("meas", "u_dc"),
            SIGNALS.selector("fbk", "psi_R", complex_=True),
            _current_reference_generator(),
            _current_controller(),
            Mux("Mux ref", [Port("tau_M", 1), Port("i_s", 2), Port("u_s", 2)]),
            Tag("Goto u_s", "u_s", 2, goto=True),
        ],
        [
            (ENABLE, "SpeedFluxObserver:enable"),
            (ENABLE, f"{crg}:enable"),
            (ENABLE, "CurrentController:enable"),
            ("meas", "SpeedFluxObserver:meas"),
            ("u_c_ab", "SpeedFluxObserver:u_s_ab"),
            ("meas", "meas.u_dc:u"),
            ("SpeedFluxObserver:out", "fbk.psi_R:u"),
            ("SpeedFluxObserver:out", "CurrentController:fbk"),
            ("SpeedFluxObserver:out", "fbk"),
            ("tau_M_ref", f"{crg}:tau_M_ref"),
            ("fbk.psi_R:y", f"{crg}:psi_R"),
            ("From u_s", f"{crg}:u_s"),
            ("meas.u_dc:y", f"{crg}:u_dc"),
            (f"{crg}:i_s", "CurrentController:i_s_ref"),
            (f"{crg}:tau_M", "Mux ref:tau_M"),
            (f"{crg}:i_s", "Mux ref:i_s"),
            ("CurrentController:u_s", "Mux ref:u_s"),
            ("CurrentController:u_s", "Goto u_s"),
            ("Mux ref:y", "ref"),
        ],
        columns=[
            ["tau_M_ref", "meas", "u_c_ab", ENABLE],
            ["SpeedFluxObserver"],
            ["From u_s", "meas.u_dc", "fbk.psi_R"],
            [crg],
            ["CurrentController"],
            ["Mux ref", "Goto u_s"],
            ["ref", "fbk"],
        ],
        align={
            "tau_M_ref": f"{crg}:tau_M_ref",
            "From u_s": f"{crg}:u_s",
            "meas.u_dc:y": f"{crg}:u_dc+1",
            "fbk.psi_R:y": f"{crg}:psi_R+2",
            "SpeedFluxObserver:out": "fbk.psi_R:u+1",
            "meas": "SpeedFluxObserver:meas",
            "CurrentController:i_s_ref": f"{crg}:i_s",
            "Mux ref:u_s": "CurrentController:u_s",
            "ref": "Mux ref:y",
            "fbk": "SpeedFluxObserver:out",
        },
        lanes={
            "Mux ref:tau_M": "over:CurrentController:1",
            "Mux ref:i_s": "over:CurrentController:0",
            "meas.u_dc:u": "early",
        },
        tags=[ENABLE],
        mask=mask,
    )


# %%
MONITORED = {
    "w_M_ref": "InputSignal(3, 0)",
    "w_M": "fbk.w_M",
    "tau_M_ref": "ref.tau_M",
    "tau_M": "fbk.tau_M",
    "psi_s": "cabs(fbk.psi_s)",
    "psi_R": "cabs(fbk.psi_R)",
    "i_sd_ref": "creal(rot * ref.i_s)",
    "i_sd": "creal(rot * fbk.i_s)",
    "i_sq_ref": "cimag(rot * ref.i_s)",
    "i_sq": "cimag(rot * fbk.i_s)",
}


def control_system() -> Subsystem:
    """Contents of the control-system block (VectorControlSystem)."""
    measurement = get_measurement(
        SIGNALS,
        CTRL_INPUTS[2:],
        [
            ("i_c_ab", "abc2complex(i_s_abc)"),
            ("u_dc", "fmax(InputSignal(1, 0), U_DC_MIN)"),
            ("w_M", "InputSignal(2, 0)"),
        ],
        abc_input("i_s_abc", 0),
    )
    signals = monitor(
        SIGNALS,
        CTRL_OUTPUTS,
        MONITORED,
        [Port("w_M_ref", 1)],
        "/* The currents in estimated rotor flux coordinates */\n"
        "double complex rot = cexp(-I * carg(fbk.psi_R));\n",
    )
    return vector_control_system(
        SIGNALS, NAME, CTRL_INPUTS, measurement, _current_vector_controller(), signals
    )


CVC_BLOCK = control_block(
    NAME,
    "Speed control of an induction machine drive with current-vector control "
    "(VectorControlSystem with CurrentVectorController and SpeedController).",
    CTRL_INPUTS,
    CTRL_OUTPUTS,
    control_system,
)


# %%
def init_script(
    variables: list[tuple[str, Any]], values: dict[str, Any], comment: str
) -> str:
    """
    Model initialization: the variables of the system model, and the structs of the
    control system (par, cfg, speed_ctrl, and pwm) from the parameter values of the
    control system (`im.export_values`).
    """
    cfg = ["psi_s_nom", "i_s_max", "alpha_c", "alpha_i", "alpha_o", "w_s_nom"]
    cfg += ["k_u", "k_fw", "J", "sensorless", "T_s"]
    return (
        init_header(variables, comment)
        + "% Machine model of the control system (InductionMachineInvGammaPars)\n"
        + "".join(assign(f"par.{n}", values[n]) for n in PAR_FIELDS)
        + "% Current-vector control (CurrentVectorControllerCfg)\n"
        + "".join(assign(f"cfg.{n}", values[n]) for n in cfg)
        + speed_ctrl_init(values)
        + pwm_init(values)
    )
