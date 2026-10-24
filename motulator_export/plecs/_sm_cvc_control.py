"""
Current-vector control of synchronous machine drives: each class is a block.

The control system mirrors the classes of motulator as a hierarchy of blocks, each
with a mask of the arguments of its class (see `_control`):

- VectorControlSystem (the masked subsystem of the control system), containing the
  blocks of its methods (`get_measurement` and `modulate`), the monitored signals,
  the `PWM`, the `SpeedController`, and the `CurrentVectorController`
- CurrentVectorController(par, cfg), containing the `ReferenceGenerator` (with the
  current reference, its state), the `CurrentController`, and the
  `SpeedFluxObserver` of flux-vector control

When the observer catches the machine after the release of the gates (the flag
`caught` of the feedback signals), the ReferenceGenerator and the CurrentController
restart from the zero-torque operating point of the caught speed
(`CurrentVectorController.get_feedback`). The mask of the VectorControlSystem has
its arguments from the model initialization (`init_script`): the vector controller
`vector_ctrl` (the machine model `par` and the configuration `cfg`), the speed
controller `speed_ctrl`, and the PWM `pwm`. The measurements and the feedback
signals are those of flux-vector control (`sm_flux_vector.h`), and the references
are defined in `sm_current_vector.h`.

"""

from typing import Any

from motulator_export.plecs._control import (
    PAR_CFG,
    SAMPLING,
    Signals,
    abc_input,
    assign,
    checks,
    complex_in,
    complex_out,
    control_block,
    get_measurement,
    init_header,
    inner_ctrl_init,
    mask_params,
    monitor,
    pwm_init,
    speed_ctrl_init,
    vector_control_system,
)
from motulator_export.plecs._netlist import (
    COMPLEX,
    CBlock,
    Mask,
    MCode,
    Mux,
    Param,
    Port,
    Subsystem,
    required,
)
from motulator_export.plecs._sm_control import CTRL_INPUTS as FVC_INPUTS
from motulator_export.plecs._sm_control import (
    LUTS_INIT,
    LUTS_SIMULINK_INIT,
    M_PAR,
    M_PAR_CHECKS,
    M_PAR_FUNCTION,
    M_REFERENCE_GEN_CHECKS,
    M_REFERENCE_GEN_START,
    PAR,
    PAR_DESCRIPTION,
    PAR_FIELDS,
    PAR_INIT,
    PAR_PROMPT,
    _par_code,
    _reference_gen_start,
    _speed_flux_observer,
)
from motulator_export.plecs._sm_control import SIGNALS as FVC_SIGNALS

# The measurements and the feedback signals of flux-vector control, with the
# references of current-vector control
SIGNALS = Signals(
    (*FVC_SIGNALS.sources, "sm_current_vector.c"),
    (*FVC_SIGNALS.headers, "sm_current_vector.h"),
    {
        "meas": FVC_SIGNALS.vectors["meas"],
        "fbk": FVC_SIGNALS.vectors["fbk"],
        "ref": ("References", "cvc_references", "CVC_REF"),
    },
    "sfun_smcvc",
)

# Name of the control-system block, its inputs, and the monitored signals (mask
# probes)
NAME = "Current-vector control (SM)"
CTRL_INPUTS = FVC_INPUTS
CTRL_OUTPUTS = {
    "Speed (w_M_ref, w_M)": ["w_M_ref", "w_M"],
    "Torque (tau_M_ref, tau_M)": ["tau_M_ref", "tau_M"],
    "Flux linkage (psi_s_ref, psi_s)": ["psi_s_ref", "psi_s"],
    "Current (i_d_ref, i_d, i_q_ref, i_q)": ["i_d_ref", "i_d", "i_q_ref", "i_q"],
}


# %%
# Blocks of the classes
def _reference_generator() -> CBlock:
    """Reference generator with the current reference (ReferenceGenerator)."""
    mask = Mask(
        "ReferenceGenerator (motulator)",
        "Flux, torque, and current reference generator (ReferenceGenerator) with the "
        "lookup tables of the MTPA, MTPV, and current-limit loci, computed by the "
        "function sm_reference_luts.m of the library in the mask initialization "
        "(Simulink) or by the block at the start (PLECS). The output ref is [psi_s "
        "tau_M], the flux reference and the limited torque reference, and the output "
        "i_s is the current reference, a state driven toward them by the tracking "
        "law with the bandwidth alpha_ref. When the observer catches the machine "
        "(fbk.caught), the current reference restarts from zero." + PAR_DESCRIPTION,
        [
            Param("par", PAR_PROMPT, "par"),
            Param("i_s_max", "i_s_max: Maximum stator current (A)", "cfg.i_s_max"),
            Param(
                "psi_s_min",
                "psi_s_min: Minimum stator flux (Vs), [] = psi_f",
                "cfg.psi_s_min",
            ),
            Param("psi_s_max", "psi_s_max: Maximum stator flux (Vs)", "cfg.psi_s_max"),
            Param("k_u", "k_u: Voltage utilization factor", "cfg.k_u"),
            Param("k_mtpv", "k_mtpv: MTPV margin", "cfg.k_mtpv"),
            Param(
                "alpha_ref",
                "alpha_ref: Bandwidth of the current-reference tracking (rad/s)",
                "cfg.alpha_ref",
            ),
            Param("T_s", "T_s: Sampling period (s)", "cfg.T_s"),
        ],
        init=PAR_INIT + LUTS_INIT,
        simulink_init=LUTS_SIMULINK_INIT,
    )
    params = [*mask_params(mask, PAR), "luts"]
    i = {p: k for k, p in enumerate(params)}
    start = (
        checks(params, ["alpha_ref", "T_s"])
        + _reference_gen_start(params)
        + f"self.alpha = P({i['alpha_ref']}, 0);\n"
        f"T_s = P({i['T_s']}, 0);\n"
    )
    output = (
        SIGNALS.read(2, "fbk")
        + "/* Restart from zero current when the observer catches the machine\n"
        " * (CurrentVectorController.get_feedback) */\n"
        "if (fbk.caught) {\n"
        "    self.i_s_ref = 0.0;\n"
        "}\n"
        "/* Flux and limited torque references (ref.psi_s, ref.tau_M), and the\n"
        " * current reference (ref.i_s) */\n"
        "double psi_s_ref, tau_M_ref;\n"
        "reference_gen_compute_flux_and_torque_refs(&self, InputSignal(0, 0), "
        "fbk.w_m,\n"
        "    InputSignal(1, 0), &psi_s_ref, &tau_M_ref);\n"
        "double complex i_s_ref =\n"
        "    reference_gen_compute_current_ref(&self, psi_s_ref, tau_M_ref);\n"
        "OutputSignal(0, 0) = psi_s_ref;\n"
        "OutputSignal(0, 1) = tau_M_ref;\n" + complex_out(1, "i_s_ref")
    )
    return CBlock(
        "ReferenceGenerator",
        [Port("tau_M_ref", 1), Port("u_dc", 1), SIGNALS.port("fbk")],
        [Port("ref", 2), Port("i_s", 2)],
        SIGNALS.class_code(
            "ReferenceGenerator",
            start,
            output,
            "reference_gen_update(&self, T_s);\n",
            workspace=SAMPLING,
        ),
        params,
        SIGNALS.sfunction("reference_generator"),
        mask=mask,
        mcode=MCode(
            start=M_REFERENCE_GEN_START + "self.alpha = p.alpha_ref;\n",
            output="% Restart from zero current when the observer catches the "
            "machine\n"
            "% (CurrentVectorController.get_feedback)\n"
            "if fbk.caught\n"
            "    self.i_s_ref = complex(0);\n"
            "end\n"
            "% Flux and limited torque references (ref.psi_s, ref.tau_M), and the "
            "current\n"
            "% reference (ref.i_s)\n"
            "[psi_s_ref, tau_M_lim] = reference_gen_compute_flux_and_torque_refs( ...\n"
            "    self, tau_M_ref, fbk.w_m, u_dc);\n"
            "[self, i_s] = reference_gen_compute_current_ref(self, psi_s_ref, "
            "tau_M_lim);\n"
            "ref = [psi_s_ref; tau_M_lim];\n",
            update="self = reference_gen_update(self, p.T_s);\n",
            inputs={"fbk": SIGNALS.vector("fbk")},
            outputs={"i_s": COMPLEX},
            functions=M_PAR_FUNCTION,
            checks=(*required("alpha_ref", "T_s"), *M_REFERENCE_GEN_CHECKS),
        ),
    )


def _current_controller() -> CBlock:
    """Current controller (CurrentController)."""
    mask = Mask(
        "CurrentController (motulator)",
        "Flux-linkage-based 2DOF PI current controller (CurrentController): the "
        "currents are mapped to the flux linkages by the machine model. The inputs "
        "are the current reference and the feedback signals fbk (the current fbk.i_s, "
        "and the realized voltage fbk.u_s and the angular speed fbk.w_c of the "
        "coordinates for the integral state), and the output is the voltage "
        "reference. When the observer catches the machine (fbk.caught), the integral "
        "state is preset to the back-EMF of the caught speed." + PAR_DESCRIPTION,
        [
            Param("par", PAR_PROMPT, "par"),
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
        init=PAR_INIT,
    )
    params = mask_params(mask, PAR)
    i = {p: k for k, p in enumerate(params)}
    start = (
        checks(params, ["alpha_c", "T_s"])
        + _par_code(i["par_n_p"])
        + f"sm_current_ctrl_init(&self, par, P({i['alpha_c']}, 0), "
        f"PARAM({i['alpha_i']}));\n"
        f"T_s = P({i['T_s']}, 0);\n"
    )
    output = (
        SIGNALS.read(1, "fbk", declare=False)
        + "/* Preset at the back-EMF of the caught speed\n"
        " * (CurrentVectorController.get_feedback) */\n"
        "if (fbk.caught) {\n"
        "    sm_current_ctrl_preset(&self, I * fbk.w_m * fbk.psi_f);\n"
        "}\n"
        "double complex u_s_ref =\n"
        f"    sm_current_ctrl_compute_output(&self, {complex_in(0)}, fbk.i_s);\n"
        + complex_out(0, "u_s_ref")
    )
    return CBlock(
        "CurrentController",
        [Port("i_s_ref", 2), SIGNALS.port("fbk")],
        [Port("u_s", 2)],
        SIGNALS.class_code(
            "SMCurrentController",
            start,
            output,
            "sm_current_ctrl_update(&self, T_s, fbk.u_s, fbk.w_c);\n",
            workspace=SAMPLING + "static ObserverOutputs fbk;\n",
        ),
        params,
        SIGNALS.sfunction("current_controller"),
        mask=mask,
        mcode=MCode(
            start=M_PAR + "self = sm_current_ctrl_init(par, p.alpha_c, p.alpha_i);\n",
            output="% Preset at the back-EMF of the caught speed\n"
            "% (CurrentVectorController.get_feedback)\n"
            "if fbk.caught\n"
            "    self = sm_current_ctrl_preset(self, 1i*fbk.w_m*fbk.psi_f);\n"
            "end\n"
            "[self, u_s] = sm_current_ctrl_compute_output(self, i_s_ref, fbk.i_s);\n",
            update="self = sm_current_ctrl_update(self, p.T_s, fbk.u_s, fbk.w_c);\n",
            inputs={"i_s_ref": COMPLEX, "fbk": SIGNALS.vector("fbk")},
            outputs={"u_s": COMPLEX},
            functions=M_PAR_FUNCTION,
            checks=(*required("alpha_c", "T_s"), *M_PAR_CHECKS),
        ),
    )


def _current_vector_controller() -> Subsystem:
    """Current-vector controller (CurrentVectorController)."""
    layout = [SIGNALS.index("ref", n) for n in ("psi_s", "tau_M", "i_s", "u_s")]
    if layout != [0, 1, 2, 4]:
        raise RuntimeError("The layout of ref does not match its multiplexer")
    mask = Mask(
        "CurrentVectorController (motulator)",
        "Current-vector controller (CurrentVectorController). The configuration cfg is "
        "a struct with the fields of CurrentVectorControllerCfg ([] = None, i.e., the "
        "default; the offline reference generation, k_f = [], and the "
        "continuous-time current controller only). The parameters are passed to the "
        "blocks inside it, as CurrentVectorController, and the SpeedFluxObserver "
        "resolves the defaults of create_speed_flux_observer." + PAR_DESCRIPTION,
        [
            Param("par", PAR_PROMPT, "vector_ctrl.par"),
            Param(
                "cfg",
                "cfg: Configuration (CurrentVectorControllerCfg) as a struct",
                "vector_ctrl.cfg",
            ),
        ],
    )
    return Subsystem(
        "CurrentVectorController",
        [
            Port("tau_M_ref", 1),
            SIGNALS.port("meas"),
            Port("u_c_ab", 2),
            Port("u_c_zoh_ab", 2),
        ],
        [SIGNALS.port("fbk"), SIGNALS.port("ref")],
        [
            _speed_flux_observer(SIGNALS),
            SIGNALS.selector("meas", "u_dc"),
            _reference_generator(),
            _current_controller(),
            Mux("Mux ref", [Port("ref", 2), Port("i_s", 2), Port("u_s", 2)]),
        ],
        [
            ("meas", "SpeedFluxObserver:meas"),
            ("u_c_ab", "SpeedFluxObserver:u_s_ab"),
            ("u_c_zoh_ab", "SpeedFluxObserver:u_s_zoh_ab"),
            ("meas", "meas.u_dc:u"),
            ("SpeedFluxObserver:out", "ReferenceGenerator:fbk"),
            ("SpeedFluxObserver:out", "CurrentController:fbk"),
            ("SpeedFluxObserver:out", "fbk"),
            ("tau_M_ref", "ReferenceGenerator:tau_M_ref"),
            ("meas.u_dc:y", "ReferenceGenerator:u_dc"),
            ("ReferenceGenerator:i_s", "CurrentController:i_s_ref"),
            ("ReferenceGenerator:ref", "Mux ref:ref"),
            ("ReferenceGenerator:i_s", "Mux ref:i_s"),
            ("CurrentController:u_s", "Mux ref:u_s"),
            ("Mux ref:y", "ref"),
        ],
        columns=[
            ["tau_M_ref", "meas", "u_c_ab", "u_c_zoh_ab"],
            ["SpeedFluxObserver"],
            ["meas.u_dc"],
            ["ReferenceGenerator"],
            ["CurrentController"],
            ["Mux ref"],
            ["ref", "fbk"],
        ],
        align={
            "tau_M_ref": "ReferenceGenerator:tau_M_ref",
            "meas.u_dc:y": "ReferenceGenerator:u_dc+1",
            "SpeedFluxObserver:out": "ReferenceGenerator:fbk+2",
            "meas": "SpeedFluxObserver:meas",
            "CurrentController:i_s_ref": "ReferenceGenerator:i_s",
            "Mux ref:u_s": "CurrentController:u_s",
            "ref": "Mux ref:y",
            "fbk": "SpeedFluxObserver:out",
        },
        lanes={
            "Mux ref:ref": "over:CurrentController:1",
            "Mux ref:i_s": "over:CurrentController:0",
            "meas.u_dc:u": "early",
        },
        mask=mask,
    )


# %%
MONITORED = {
    "w_M_ref": "InputSignal(2, 0)",
    "w_M": "fbk.w_M",
    "tau_M_ref": "ref.tau_M",
    "tau_M": "fbk.tau_M",
    "psi_s_ref": "ref.psi_s",
    "psi_s": "cabs(fbk.psi_s)",
    "i_d_ref": "creal(ref.i_s)",
    "i_d": "creal(fbk.i_s)",
    "i_q_ref": "cimag(ref.i_s)",
    "i_q": "cimag(fbk.i_s)",
}


def control_system() -> Subsystem:
    """Contents of the control-system block (VectorControlSystem)."""
    measurement = get_measurement(
        SIGNALS,
        CTRL_INPUTS[2:],
        [
            ("i_c_ab", "abc2complex(i_s_abc)"),
            ("u_dc", "fmax(InputSignal(1, 0), U_DC_MIN)"),
            ("theta_M", "InputSignal(2, 0)"),
        ],
        abc_input("i_s_abc", 0),
    )
    signals = monitor(SIGNALS, CTRL_OUTPUTS, MONITORED, [Port("w_M_ref", 1)])
    return vector_control_system(
        SIGNALS, NAME, CTRL_INPUTS, measurement, _current_vector_controller(), signals
    )


CVC_BLOCK = control_block(
    NAME,
    "Speed control of a synchronous machine drive with current-vector control "
    "(VectorControlSystem with CurrentVectorController and SpeedController).",
    CTRL_INPUTS,
    CTRL_OUTPUTS,
    control_system,
    "CurrentVectorController",
)


# %%
# Fields of the configuration of current-vector control (CurrentVectorControllerCfg,
# without k_f), in the order of the model initialization and of the native library
CFG_FIELDS = ["i_s_max", "alpha_c", "alpha_i", "alpha_o", "alpha_ref", "k_o"]
CFG_FIELDS += ["psi_s_min", "psi_s_max", "k_u", "k_mtpv", "J", "sensorless", "T_s"]


def init_script(
    variables: list[tuple[str, Any]], values: dict[str, Any], comment: str
) -> str:
    """
    Model initialization: the variables of the system model, and the structs of the
    control system (par, cfg, vector_ctrl, speed_ctrl, and pwm) from the parameter
    values of the control system (`_sm.export_cvc_values`).
    """
    flux_map = values["psi_s_dq_fcn"]
    machine = PAR_FIELDS if flux_map is None else PAR_FIELDS[:2]
    k_f = CFG_FIELDS.index("k_o") + 1  # cfg.k_f follows cfg.k_o
    return (
        init_header(variables, comment)
        + "% Machine model of the control system (SynchronousMachinePars"
        + (")\n" if flux_map is None else ", here with a flux map)\n")
        + "".join(assign(f"par.{n}", values[n]) for n in machine)
        + ("" if flux_map is None else f"par.psi_s_dq_fcn = {flux_map};\n")
        + "% Current-vector control (CurrentVectorControllerCfg)\n"
        + "".join(assign(f"cfg.{n}", values[n]) for n in CFG_FIELDS[:k_f])
        + "cfg.k_f = [];\n"
        + "".join(assign(f"cfg.{n}", values[n]) for n in CFG_FIELDS[k_f:])
        + inner_ctrl_init(
            "vector_ctrl",
            "CurrentVectorController",
            "Current-vector controller",
            PAR_CFG,
        )
        + speed_ctrl_init(values)
        + pwm_init(values)
    )
