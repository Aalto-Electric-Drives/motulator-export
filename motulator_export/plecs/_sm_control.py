"""
Flux-vector control of synchronous machine drives: each class is a block.

The control system mirrors the classes of motulator as a hierarchy of blocks, each
with a mask of the arguments of its class (see `_control`):

- VectorControlSystem (the masked subsystem of the control system), containing the
  blocks of its methods (`get_measurement` and `modulate`), the monitored signals,
  the `PWM`, the `SpeedController`, and the `FluxVectorController`
- FluxVectorController(par, cfg), containing the `ReferenceGenerator`, the
  `FluxTorqueController`, and the `SpeedFluxObserver`, which contains the
  `SpeedObserver` and the `FluxObserver`

The mask initialization of the FluxVectorController resolves the defaults of
`FluxVectorControllerCfg` and `FluxVectorController.__init__`, and that of the
SpeedFluxObserver computes the gains of `create_speed_flux_observer` and
`SpeedFluxObserver.__init__`. The masks of the top-level classes refer to the
variables of the model initialization (`init_script`): the machine model `par`, the
configuration `cfg`, the speed controller `speed_ctrl`, and the PWM `pwm`. The
signal vectors are defined in `sm_flux_vector.h`.

"""

from typing import Any

from motulator_export.plecs._common import ENABLE, GRADNET_FIELDS
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
from motulator_export.plecs._netlist import CBlock, Mask, Mux, Param, Port, Subsystem

SIGNALS = Signals(
    (
        "common.c",
        "gradnet.c",
        "sm_parameters.c",
        "sm_control_loci.c",
        "sm_flux_vector.c",
    ),
    ("sm_flux_vector.h",),
    {
        "meas": ("Measurements", "measurements"),
        "fbk": ("ObserverOutputs", "observer_outputs"),
        "ref": ("References", "references"),
    },
    "sfun_fvc",
)

# Name of the control-system block, its inputs, and the monitored signals (mask
# probes)
NAME = "Flux-vector control"
CTRL_INPUTS = [
    Port(ENABLE, 1),
    Port("w_M_ref", 1),
    Port("i_s_abc", 3),
    Port("u_dc", 1),
    Port("theta_M", 1),
]
CTRL_OUTPUTS = {
    "Speed (w_M_ref, w_M)": ["w_M_ref", "w_M"],
    "Torque (tau_M_ref, tau_M)": ["tau_M_ref", "tau_M"],
    "Flux linkage (psi_s_ref, psi_s)": ["psi_s_ref", "psi_s"],
    "Angle and current (theta_m, i_d, i_q)": ["theta_m", "i_d", "i_q"],
}

# The machine model (par) as the parameters of the C-Script: the mask
# initialization takes its fields (SynchronousMachinePars, or
# SaturatedSynchronousMachinePars with a GradNet flux map psi_s_dq_fcn)
PAR_PARAMS = ["par_n_p", "par_R_s", "par_L_d", "par_L_q", "par_psi_f"]
PAR_PARAMS += [f"gn_{f}" for f in GRADNET_FIELDS]
PAR = {"par": PAR_PARAMS}
PAR_INIT = (
    "% Fields of the machine model par for the C-Script: SynchronousMachinePars, or\n"
    "% SaturatedSynchronousMachinePars with a GradNet flux map psi_s_dq_fcn\n"
    "par_n_p = par.n_p;\n"
    "par_R_s = par.R_s;\n"
    "if isfield(par, 'psi_s_dq_fcn') && ~isempty(par.psi_s_dq_fcn)\n"
    "  par_L_d = [];\n"
    "  par_L_q = [];\n"
    "  par_psi_f = [];\n"
    + "".join(f"  gn_{f} = par.psi_s_dq_fcn.{f};\n" for f in GRADNET_FIELDS)
    + "else\n"
    "  par_L_d = par.L_d;\n"
    "  par_L_q = par.L_q;\n"
    "  par_psi_f = par.psi_f;\n"
    + "".join(f"  gn_{f} = [];\n" for f in GRADNET_FIELDS)
    + "end\n"
)
PAR_PROMPT = (
    "par: Machine model (SynchronousMachinePars, or SaturatedSynchronousMachinePars "
    "with a GradNet flux map psi_s_dq_fcn) as a struct"
)
PAR_DESCRIPTION = (
    " The machine model par is a struct with the fields n_p, R_s, L_d, L_q, and "
    "psi_f of SynchronousMachinePars, or n_p, R_s, and psi_s_dq_fcn (a GradNet flux "
    "map) of SaturatedSynchronousMachinePars."
)


def _par_code(i: int) -> str:
    """C code of the machine model par from the parameters PAR_PARAMS from i on."""
    i_gn = i + 5
    return (
        "/* Machine model: SaturatedSynchronousMachinePars with a GradNet flux map,\n"
        " * or SynchronousMachinePars with constant inductances */\n"
        "SynchronousMachinePars par;\n"
        f"if (PDIM({i_gn}) > 0) {{\n"
        "    GradNet flux_map;\n"
        f"    CHECK_GRADNET({i_gn});\n"
        f"    READ_GRADNET(flux_map, {i_gn});\n"
        f"    par = saturated_synchronous_machine_pars(P({i}, 0), P({i + 1}, 0),\n"
        "                                             &flux_map);\n"
        "} else {\n"
        f"    if (PDIM({i + 2}) != 1 || PDIM({i + 3}) != 1 || PDIM({i + 4}) != 1) {{\n"
        '        SetErrorMessage("L_d, L_q, and psi_f needed without a flux map.");\n'
        "        return;\n"
        "    }\n"
        f"    par = synchronous_machine_pars(P({i}, 0), P({i + 1}, 0), P({i + 2}, 0),\n"
        f"                                   P({i + 3}, 0), P({i + 4}, 0));\n"
        "}\n"
    )


# %%
# Blocks of the classes
def _flux_observer() -> CBlock:
    """Flux observer (FluxObserver)."""
    mask = Mask(
        "FluxObserver (motulator)",
        "Flux observer in estimated rotor coordinates (FluxObserver). In the sensored "
        "mode (h = 1), the position error signal is computed from the measured rotor "
        "angle (position_error)." + PAR_DESCRIPTION,
        [
            Param("par", PAR_PROMPT, "par"),
            Param("k_theta", "k_theta: Rotor angle estimation gain (rad/s)", "k_theta"),
            Param(
                "k_o",
                "k_o: Observer gain [k0 k1], k_o(w_m) = k0 + k1*abs(w_m), k0 = NaN: "
                "sigma0 of the machine model",
                "k_o_obs",
            ),
            Param("k_f", "k_f: PM-flux estimation gain, only [] (zero)", "k_f"),
            Param(
                "h",
                "h: Weight of the measured rotor angle, 0 = sensorless, 1 = sensored",
                "h",
            ),
            T_S,
        ],
        init=PAR_INIT,
    )
    params = mask_params(mask, PAR)
    i = {p: k for k, p in enumerate(params)}
    k_o, k_f = i["k_o"], i["k_f"]
    start = (
        checks(params, ["k_theta", "h", "T_s"]) + f"if (PDIM({k_o}) != 2) {{\n"
        '    SetErrorMessage("k_o must be [k0 k1].");\n'
        "    return;\n"
        "}\n"
        f"if (PDIM({k_f}) > 0 && P({k_f}, 0) != 0.0) {{\n"
        '    SetErrorMessage("Only k_f = [] (zero) supported.");\n'
        "    return;\n"
        "}\n" + _par_code(i["par_n_p"]) + f"double k_o[2] = {{P({k_o}, 0), "
        f"P({k_o}, 1)}};\n"
        "if (isnan(k_o[0])) {\n"
        "    /* Poles at zero speed at s = 0 and s = -2*sigma0 */\n"
        "    IncrIndMat L_s0 = incr_ind_mat(&par, 0.0);\n"
        "    k_o[0] = 0.25 * par.R_s * (1.0 / L_s0.L_dd + 1.0 / L_s0.L_qq);\n"
        "}\n"
        f"flux_observer_init(&self, par, P({i['k_theta']}, 0), k_o);\n"
        f"T_s = P({i['T_s']}, 0);\n"
        f"h = P({i['h']}, 0);\n"
    )
    output = (
        SIGNALS.read(1, "meas") + f"double complex u_s_ab = {complex_in(2)};\n"
        f"double complex u_s_zoh_ab = {complex_in(3)};\n"
        "double w_M = InputSignal(4, 0);\n"
        "if (h == 0.0) {\n"
        "    flux_observer_compute_output(&self, u_s_ab, u_s_zoh_ab, meas.i_c_ab,\n"
        "                                 w_M, 0.0, 0.0, &out);\n"
        "} else {\n"
        "    /* Position error from the measured rotor angle (position_error) */\n"
        "    double n_p = self.par.n_p;\n"
        "    double eps = wrap(n_p * meas.theta_M - self.theta_m) / n_p;\n"
        "    flux_observer_compute_output(&self, u_s_ab, u_s_zoh_ab, meas.i_c_ab,\n"
        "                                 w_M, eps, h, &out);\n"
        "}\n" + SIGNALS.write(0, "fbk", "out")
    )
    return CBlock(
        "FluxObserver",
        [
            Port(ENABLE, 1),
            SIGNALS.port("meas"),
            Port("u_s_ab", 2),
            Port("u_s_zoh_ab", 2),
            Port("w_M", 1),
        ],
        [Port("out", SIGNALS.width("fbk"))],
        SIGNALS.class_code(
            "FluxObserver",
            start,
            output,
            "flux_observer_update(&self, T_s, &out);\n",
            workspace=SAMPLING + "static double h;\nstatic ObserverOutputs out;\n",
            outputs=(SIGNALS.width("fbk"),),
        ),
        params,
        SIGNALS.sfunction("flux_observer"),
        mask=mask,
    )


# Mask initialization of the SpeedFluxObserver: create_speed_flux_observer and
# SpeedFluxObserver.__init__
SFO_INIT = (
    "% Default observer gain of create_speed_flux_observer, k_o(w_m) = k0 +\n"
    "% k1*abs(w_m) (k0 = NaN: sigma0 of the machine model, see the FluxObserver)\n"
    "k_o_obs = k_o;\n"
    "if isempty(k_o_obs)\n"
    "  if sensorless\n"
    "    k_o_obs = [NaN 0.2];\n"
    "  else\n"
    "    k_o_obs = [2*pi*15 0];\n"
    "  end\n"
    "end\n"
    "% Gains for critically damped dynamics (SpeedFluxObserver.__init__)\n"
    "if isempty(J)\n"
    "  k_theta = 2*alpha_o;\n"
    "  k_w = alpha_o^2;\n"
    "  k_tau = 0;\n"
    "else\n"
    "  k_theta = 3*alpha_o;\n"
    "  k_w = 3*alpha_o^2;\n"
    "  k_tau = J*alpha_o^3;\n"
    "end\n"
    "% Weight of the measured rotor angle in the FluxObserver (position_error)\n"
    "h = 1 - sensorless;\n"
)


def _speed_flux_observer() -> Subsystem:
    """Speed and flux observer (SpeedFluxObserver)."""
    mask = Mask(
        "SpeedFluxObserver (motulator)",
        "Flux observer with speed estimation, created by create_speed_flux_observer. "
        "The mask initialization computes the gains of the SpeedObserver and the "
        "FluxObserver inside it, as in SpeedFluxObserver." + PAR_DESCRIPTION,
        [
            Param("par", PAR_PROMPT, "par"),
            Param("alpha_o", "alpha_o: Speed estimation pole (rad/s)", "alpha_o"),
            Param(
                "k_o",
                "k_o: Observer gain [k0 k1], k_o(w_m) = k0 + k1*abs(w_m), [] = default",
                "cfg.k_o",
            ),
            Param("k_f", "k_f: PM-flux estimation gain, only [] (zero)", "cfg.k_f"),
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
    fbk = SIGNALS.width("fbk")
    return Subsystem(
        "SpeedFluxObserver",
        [
            Port(ENABLE, 1),
            SIGNALS.port("meas"),
            Port("u_s_ab", 2),
            Port("u_s_zoh_ab", 2),
        ],
        [Port("out", fbk)],
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
            ("u_s_zoh_ab", "FluxObserver:u_s_zoh_ab"),
            ("SpeedObserver:w_M", "FluxObserver:w_M"),
            ("FluxObserver:out", "out"),
            ("FluxObserver:out", "fbk.eps:u"),
            ("FluxObserver:out", "fbk.tau_M:u"),
            ("fbk.eps:y", "SpeedObserver:eps"),
            ("fbk.tau_M:y", "SpeedObserver:tau_M"),
        ],
        columns=[
            [ENABLE, "meas", "u_s_ab", "u_s_zoh_ab"],
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


def _reference_generator() -> CBlock:
    """Flux and torque reference generator (ReferenceGenerator)."""
    mask = Mask(
        "ReferenceGenerator (motulator)",
        "Flux and torque reference generator (ReferenceGenerator) with the lookup "
        "tables of the MTPA, MTPV, and current-limit loci, computed at the start. "
        "The output ref is [psi_s tau_M], the flux reference and the limited torque "
        "reference." + PAR_DESCRIPTION,
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
        ],
        init=PAR_INIT,
    )
    params = mask_params(mask, PAR)
    i = {p: k for k, p in enumerate(params)}
    start = (
        checks(params, ["i_s_max", "psi_s_max", "k_u", "k_mtpv"])
        + _par_code(i["par_n_p"])
        + f"reference_gen_init(&self, &par, P({i['i_s_max']}, 0), "
        f"PARAM({i['psi_s_min']}),\n"
        f"                   P({i['psi_s_max']}, 0), P({i['k_u']}, 0), "
        f"P({i['k_mtpv']}, 0));\n"
    )
    output = (
        "/* Flux and limited torque references (ref.psi_s, ref.tau_M) */\n"
        "double psi_s_ref, tau_M_ref;\n"
        "reference_gen_compute_flux_and_torque_refs(&self, InputSignal(0, 0),\n"
        "    InputSignal(2, 0), InputSignal(1, 0), &psi_s_ref, &tau_M_ref);\n"
        "OutputSignal(0, 0) = psi_s_ref;\n"
        "OutputSignal(0, 1) = tau_M_ref;\n"
    )
    return CBlock(
        "ReferenceGenerator",
        [Port("tau_M_ref", 1), Port("u_dc", 1), Port("w_m", 1)],
        [Port("ref", 2)],
        SIGNALS.class_code("ReferenceGenerator", start, output, disabled=None),
        params,
        SIGNALS.sfunction("reference_generator"),
        sample_time=None,
        mask=mask,
    )


def _flux_torque_controller() -> CBlock:
    """Flux and torque controller (FluxTorqueController)."""
    mask = Mask(
        "FluxTorqueController (motulator)",
        "Flux and torque controller (FluxTorqueController). The input ref is "
        "[psi_s tau_M], the flux and torque references, and the output is the "
        "stator voltage reference in rotor coordinates." + PAR_DESCRIPTION,
        [
            Param("par", PAR_PROMPT, "par"),
            Param(
                "alpha_psi", "alpha_psi: Flux-control bandwidth (rad/s)", "alpha_psi"
            ),
            Param(
                "alpha_tau",
                "alpha_tau: Torque-control bandwidth (rad/s)",
                "cfg.alpha_tau",
            ),
            Param("alpha_i", "alpha_i: Integral-action bandwidth (rad/s)", "alpha_i"),
            Param("T_s", "T_s: Sampling period (s)", "cfg.T_s"),
        ],
        init=PAR_INIT,
    )
    params = mask_params(mask, PAR)
    i = {p: k for k, p in enumerate(params)}
    start = (
        checks(params, ["alpha_psi", "alpha_tau", "alpha_i", "T_s"])
        + _par_code(i["par_n_p"])
        + f"flux_torque_ctrl_init(&self, par, P({i['alpha_psi']}, 0), "
        f"P({i['alpha_tau']}, 0),\n"
        f"                      P({i['alpha_i']}, 0));\n"
        f"T_s = P({i['T_s']}, 0);\n"
    )
    output = (
        SIGNALS.read(2, "fbk", declare=False) + "double complex u_s_ref =\n"
        "    flux_torque_ctrl_compute_output(&self, InputSignal(1, 0), "
        "InputSignal(1, 1), &fbk);\n" + complex_out(0, "u_s_ref")
    )
    return CBlock(
        "FluxTorqueController",
        [Port(ENABLE, 1), Port("ref", 2), SIGNALS.port("fbk")],
        [Port("u_s", 2)],
        SIGNALS.class_code(
            "FluxTorqueController",
            start,
            output,
            "flux_torque_ctrl_update(&self, T_s, &fbk);\n",
            workspace=SAMPLING + "static ObserverOutputs fbk;\n",
            outputs=(2,),
        ),
        params,
        SIGNALS.sfunction("flux_torque_controller"),
        mask=mask,
    )


# Mask initialization of the FluxVectorController: FluxVectorControllerCfg and
# FluxVectorController.__init__
FVC_INIT = (
    "% Default of alpha_o (FluxVectorControllerCfg.__post_init__)\n"
    "alpha_o = cfg.alpha_o;\n"
    "if isempty(alpha_o)\n"
    "  if cfg.sensorless\n"
    "    alpha_o = 2*pi*50;\n"
    "  else\n"
    "    alpha_o = 2*pi*400;\n"
    "  end\n"
    "  if ~isempty(cfg.J)\n"
    "    alpha_o = alpha_o/3;\n"
    "  end\n"
    "end\n"
    "% Defaults of FluxVectorController.__init__\n"
    "alpha_psi = cfg.alpha_psi;\n"
    "if isempty(alpha_psi)\n"
    "  alpha_psi = cfg.alpha_tau;\n"
    "end\n"
    "alpha_i = cfg.alpha_i;\n"
    "if isempty(alpha_i)\n"
    "  alpha_i = cfg.alpha_tau;\n"
    "end\n"
)


def _flux_vector_controller() -> Subsystem:
    """Flux-vector controller (FluxVectorController)."""
    layout = [SIGNALS.index("ref", n) for n in ("psi_s", "tau_M", "u_s")]
    if layout != [0, 1, 2]:
        raise RuntimeError("The layout of ref does not match its multiplexer")
    mask = Mask(
        "FluxVectorController (motulator)",
        "Flux-vector controller (FluxVectorController). The configuration cfg is a "
        "struct with the fields of FluxVectorControllerCfg ([] = None, i.e., the "
        "default; the offline reference generation and k_f = [] only). The mask "
        "initialization resolves the defaults and passes the parameters to the blocks "
        "inside it, as FluxVectorController." + PAR_DESCRIPTION,
        [
            Param("par", PAR_PROMPT, "par"),
            Param(
                "cfg", "cfg: Configuration (FluxVectorControllerCfg) as a struct", "cfg"
            ),
        ],
        init=FVC_INIT,
    )
    return Subsystem(
        "FluxVectorController",
        [
            Port(ENABLE, 1),
            Port("tau_M_ref", 1),
            SIGNALS.port("meas"),
            Port("u_c_ab", 2),
            Port("u_c_zoh_ab", 2),
        ],
        [SIGNALS.port("fbk"), SIGNALS.port("ref")],
        [
            _speed_flux_observer(),
            SIGNALS.selector("meas", "u_dc"),
            SIGNALS.selector("fbk", "w_m"),
            _reference_generator(),
            _flux_torque_controller(),
            Mux("Mux ref", [Port("ref", 2), Port("u_s", 2)]),
        ],
        [
            (ENABLE, "SpeedFluxObserver:enable"),
            (ENABLE, "FluxTorqueController:enable"),
            ("meas", "SpeedFluxObserver:meas"),
            ("u_c_ab", "SpeedFluxObserver:u_s_ab"),
            ("u_c_zoh_ab", "SpeedFluxObserver:u_s_zoh_ab"),
            ("meas", "meas.u_dc:u"),
            ("SpeedFluxObserver:out", "fbk.w_m:u"),
            ("SpeedFluxObserver:out", "FluxTorqueController:fbk"),
            ("SpeedFluxObserver:out", "fbk"),
            ("tau_M_ref", "ReferenceGenerator:tau_M_ref"),
            ("meas.u_dc:y", "ReferenceGenerator:u_dc"),
            ("fbk.w_m:y", "ReferenceGenerator:w_m"),
            ("ReferenceGenerator:ref", "FluxTorqueController:ref"),
            ("ReferenceGenerator:ref", "Mux ref:ref"),
            ("FluxTorqueController:u_s", "Mux ref:u_s"),
            ("Mux ref:y", "ref"),
        ],
        columns=[
            ["tau_M_ref", "meas", "u_c_ab", "u_c_zoh_ab", ENABLE],
            ["SpeedFluxObserver"],
            ["meas.u_dc", "fbk.w_m"],
            ["ReferenceGenerator"],
            ["FluxTorqueController"],
            ["Mux ref"],
            ["ref", "fbk"],
        ],
        align={
            "tau_M_ref": "ReferenceGenerator:tau_M_ref",
            "meas.u_dc:y": "ReferenceGenerator:u_dc+1",
            "fbk.w_m:y": "ReferenceGenerator:w_m+1",
            "SpeedFluxObserver:meas": "fbk.w_m:y+1",
            "meas": "SpeedFluxObserver:meas",
            "FluxTorqueController:ref": "ReferenceGenerator:ref",
            "Mux ref:u_s": "FluxTorqueController:u_s",
            "ref": "Mux ref:y",
            "fbk": "SpeedFluxObserver:out",
        },
        lanes={"Mux ref:ref": "over:FluxTorqueController:0", "meas.u_dc:u": "early"},
        tags=[ENABLE],
        mask=mask,
    )


# %%
MONITORED = {
    "w_M_ref": "InputSignal(3, 0)",
    "w_M": "fbk.w_M",
    "tau_M_ref": "ref.tau_M",
    "tau_M": "fbk.tau_M",
    "psi_s_ref": "ref.psi_s",
    "psi_s": "cabs(fbk.psi_s)",
    "theta_m": "fbk.theta_m",
    "i_d": "creal(fbk.i_s)",
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
        SIGNALS, NAME, CTRL_INPUTS, measurement, _flux_vector_controller(), signals
    )


FVC_BLOCK = control_block(
    NAME,
    "Speed control of a synchronous machine drive with flux-vector control "
    "(VectorControlSystem with FluxVectorController and SpeedController).",
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
    control system (`sm.export_values`).
    """
    flux_map = values["psi_s_dq_fcn"]
    machine = ["n_p", "R_s"] + (["L_d", "L_q", "psi_f"] if flux_map is None else [])
    cfg = ["i_s_max", "alpha_tau", "alpha_psi", "alpha_i", "alpha_o", "k_o"]
    cfg += ["psi_s_min", "psi_s_max", "k_u", "k_mtpv", "J", "sensorless", "T_s"]
    return (
        init_header(variables, comment)
        + "% Machine model of the control system (SynchronousMachinePars"
        + (")\n" if flux_map is None else ", here with a flux map)\n")
        + "".join(assign(f"par.{n}", values[n]) for n in machine)
        + ("" if flux_map is None else f"par.psi_s_dq_fcn = {flux_map};\n")
        + "% Flux-vector control (FluxVectorControllerCfg)\n"
        + "".join(assign(f"cfg.{n}", values[n]) for n in cfg[:6])
        + "cfg.k_f = [];\n"
        + "".join(assign(f"cfg.{n}", values[n]) for n in cfg[6:])
        + speed_ctrl_init(values)
        + pwm_init(values)
    )
