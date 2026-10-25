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
`FluxVectorController.__init__`, and that of the SpeedFluxObserver resolves the
defaults of `create_speed_flux_observer` and computes the gains of
`SpeedFluxObserver.__init__`. The mask of the VectorControlSystem has its
arguments, structs of the model initialization (`init_script`): the vector
controller `vector_ctrl` (the machine model `par` and the configuration `cfg`), the
speed controller `speed_ctrl`, and the PWM `pwm`. The signal vectors are defined in
`sm_flux_vector.h`.

"""

from functools import partial
from typing import Any

from motulator_export.plecs._common import ENABLE, GRADNET_FIELDS
from motulator_export.plecs._control import (
    PAR_CFG,
    SAMPLING,
    T_S,
    Signals,
    abc_input,
    assign,
    checks,
    complex_in,
    complex_out,
    control_block,
    drive_monitor,
    get_measurement,
    init_header,
    inner_ctrl_init,
    mask_params,
    pwm_init,
    speed_ctrl_init,
    speed_observer,
    torque_inputs,
    torque_outputs,
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
    Tag,
    required,
)

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
TORQUE_NAME = "Flux-vector control (torque)"
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


# The machine model par in the MATLAB code from the parameters PAR_PARAMS
M_PAR_FUNCTION = """\
function par = machine_pars(p)
%MACHINE_PARS Machine model from the parameters par_* and gn_* of the block:
%   SaturatedSynchronousMachinePars with a GradNet flux map, or
%   SynchronousMachinePars with constant inductances.
if isnan(p.gn_in_dim)
    par = synchronous_machine_pars(p.par_n_p, p.par_R_s, p.par_L_d, p.par_L_q, ...
        p.par_psi_f, gradnet_placeholder());
else
    flux_map = gradnet(p.gn_in_dim, p.gn_mu_dim, p.gn_W, p.gn_b, p.gn_mu_log, ...
        p.gn_bias, p.gn_activation, p.gn_beta_log, p.gn_p, p.gn_in_base, ...
        p.gn_out_base);
    par = saturated_synchronous_machine_pars(p.par_n_p, p.par_R_s, flux_map);
end
end
"""
M_PAR_CHECKS = (
    *required("par_n_p", "par_R_s"),
    (
        "isnan(p.gn_in_dim) && any(isnan([p.par_L_d(:); p.par_L_q(:); "
        "p.par_psi_f(:)]))",
        "L_d, L_q, and psi_f needed without a flux map.",
    ),
)
M_PAR = (
    "% Machine model: SaturatedSynchronousMachinePars with a GradNet flux map, or\n"
    "% SynchronousMachinePars with constant inductances\n"
    "par = machine_pars(p);\n"
)


# %%
# Blocks of the classes
def _k_o_code(i: int) -> str:
    """
    C code of the observer gain k_o = [k0 k1] from the parameter i, k0 = NaN being
    sigma0 of the machine model par.
    """
    return (
        f"double k_o[2] = {{P({i}, 0), P({i}, 1)}};\n"
        "if (isnan(k_o[0])) {\n"
        "    /* Poles at zero speed at s = 0 and s = -2*sigma0 */\n"
        "    IncrIndMat L_s0 = incr_ind_mat(&par, 0.0);\n"
        "    k_o[0] = 0.25 * par.R_s * (1.0 / L_s0.L_dd + 1.0 / L_s0.L_qq);\n"
        "}\n"
    )


# The observer gain k_o in the MATLAB code (see `_k_o_code`)
M_K_O = (
    "k_o = p.k_o;\n"
    "if isnan(k_o(1))\n"
    "    % Poles at zero speed at s = 0 and s = -2*sigma0\n"
    "    L_s0 = incr_ind_mat(par, complex(0));\n"
    "    k_o(1) = 0.25*par.R_s*(1/L_s0.L_dd + 1/L_s0.L_qq);\n"
    "end\n"
)


def _flux_observer(sig: Signals = SIGNALS) -> CBlock:
    """Flux observer (FluxObserver)."""
    mask = Mask(
        "FluxObserver (motulator)",
        "Flux observer in estimated rotor coordinates (FluxObserver). The observer "
        "uses the voltage u_s_zoh_ab of the ongoing sampling period, and the output "
        "fbk.u_s is the voltage u_s_ab averaged over the previous and ongoing periods "
        "(DriveControlSystem.get_feedback). In standby, the voltages are unknown "
        "(NaN), and the flux estimate follows the current model alone, with the "
        "open-circuit voltage of the model as the voltage. When the voltage is "
        "known again after the release of the gates, the estimates are initialized "
        "from the current response to the zero-voltage period (the flying start of "
        "SpeedFluxObserver), and the output init = [flag w_M] initializes the "
        "SpeedObserver. In the sensored mode (h = 1), the position error signal is "
        "computed from the measured rotor angle (compute_feedback)." + PAR_DESCRIPTION,
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
        "}\n"
        + _par_code(i["par_n_p"])
        + _k_o_code(k_o)
        + f"flux_observer_init(&self, par, P({i['k_theta']}, 0), k_o);\n"
        f"T_s = P({i['T_s']}, 0);\n"
        f"h = P({i['h']}, 0);\n"
        "i_s_ab_prev = NAN;\n"
    )
    output = (
        sig.read(0, "meas") + f"double complex u_s_ab = {complex_in(1)};\n"
        f"double complex u_s_zoh_ab = {complex_in(2)};\n"
        "double w_M = InputSignal(3, 0);\n"
        "/* Flying start when the voltage is known again (SpeedFluxObserver), the\n"
        " * speed estimate initialized by the output init */\n"
        "int caught = flux_observer_flying_start(&self, &i_s_ab_prev, u_s_zoh_ab,\n"
        "                                        meas.i_c_ab, h, &w_M);\n"
        "OutputSignal(0, 0) = caught;\n"
        "OutputSignal(0, 1) = w_M;\n"
        "if (h == 0.0) {\n"
        "    flux_observer_compute_output(&self, u_s_zoh_ab, meas.i_c_ab, w_M, 0.0,\n"
        "                                 0.0, &out);\n"
        "} else {\n"
        "    /* Position error from the measured rotor angle (compute_feedback) */\n"
        "    double n_p = self.par.n_p;\n"
        "    double eps = wrap(n_p * meas.theta_M - self.theta_m) / n_p;\n"
        "    flux_observer_compute_output(&self, u_s_zoh_ab, meas.i_c_ab, w_M, eps,\n"
        "                                 h, &out);\n"
        "}\n"
        "out.caught = caught;\n"
        "/* Voltage for the control laws (DriveControlSystem.get_feedback), or the\n"
        " * open-circuit voltage of the observer if unknown */\n"
        "out.u_s = isnan(creal(u_s_ab)) ? out.u_s_zoh\n"
        "                               : cexp(-I * out.theta_c) * u_s_ab;\n"
        + sig.write(1, "fbk", "out")
    )
    mcode = MCode(
        start=M_PAR + M_K_O + "self = flux_observer_init(par, p.k_theta, k_o);\n"
        "% Current at the start of the latest sampling period with an unknown voltage\n"
        "self.i_s_ab_prev = complex(NaN);\n",
        output="% Flying start when the voltage is known again (SpeedFluxObserver), "
        "the\n"
        "% speed estimate initialized by the output init\n"
        "[self, i_s_ab_prev, caught, w_M] = flux_observer_flying_start(self, ...\n"
        "    self.i_s_ab_prev, u_s_zoh_ab, meas.i_c_ab, p.h, w_M);\n"
        "self.i_s_ab_prev = i_s_ab_prev;\n"
        "init = [caught; w_M];\n"
        "if p.h == 0\n"
        "    out = flux_observer_compute_output(self, u_s_zoh_ab, meas.i_c_ab, w_M, "
        "0, 0);\n"
        "else\n"
        "    % Position error from the measured rotor angle (compute_feedback)\n"
        "    n_p = self.par.n_p;\n"
        "    eps = wrap(n_p*meas.theta_M - self.theta_m)/n_p;\n"
        "    out = flux_observer_compute_output(self, u_s_zoh_ab, meas.i_c_ab, w_M, "
        "eps, p.h);\n"
        "end\n"
        "out.caught = caught;\n"
        "% Voltage for the control laws (DriveControlSystem.get_feedback), or the\n"
        "% open-circuit voltage of the observer if unknown\n"
        "if isnan(real(u_s_ab))\n"
        "    out.u_s = out.u_s_zoh;\n"
        "else\n"
        "    out.u_s = exp(-1i*out.theta_c)*u_s_ab;\n"
        "end\n",
        update="self = flux_observer_update(self, p.T_s, out);\n",
        inputs={"meas": sig.vector("meas"), "u_s_ab": COMPLEX, "u_s_zoh_ab": COMPLEX},
        outputs={"out": sig.vector("fbk")},
        functions=M_PAR_FUNCTION,
        checks=(
            *required("k_theta", "h", "T_s"),
            ("numel(p.k_o) ~= 2", "k_o must be [k0 k1]."),
            ("~isnan(p.k_f(1)) && p.k_f(1) ~= 0", "Only k_f = [] (zero) supported."),
            *M_PAR_CHECKS,
        ),
    )
    return CBlock(
        "FluxObserver",
        [sig.port("meas"), Port("u_s_ab", 2), Port("u_s_zoh_ab", 2), Port("w_M", 1)],
        [Port("init", 2), Port("out", sig.width("fbk"))],
        sig.class_code(
            "FluxObserver",
            start,
            output,
            "flux_observer_update(&self, T_s, &out);\n",
            workspace=SAMPLING + "static double h;\nstatic ObserverOutputs out;\n"
            "/* Current at the start of the latest sampling period with an unknown\n"
            " * voltage, NAN if the voltage is known */\n"
            "static double complex i_s_ab_prev;\n",
        ),
        params,
        sig.sfunction("flux_observer"),
        mask=mask,
        mcode=mcode,
    )


# Mask initialization of the SpeedFluxObserver: create_speed_flux_observer and
# SpeedFluxObserver.__init__
SFO_INIT = (
    "% Default speed estimation poles of create_speed_flux_observer\n"
    "% (default_alpha_o). With J, the triple pole at alpha_o/sqrt(3) keeps the\n"
    "% speed gain about the same.\n"
    "alpha_o_obs = alpha_o;\n"
    "if isempty(alpha_o_obs)\n"
    "  if sensorless\n"
    "    alpha_o_obs = 2*pi*50;\n"
    "  else\n"
    "    alpha_o_obs = 2*pi*400;\n"
    "  end\n"
    "  if ~isempty(J)\n"
    "    alpha_o_obs = alpha_o_obs/sqrt(3);\n"
    "  end\n"
    "end\n"
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
    "% Gains for critically damped dynamics (SpeedFluxObserver.__init__), the\n"
    "% pole matched to the sampling period (matched_pole)\n"
    "alpha_o_obs = -expm1(-alpha_o_obs*T_s)/T_s;\n"
    "if isempty(J)\n"
    "  k_theta = 2*alpha_o_obs;\n"
    "  k_w = alpha_o_obs^2;\n"
    "  k_tau = 0;\n"
    "else\n"
    "  k_theta = 3*alpha_o_obs;\n"
    "  k_w = 3*alpha_o_obs^2;\n"
    "  k_tau = J*alpha_o_obs^3;\n"
    "end\n"
    "% Weight of the measured rotor angle in the FluxObserver (position_error)\n"
    "h = 1 - sensorless;\n"
)


def _speed_flux_observer(sig: Signals = SIGNALS) -> Subsystem:
    """Speed and flux observer (SpeedFluxObserver)."""
    mask = Mask(
        "SpeedFluxObserver (motulator)",
        "Flux observer with speed estimation, created by create_speed_flux_observer. "
        "The mask initialization resolves the defaults of create_speed_flux_observer "
        "and computes the gains of the SpeedObserver and the FluxObserver inside it, "
        "as in SpeedFluxObserver." + PAR_DESCRIPTION,
        [
            Param("par", PAR_PROMPT, "par"),
            Param(
                "alpha_o",
                "alpha_o: Speed estimation poles (rad/s), [] = default",
                "cfg.alpha_o",
            ),
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
    fbk = sig.width("fbk")
    return Subsystem(
        "SpeedFluxObserver",
        [sig.port("meas"), Port("u_s_ab", 2), Port("u_s_zoh_ab", 2)],
        [Port("out", fbk)],
        [
            _flux_observer(sig),
            Tag("Goto init", "init", 2, goto=True),
            sig.selector("fbk", "eps"),
            Tag("From init", "init", 2, goto=False),
            sig.selector("fbk", "tau_M"),
            speed_observer(sig, init=True),
        ],
        [
            ("meas", "FluxObserver:meas"),
            ("u_s_ab", "FluxObserver:u_s_ab"),
            ("u_s_zoh_ab", "FluxObserver:u_s_zoh_ab"),
            ("SpeedObserver:w_M", "FluxObserver:w_M"),
            ("FluxObserver:out", "out"),
            ("FluxObserver:out", "fbk.eps:u"),
            ("FluxObserver:out", "fbk.tau_M:u"),
            ("FluxObserver:init", "Goto init"),
            ("From init", "SpeedObserver:init"),
            ("fbk.eps:y", "SpeedObserver:eps"),
            ("fbk.tau_M:y", "SpeedObserver:tau_M"),
        ],
        columns=[
            ["meas", "u_s_ab", "u_s_zoh_ab"],
            ["FluxObserver"],
            ["Goto init", "fbk.eps", "From init", "fbk.tau_M"],
            ["SpeedObserver"],
            ["out"],
        ],
        align={
            "meas": "FluxObserver:meas",
            "out": "FluxObserver:out",
            "Goto init": "FluxObserver:init",
            "fbk.eps:y": "FluxObserver:w_M+1",
            "SpeedObserver:eps": "fbk.eps:y",
            "From init": "SpeedObserver:init",
        },
        lanes={"FluxObserver:w_M": "below:0"},
        mask=mask,
    )


# The lookup tables of the loci of the ReferenceGenerator: computed by the mask
# initialization in Simulink (the function sm_reference_luts.m of the library), or
# by the C-Script block at its start in PLECS
LUTS_INIT = (
    "% Lookup tables of the loci, computed by the block at its start if empty\n"
    "luts = [];\n"
)
LUTS_SIMULINK_INIT = (
    "% Lookup tables of the loci by sm_reference_luts.m of the library\n"
    "luts = sm_reference_luts(par, i_s_max);\n"
)


def _luts_code(i: int) -> str:
    """C code reading the lookup tables of the loci from the parameter i, if given."""
    return (
        "/* Lookup tables of the loci: given (sm_reference_luts.m in Simulink), or\n"
        " * computed at the start if empty (PLECS) */\n"
        "ReferenceLUTs luts_given;\n"
        "const ReferenceLUTs *luts = NULL;\n"
        f"if (PDIM({i}) > 0) {{\n"
        f"    if (PDIM({i}) != 6 * NUM_LOCUS) {{\n"
        '        SetErrorMessage("luts must be a 16-by-6 matrix (sm_reference_luts.m) '
        'or empty.");\n'
        "        return;\n"
        "    }\n"
        "    LUT *tables[3] = {&luts_given.psi_s_mtpa, &luts_given.tau_M_cl,\n"
        "                      &luts_given.tau_M_mtpv};\n"
        "    for (int t = 0; t < 3; t++) {\n"
        "        tables[t]->n = NUM_LOCUS;\n"
        "        for (int k = 0; k < NUM_LOCUS; k++) {\n"
        f"            tables[t]->x[k] = P({i}, 2 * t * NUM_LOCUS + k);\n"
        f"            tables[t]->y[k] = P({i}, (2 * t + 1) * NUM_LOCUS + k);\n"
        "        }\n"
        "    }\n"
        "    luts = &luts_given;\n"
        "}\n"
    )


def _reference_gen_start(params: list[str]) -> str:
    """
    C code of the start of a ReferenceGenerator block with the parameters `params`
    (the machine model, i_s_max, psi_s_min, psi_s_max, k_u, k_mtpv, and luts).
    """
    i = {p: k for k, p in enumerate(params)}
    return (
        checks(params, ["i_s_max", "psi_s_max", "k_u", "k_mtpv"])
        + _par_code(i["par_n_p"])
        + _luts_code(i["luts"])
        + f"reference_gen_init(&self, &par, P({i['i_s_max']}, 0), "
        f"PARAM({i['psi_s_min']}),\n"
        f"                   P({i['psi_s_max']}, 0), P({i['k_u']}, 0), "
        f"P({i['k_mtpv']}, 0), luts);\n"
        "/* Errors of ReferenceGenerator of motulator */\n"
        "const char *error = reference_gen_error(&self, &par);\n"
        "if (error != NULL) {\n"
        "    SetErrorMessage(error);\n"
        "    return;\n"
        "}\n"
    )


# The start of a ReferenceGenerator block in the MATLAB code (see
# `_reference_gen_start`), and the checks of its parameters
M_REFERENCE_GEN_START = (
    M_PAR + "% Lookup tables of the loci from the mask initialization "
    "(sm_reference_luts.m)\n"
    "self = reference_gen_init(par, p.psi_s_min, p.psi_s_max, p.k_u, "
    "p.k_mtpv, p.luts);\n"
    "% Error of ReferenceGenerator of motulator, also with a flux map (the\n"
    "% check of the mask needs psi_f)\n"
    "if par.psi_f == 0 && self.psi_s_min <= 0\n"
    "    error('psi_s_min must be positive for machines without PMs.');\n"
    "end\n"
)
M_REFERENCE_GEN_CHECKS = (
    *required("i_s_max", "psi_s_max", "k_u", "k_mtpv"),
    *M_PAR_CHECKS,
    (
        "~isequal(size(p.luts), [16, 6])",
        "luts must be a 16-by-6 matrix (sm_reference_luts.m).",
    ),
    # Error of ReferenceGenerator of motulator (psi_s_min NaN being psi_f), checked
    # here with constant inductances
    (
        "p.par_psi_f == 0 && ~(p.psi_s_min > 0)",
        "psi_s_min must be positive for machines without PMs.",
    ),
)


def _reference_generator(sig: Signals = SIGNALS) -> CBlock:
    """Flux and torque reference generator (ReferenceGenerator)."""
    mask = Mask(
        "ReferenceGenerator (motulator)",
        "Flux and torque reference generator (ReferenceGenerator) with the lookup "
        "tables of the MTPA, MTPV, and current-limit loci, computed by the function "
        "sm_reference_luts.m of the library in the mask initialization (Simulink) "
        "or by the block at the start (PLECS). The output ref is [psi_s tau_M], the "
        "flux reference and the limited torque reference." + PAR_DESCRIPTION,
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
        init=PAR_INIT + LUTS_INIT,
        simulink_init=LUTS_SIMULINK_INIT,
    )
    params = [*mask_params(mask, PAR), "luts"]
    start = _reference_gen_start(params)
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
        sig.class_code("ReferenceGenerator", start, output),
        params,
        sig.sfunction("reference_generator"),
        sample_time=None,
        mask=mask,
        mcode=MCode(
            start=M_REFERENCE_GEN_START,
            output="% Flux and limited torque references (ref.psi_s, ref.tau_M)\n"
            "[psi_s_ref, tau_M_lim] = reference_gen_compute_flux_and_torque_refs( ...\n"
            "    self, tau_M_ref, w_m, u_dc);\n"
            "ref = [psi_s_ref; tau_M_lim];\n",
            functions=M_PAR_FUNCTION,
            checks=M_REFERENCE_GEN_CHECKS,
        ),
    )


def _flux_torque_controller(sig: Signals = SIGNALS) -> CBlock:
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
        sig.read(1, "fbk", declare=False) + "double complex u_s_ref =\n"
        "    flux_torque_ctrl_compute_output(&self, InputSignal(0, 0), "
        "InputSignal(0, 1), &fbk);\n" + complex_out(0, "u_s_ref")
    )
    return CBlock(
        "FluxTorqueController",
        [Port("ref", 2), sig.port("fbk")],
        [Port("u_s", 2)],
        sig.class_code(
            "FluxTorqueController",
            start,
            output,
            "flux_torque_ctrl_update(&self, T_s, &fbk);\n",
            workspace=SAMPLING + "static ObserverOutputs fbk;\n",
        ),
        params,
        sig.sfunction("flux_torque_controller"),
        mask=mask,
        mcode=MCode(
            start=M_PAR + "self = flux_torque_ctrl_init(par, p.alpha_psi, "
            "p.alpha_tau, p.alpha_i);\n",
            output="[self, u_s] = flux_torque_ctrl_compute_output(self, ref(1), "
            "ref(2), fbk);\n",
            update="self = flux_torque_ctrl_update(self, p.T_s, fbk);\n",
            inputs={"fbk": sig.vector("fbk")},
            outputs={"u_s": COMPLEX},
            functions=M_PAR_FUNCTION,
            checks=(
                *required("alpha_psi", "alpha_tau", "alpha_i", "T_s"),
                *M_PAR_CHECKS,
            ),
        ),
    )


# Mask initialization of the FluxVectorController: FluxVectorController.__init__
FVC_INIT = (
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
            Param("par", PAR_PROMPT, "vector_ctrl.par"),
            Param(
                "cfg",
                "cfg: Configuration (FluxVectorControllerCfg) as a struct",
                "vector_ctrl.cfg",
            ),
        ],
        init=FVC_INIT,
    )
    return Subsystem(
        "FluxVectorController",
        [
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
            ["tau_M_ref", "meas", "u_c_ab", "u_c_zoh_ab"],
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
    "theta_m": "fbk.theta_m",
    "i_d": "creal(fbk.i_s)",
    "i_q": "cimag(fbk.i_s)",
}


def control_system(torque: bool = False) -> Subsystem:
    """
    Contents of the control-system block (VectorControlSystem), also in the
    torque-control mode without the speed controller (`torque`).
    """
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
    signals = drive_monitor(SIGNALS, CTRL_OUTPUTS, MONITORED, torque)
    return vector_control_system(
        SIGNALS,
        TORQUE_NAME if torque else NAME,
        torque_inputs(CTRL_INPUTS) if torque else CTRL_INPUTS,
        measurement,
        _flux_vector_controller(),
        signals,
        torque,
    )


FVC_BLOCK = control_block(
    NAME,
    "Speed control of a synchronous machine drive with flux-vector control "
    "(VectorControlSystem with FluxVectorController and SpeedController).",
    CTRL_INPUTS,
    CTRL_OUTPUTS,
    control_system,
    "FluxVectorController",
)
FVC_TORQUE_BLOCK = control_block(
    TORQUE_NAME,
    "Torque control of a synchronous machine drive with flux-vector control "
    "(VectorControlSystem with FluxVectorController, without the speed controller).",
    torque_inputs(CTRL_INPUTS),
    torque_outputs(CTRL_OUTPUTS),
    partial(control_system, True),
    "FluxVectorController",
    torque=True,
)


# %%
# Fields of the machine model (SynchronousMachinePars) and of the configuration
# (FluxVectorControllerCfg, without k_f) of the control system, in the order of the
# model initialization and of the native library
PAR_FIELDS = ["n_p", "R_s", "L_d", "L_q", "psi_f"]
CFG_FIELDS = ["i_s_max", "alpha_tau", "alpha_psi", "alpha_i", "alpha_o", "k_o"]
CFG_FIELDS += ["psi_s_min", "psi_s_max", "k_u", "k_mtpv", "J", "sensorless", "T_s"]


def init_script(
    variables: list[tuple[str, Any]], values: dict[str, Any], comment: str
) -> str:
    """
    Model initialization: the variables of the system model, and the structs of the
    control system (par, cfg, vector_ctrl, speed_ctrl, and pwm) from the parameter
    values of the control system (`_sm.export_values`).
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
        + "% Flux-vector control (FluxVectorControllerCfg)\n"
        + "".join(assign(f"cfg.{n}", values[n]) for n in CFG_FIELDS[:k_f])
        + "cfg.k_f = [];\n"
        + "".join(assign(f"cfg.{n}", values[n]) for n in CFG_FIELDS[k_f:])
        + inner_ctrl_init(
            "vector_ctrl", "FluxVectorController", "Flux-vector controller", PAR_CFG
        )
        + speed_ctrl_init(values)
        + pwm_init(values)
    )
