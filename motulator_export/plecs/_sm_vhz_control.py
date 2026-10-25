"""
Observer-based V/Hz control of synchronous machine drives: each class is a block.

The control system mirrors the classes of motulator as a hierarchy of blocks, each
with a mask of the arguments of its class (see `_control`):

- VHzControlSystem (the masked subsystem of the control system), containing the
  blocks of its methods (`get_measurement` and `modulate`), the monitored signals,
  the `RateLimiter` of the speed reference, the `PWM` (with the MME overmodulation by
  default), and the `ObserverBasedVHzController`
- ObserverBasedVHzController(par, cfg), containing the `FluxObserver` (created by
  `create_vhz_observer`, the rotor speed being the rate-limited speed reference),
  the `ReferenceGenerator`, the `FluxTorqueController` (without the integral
  action), and the low-pass filter of the torque estimate (its state `tau_M_lpf`)

The mask initialization of the ObserverBasedVHzController resolves the defaults of
`ObserverBasedVHzController.__init__` and `create_vhz_observer`. The mask of the
VHzControlSystem has its arguments from the model initialization (`init_script`):
the V/Hz controller `vhz_ctrl` (the machine model `par` and the configuration
`cfg`), the slew rate `slew_rate` of the speed reference, and the PWM `pwm`. The
signal vectors are those of flux-vector control (`sm_flux_vector.h`), and the
ReferenceGenerator and the FluxTorqueController are its blocks (`_sm_control`).

"""

from typing import Any

from motulator_export.plecs._common import ENABLE
from motulator_export.plecs._control import (
    PAR_CFG,
    SAMPLING,
    Signals,
    abc_input,
    assign,
    checks,
    complex_in,
    compute_standby_output,
    control_block,
    get_measurement,
    init_header,
    inner_ctrl_init,
    mask_params,
    monitor,
    pwm_init,
    torque_filter,
    vhz_control_system,
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
from motulator_export.plecs._sm_control import (
    M_K_O,
    M_PAR,
    M_PAR_CHECKS,
    M_PAR_FUNCTION,
    PAR,
    PAR_DESCRIPTION,
    PAR_FIELDS,
    PAR_INIT,
    PAR_PROMPT,
    _flux_torque_controller,
    _k_o_code,
    _par_code,
    _reference_generator,
)
from motulator_export.plecs._sm_control import SIGNALS as FVC_SIGNALS

# The signal vectors of flux-vector control, with the S-functions of this method
SIGNALS = Signals(
    FVC_SIGNALS.sources, FVC_SIGNALS.headers, FVC_SIGNALS.vectors, "sfun_smvhz"
)

# Name of the control-system block, its inputs, and the monitored signals (mask
# probes)
NAME = "Observer-based VHz control (SM)"  # No slash, the path separator of Simulink
CTRL_INPUTS = [Port(ENABLE, 1), Port("w_M_ref", 1), Port("i_s_abc", 3), Port("u_dc", 1)]
CTRL_OUTPUTS = {
    "Speed (w_M_ref, w_c)": ["w_M_ref", "w_c"],
    "Torque (tau_M_ref, tau_M)": ["tau_M_ref", "tau_M"],
    "Flux linkage (psi_s_ref, psi_s)": ["psi_s_ref", "psi_s"],
    "Angle and current (theta_m, i_d, i_q)": ["theta_m", "i_d", "i_q"],
}


# %%
# Blocks of the classes
def _flux_observer() -> CBlock:
    """Flux observer without speed estimation (FluxObserver)."""
    mask = Mask(
        "FluxObserver (motulator)",
        "Flux observer in estimated rotor coordinates (FluxObserver), created by "
        "create_vhz_observer: the rotor speed w_M is the rate-limited speed "
        "reference, and the PM-flux estimation gain is zero. The observer uses the "
        "voltage u_s_zoh_ab of the ongoing sampling period, and the output fbk.u_s "
        "is the voltage u_s_ab averaged over the previous and ongoing periods "
        "(DriveControlSystem.get_feedback). In standby, the voltages are unknown "
        "(NaN), and the flux estimate follows the current model alone, with the "
        "open-circuit voltage of the model as the voltage." + PAR_DESCRIPTION,
        [
            Param("par", PAR_PROMPT, "par"),
            Param("k_theta", "k_theta: Rotor angle estimation gain (rad/s)", "k_theta"),
            Param(
                "k_o",
                "k_o: Observer gain [k0 k1], k_o(w_m) = k0 + k1*abs(w_m), k0 = NaN: "
                "sigma0 of the machine model",
                "k_o",
            ),
            Param("T_s", "T_s: Sampling period (s)", "cfg.T_s"),
        ],
        init=PAR_INIT,
    )
    params = mask_params(mask, PAR)
    i = {p: k for k, p in enumerate(params)}
    k_o = i["k_o"]
    start = (
        checks(params, ["k_theta", "T_s"]) + f"if (PDIM({k_o}) != 2) {{\n"
        '    SetErrorMessage("k_o must be [k0 k1].");\n'
        "    return;\n"
        "}\n"
        + _par_code(i["par_n_p"])
        + _k_o_code(k_o)
        + f"flux_observer_init(&self, par, P({i['k_theta']}, 0), k_o);\n"
        f"T_s = P({i['T_s']}, 0);\n"
    )
    output = (
        SIGNALS.read(0, "meas") + f"double complex u_s_ab = {complex_in(1)};\n"
        f"double complex u_s_zoh_ab = {complex_in(2)};\n"
        "/* The rotor speed is the speed reference */\n"
        "flux_observer_compute_output(&self, u_s_zoh_ab, meas.i_c_ab,\n"
        "                             InputSignal(3, 0), 0.0, 0.0, &out);\n"
        "/* Voltage for the control laws (DriveControlSystem.get_feedback), or the\n"
        " * open-circuit voltage of the observer if unknown */\n"
        "out.u_s = isnan(creal(u_s_ab)) ? out.u_s_zoh\n"
        "                               : cexp(-I * out.theta_c) * u_s_ab;\n"
        + SIGNALS.write(0, "fbk", "out")
    )
    mcode = MCode(
        start=M_PAR + M_K_O + "self = flux_observer_init(par, p.k_theta, k_o);\n",
        output="% The rotor speed is the speed reference\n"
        "out = flux_observer_compute_output(self, u_s_zoh_ab, meas.i_c_ab, w_M, ...\n"
        "    0, 0);\n"
        "% Voltage for the control laws (DriveControlSystem.get_feedback), or the\n"
        "% open-circuit voltage of the observer if unknown\n"
        "if isnan(real(u_s_ab))\n"
        "    out.u_s = out.u_s_zoh;\n"
        "else\n"
        "    out.u_s = exp(-1i*out.theta_c)*u_s_ab;\n"
        "end\n",
        update="self = flux_observer_update(self, p.T_s, out);\n",
        inputs={
            "meas": SIGNALS.vector("meas"),
            "u_s_ab": COMPLEX,
            "u_s_zoh_ab": COMPLEX,
        },
        outputs={"out": SIGNALS.vector("fbk")},
        functions=M_PAR_FUNCTION,
        checks=(
            *required("k_theta", "T_s"),
            ("numel(p.k_o) ~= 2", "k_o must be [k0 k1]."),
            *M_PAR_CHECKS,
        ),
    )
    return CBlock(
        "FluxObserver",
        [
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
            workspace=SAMPLING + "static ObserverOutputs out;\n",
        ),
        params,
        SIGNALS.sfunction("flux_observer"),
        mask=mask,
        mcode=mcode,
    )


# Mask initialization of the ObserverBasedVHzController:
# ObserverBasedVHzController.__init__ and create_vhz_observer
VHZ_INIT = (
    "% First-order angle estimation, the pole matched to the sampling period\n"
    "% (matched_pole)\n"
    "k_theta = -expm1(-cfg.alpha_o*cfg.T_s)/cfg.T_s;\n"
    "% Default observer gain of create_vhz_observer, k_o(w_m) = k0 + k1*abs(w_m)\n"
    "% (k0 = NaN: sigma0 of the machine model, see the FluxObserver)\n"
    "k_o = cfg.k_o;\n"
    "if isempty(k_o)\n"
    "  k_o = [NaN 0.2];\n"
    "end\n"
    "% Flux and torque controller without the integral action\n"
    "alpha_psi = cfg.alpha_psi;\n"
    "alpha_i = 0;\n"
)


def _vhz_controller() -> Subsystem:
    """Observer-based V/Hz controller (ObserverBasedVHzController)."""
    layout = [SIGNALS.index("ref", n) for n in ("psi_s", "tau_M", "u_s")]
    if layout != [0, 1, 2]:
        raise RuntimeError("The layout of ref does not match its multiplexer")
    mask = Mask(
        "ObserverBasedVHzController (motulator)",
        "Observer-based V/Hz controller (ObserverBasedVHzController). The "
        "configuration cfg is a struct with the fields of "
        "ObserverBasedVHzControllerCfg ([] = None, i.e., the default). The mask "
        "initialization resolves the defaults and passes the parameters to the "
        "blocks inside it, as ObserverBasedVHzController. The block tau_M_lpf is the "
        "low-pass filter of the torque estimate, whose output is the torque "
        "reference." + PAR_DESCRIPTION,
        [
            Param("par", PAR_PROMPT, "vhz_ctrl.par"),
            Param(
                "cfg",
                "cfg: Configuration (ObserverBasedVHzControllerCfg) as a struct",
                "vhz_ctrl.cfg",
            ),
        ],
        init=VHZ_INIT,
    )
    return Subsystem(
        "ObserverBasedVHzController",
        [
            Port(ENABLE, 1),
            Port("w_M", 1),
            SIGNALS.port("meas"),
            Port("u_c_ab", 2),
            Port("u_c_zoh_ab", 2),
        ],
        [SIGNALS.port("fbk"), SIGNALS.port("ref")],
        [
            _flux_observer(),
            # The measurements to the selector of the DC-bus voltage by a tag, since
            # the inputs of the ReferenceGenerator from the observer are on both
            # sides of the DC-bus voltage
            Tag("Goto meas", "meas", SIGNALS.width("meas"), goto=True),
            Tag("From meas", "meas", SIGNALS.width("meas"), goto=False),
            SIGNALS.selector("meas", "u_dc"),
            SIGNALS.selector("fbk", "tau_M"),
            SIGNALS.selector("fbk", "w_m"),
            torque_filter(SIGNALS),
            _reference_generator(SIGNALS),
            # Standby: the flux and torque controller tracks the estimated flux and
            # torque
            compute_standby_output(
                SIGNALS, [Port("ref", 2)], ["cabs(fbk.psi_s)", "fbk.tau_M"]
            ),
            _flux_torque_controller(SIGNALS),
            Mux("Mux ref", [Port("ref", 2), Port("u_s", 2)]),
        ],
        [
            (ENABLE, "compute_standby_output:enable"),
            ("meas", "FluxObserver:meas"),
            ("meas", "Goto meas"),
            ("u_c_ab", "FluxObserver:u_s_ab"),
            ("u_c_zoh_ab", "FluxObserver:u_s_zoh_ab"),
            ("w_M", "FluxObserver:w_M"),
            ("From meas", "meas.u_dc:u"),
            ("FluxObserver:out", "fbk.tau_M:u"),
            ("FluxObserver:out", "fbk.w_m:u"),
            ("FluxObserver:out", "compute_standby_output:fbk"),
            ("FluxObserver:out", "FluxTorqueController:fbk"),
            ("FluxObserver:out", "fbk"),
            ("fbk.tau_M:y", "tau_M_lpf:tau_M"),
            ("tau_M_lpf:tau_M_lpf", "ReferenceGenerator:tau_M_ref"),
            ("meas.u_dc:y", "ReferenceGenerator:u_dc"),
            ("fbk.w_m:y", "ReferenceGenerator:w_m"),
            ("ReferenceGenerator:ref", "compute_standby_output:ref_in"),
            ("compute_standby_output:ref", "FluxTorqueController:ref"),
            ("compute_standby_output:ref", "Mux ref:ref"),
            ("FluxTorqueController:u_s", "Mux ref:u_s"),
            ("Mux ref:y", "ref"),
        ],
        columns=[
            ["meas", "u_c_ab", "u_c_zoh_ab", "w_M", ENABLE],
            ["Goto meas", "FluxObserver"],
            ["fbk.tau_M", "From meas", "fbk.w_m"],
            ["tau_M_lpf", "meas.u_dc"],
            ["ReferenceGenerator"],
            ["compute_standby_output"],
            ["FluxTorqueController"],
            ["Mux ref"],
            ["ref", "fbk"],
        ],
        align={
            "tau_M_lpf:tau_M_lpf": "ReferenceGenerator:tau_M_ref",
            "fbk.tau_M:y": "tau_M_lpf:tau_M",
            "meas.u_dc:y": "ReferenceGenerator:u_dc+1",
            "From meas": "meas.u_dc:u",
            "fbk.w_m:y": "ReferenceGenerator:w_m+2",
            "FluxObserver:out": "fbk.w_m:y+1",
            "meas": "FluxObserver:meas",
            "u_c_ab": "FluxObserver:u_s_ab",
            "compute_standby_output:ref_in": "ReferenceGenerator:ref",
            "FluxTorqueController:ref": "compute_standby_output:ref",
            "Mux ref:u_s": "FluxTorqueController:u_s",
            "ref": "Mux ref:y",
            "fbk": "FluxObserver:out",
        },
        lanes={"Mux ref:ref": "over:FluxTorqueController:0"},
        tags=[ENABLE],
        mask=mask,
    )


# %%
MONITORED = {
    "w_M_ref": "InputSignal(2, 0)",
    "w_c": "fbk.w_c",
    "tau_M_ref": "ref.tau_M",
    "tau_M": "fbk.tau_M",
    "psi_s_ref": "ref.psi_s",
    "psi_s": "cabs(fbk.psi_s)",
    "theta_m": "fbk.theta_m",
    "i_d": "creal(fbk.i_s)",
    "i_q": "cimag(fbk.i_s)",
}


def control_system() -> Subsystem:
    """Contents of the control-system block (VHzControlSystem)."""
    measurement = get_measurement(
        SIGNALS,
        CTRL_INPUTS[2:],
        [
            ("i_c_ab", "abc2complex(i_s_abc)"),
            ("u_dc", "fmax(InputSignal(1, 0), U_DC_MIN)"),
            ("theta_M", "0.0"),
        ],
        abc_input("i_s_abc", 0),
    )
    signals = monitor(SIGNALS, CTRL_OUTPUTS, MONITORED, [Port("w_M_ref", 1)])
    return vhz_control_system(
        SIGNALS, NAME, CTRL_INPUTS, measurement, _vhz_controller(), signals
    )


VHZ_BLOCK = control_block(
    NAME,
    "Observer-based V/Hz control of a synchronous machine drive (VHzControlSystem "
    "with ObserverBasedVHzController). The speed reference is in mechanical rad/s.",
    CTRL_INPUTS,
    CTRL_OUTPUTS,
    control_system,
    "ObserverBasedVHzController",
    cls="VHzControlSystem",
)


# %%
# Fields of the configuration of observer-based V/Hz control
# (ObserverBasedVHzControllerCfg), in the order of the model initialization and of
# the native library
CFG_FIELDS = ["i_s_max", "alpha_psi", "alpha_tau", "alpha_f", "alpha_o", "k_o"]
CFG_FIELDS += ["k_u", "k_mtpv", "psi_s_min", "psi_s_max", "T_s"]


def init_script(
    variables: list[tuple[str, Any]], values: dict[str, Any], comment: str
) -> str:
    """
    Model initialization: the variables of the system model, and the structs of the
    control system (par, cfg, vhz_ctrl, slew_rate, and pwm) from the parameter
    values of the control system (`_sm.export_vhz_values`).
    """
    flux_map = values["psi_s_dq_fcn"]
    machine = PAR_FIELDS if flux_map is None else PAR_FIELDS[:2]
    return (
        init_header(variables, comment)
        + "% Machine model of the control system (SynchronousMachinePars"
        + (")\n" if flux_map is None else ", here with a flux map)\n")
        + "".join(assign(f"par.{n}", values[n]) for n in machine)
        + ("" if flux_map is None else f"par.psi_s_dq_fcn = {flux_map};\n")
        + "% Observer-based V/Hz control (ObserverBasedVHzControllerCfg)\n"
        + "".join(assign(f"cfg.{n}", values[n]) for n in CFG_FIELDS)
        + inner_ctrl_init(
            "vhz_ctrl", "ObserverBasedVHzController", "V/Hz controller", PAR_CFG
        )
        + "% Slew rate of the speed reference (VHzControlSystem), mechanical rad/s^2\n"
        + assign("slew_rate", values["slew_rate"])
        + pwm_init(values)
    )
