"""
Observer-based V/Hz control of induction machine drives: each class is a block.

The control system mirrors the classes of motulator as a hierarchy of blocks, each
with a mask of the arguments of its class (see `_control`):

- VHzControlSystem (the masked subsystem of the control system), containing the
  blocks of its methods (`get_measurement` and `modulate`), the monitored signals,
  the `RateLimiter` of the speed reference, the `PWM` (with the MME overmodulation by
  default), and the `ObserverBasedVHzController`
- ObserverBasedVHzController(par, cfg), containing the `FluxObserver` (created by
  `create_vhz_observer`, the rotor speed being the rate-limited speed reference),
  the `ReferenceGenerator`, the `FluxTorqueController`, and the low-pass filter of
  the torque estimate (its state `tau_M_lpf`)

Pure open-loop V/Hz control is the special case ``L_M = inf`` of the machine model
(with ``R_s = R_R = L_sgm = 0`` and zero gains, see `plot_2kw_im_diode_vhz.py` of
motulator): the mask initialization of the ObserverBasedVHzController then sets the
unit observer gain, the weight h = 1 of the external speed error (which disables the
model-based correction), the initial stator flux estimate psi_s_nom, and k_u = inf,
as `ObserverBasedVHzController.__init__` does. The masks of the top-level classes
refer to the variables of the model initialization (`init_script`): the machine model
`par`, the configuration `cfg`, the slew rate `slew_rate` of the speed reference, and
the PWM `pwm`. The signal vectors are defined in `im_flux_vector.h` (the
measurements and the references) and `im_current_vector.h` (the feedback signals).

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
    get_measurement,
    init_header,
    mask_params,
    monitor,
    pwm_init,
    vhz_control_system,
)
from motulator_export.plecs._im_control import (
    PAR,
    PAR_DESCRIPTION,
    PAR_FIELDS,
    PAR_INIT,
    PAR_PROMPT,
    _par_code,
)
from motulator_export.plecs._netlist import CBlock, Mask, Mux, Param, Port, Subsystem

SIGNALS = Signals(
    ("common.c", "im_current_vector.c", "im_flux_vector.c"),
    ("im_flux_vector.h", "im_current_vector.h"),
    {
        "meas": ("IMVHzMeasurements", "im_vhz_measurements", "VHZ_MEAS"),
        "fbk": ("IMObserverOutputs", "im_observer_outputs"),
        "ref": ("IMVHzReferences", "im_vhz_references", "VHZ_REF"),
    },
    "sfun_ovhz",
)

# Name of the control-system block, its inputs, and the monitored signals (mask
# probes)
NAME = "Observer-based VHz control"  # No slash, the path separator of Simulink
CTRL_INPUTS = [Port(ENABLE, 1), Port("w_M_ref", 1), Port("i_s_abc", 3), Port("u_dc", 1)]
CTRL_OUTPUTS = {
    "Speed (w_M_ref, w_s)": ["w_M_ref", "w_s"],
    "Torque (tau_M_ref, tau_M)": ["tau_M_ref", "tau_M"],
    "Flux linkage (psi_s_ref, psi_s, psi_R)": ["psi_s_ref", "psi_s", "psi_R"],
    "Current (i_sd, i_sq)": ["i_sd", "i_sq"],
}
VHZ_PAR_DESCRIPTION = PAR_DESCRIPTION + " L_M = inf in pure open-loop V/Hz control."


# %%
# Blocks of the classes
def _flux_observer() -> CBlock:
    """Flux observer without speed estimation (FluxObserver)."""
    mask = Mask(
        "FluxObserver (motulator)",
        "Reduced-order flux observer in synchronous coordinates (FluxObserver), "
        "created by create_vhz_observer: the rotor speed w_M is the rate-limited "
        "speed reference, and the external speed error signal is zero, with the "
        "weight h (1 in pure open-loop V/Hz control, which disables the model-based "
        "correction). The initial stator flux estimate is psi_s0."
        + VHZ_PAR_DESCRIPTION,
        [
            Param("par", PAR_PROMPT, "par"),
            Param(
                "k_o",
                "k_o: Observer gain k_o(w_m), [] = the default of create_vhz_observer, "
                "1 = unity",
                "k_o",
            ),
            Param("h", "h: Weight of the external speed error signal", "h"),
            Param("psi_s0", "psi_s0: Initial stator flux estimate (Vs)", "psi_s0"),
            Param("T_s", "T_s: Sampling period (s)", "cfg.T_s"),
        ],
        init=PAR_INIT,
    )
    params = mask_params(mask, PAR)
    i = {p: k for k, p in enumerate(params)}
    k_o = i["k_o"]
    start = (
        checks(params, ["h", "psi_s0", "T_s"])
        + f"if (PDIM({k_o}) > 0 && (PDIM({k_o}) != 1 || P({k_o}, 0) != 1.0)) {{\n"
        '    SetErrorMessage("Only k_o = [] (the default) or 1 supported.");\n'
        "    return;\n"
        "}\n" + _par_code(params) + "im_flux_observer_init(&self, par, 1);\n"
        f"self.unit_gain = PDIM({k_o}) > 0;\n"
        f"self.psi_s = P({i['psi_s0']}, 0);\n"
        f"T_s = P({i['T_s']}, 0);\n"
        f"h = P({i['h']}, 0);\n"
    )
    output = (
        SIGNALS.read(1, "meas") + f"double complex u_s_ab = {complex_in(2)};\n"
        "/* The rotor speed is the speed reference, without the external speed\n"
        " * error signal */\n"
        "im_flux_observer_compute_output(&self, u_s_ab, meas.i_c_ab,\n"
        "                                InputSignal(3, 0), 0.0, h, &out);\n"
        + SIGNALS.write(0, "fbk", "out")
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


def _torque_filter() -> CBlock:
    """Low-pass filter of the torque estimate (tau_M_lpf of the controller)."""
    params = ["cfg.alpha_f", "cfg.T_s"]
    start = (
        checks(params, params) + "/* Low-pass-filtered torque estimate */\n"
        "self = 0.0;\n"
        "alpha_f = P(0, 0);\n"
        "T_s = P(1, 0);\n"
    )
    return CBlock(
        "tau_M_lpf",
        [Port(ENABLE, 1), Port("tau_M", 1)],
        [Port("tau_M_lpf", 1)],
        SIGNALS.class_code(
            "double",
            start,
            "OutputSignal(0, 0) = self;\n",
            "self += T_s * alpha_f * (InputSignal(1, 0) - self);\n",
            SAMPLING + "static double alpha_f; /* Low-pass-filter bandwidth */\n",
        ),
        params,
        SIGNALS.sfunction("torque_filter"),
        feedthrough=[1, 0],
        sample_time="cfg.T_s",
    )


def _reference_generator() -> CBlock:
    """Flux and torque reference generator (ReferenceGenerator)."""
    mask = Mask(
        "ReferenceGenerator (motulator)",
        "Reference generator with field weakening and torque limits "
        "(ReferenceGenerator). The inputs are the DC-bus voltage, the torque "
        "reference, and the feedback signals fbk (the synchronous angular frequency "
        "fbk.w_s and the rotor flux estimate fbk.psi_R). The output ref is [psi_s "
        "tau_M], the flux reference and the limited torque reference."
        + VHZ_PAR_DESCRIPTION,
        [
            Param("par", PAR_PROMPT, "par"),
            Param(
                "psi_s_nom",
                "psi_s_nom: Nominal stator flux linkage (Vs)",
                "cfg.psi_s_nom",
            ),
            Param("i_s_max", "i_s_max: Maximum stator current (A)", "cfg.i_s_max"),
            Param("tau_M_max", "tau_M_max: Maximum torque reference (Nm)", "inf"),
            Param("k_u", "k_u: Voltage utilization factor", "k_u"),
            Param("k_b", "k_b: Breakdown torque margin", "cfg.k_b"),
        ],
        init=PAR_INIT,
    )
    params = mask_params(mask, PAR)
    i = {p: k for k, p in enumerate(params)}
    start = (
        checks(params, ["psi_s_nom", "i_s_max", "tau_M_max", "k_u", "k_b"])
        + _par_code(params)
        + f"im_flux_reference_gen_init(&self, par, P({i['psi_s_nom']}, 0), "
        f"P({i['i_s_max']}, 0),\n"
        f"                           P({i['tau_M_max']}, 0), P({i['k_u']}, 0), "
        f"P({i['k_b']}, 0));\n"
    )
    output = (
        SIGNALS.read(2, "fbk")
        + "/* Flux and limited torque references (ref.psi_s, ref.tau_M) */\n"
        "double psi_s_ref, tau_M_ref;\n"
        "im_reference_gen_flux_and_torque(&self, InputSignal(1, 0), fbk.w_s,\n"
        "                                 cabs(fbk.psi_R), InputSignal(0, 0),\n"
        "                                 &psi_s_ref, &tau_M_ref);\n"
        "OutputSignal(0, 0) = psi_s_ref;\n"
        "OutputSignal(0, 1) = tau_M_ref;\n"
    )
    return CBlock(
        "ReferenceGenerator",
        [Port("u_dc", 1), Port("tau_M_ref", 1), SIGNALS.port("fbk")],
        [Port("ref", 2)],
        SIGNALS.class_code("IMReferenceGenerator", start, output, disabled=None),
        params,
        SIGNALS.sfunction("reference_generator"),
        sample_time=None,
        mask=mask,
    )


def _flux_torque_controller() -> CBlock:
    """Flux and torque controller (FluxTorqueController)."""
    mask = Mask(
        "FluxTorqueController (motulator)",
        "Flux and torque controller (FluxTorqueController) without the integral "
        "action. The input ref is [psi_s tau_M], the flux and torque references, and "
        "the output is the stator voltage reference." + VHZ_PAR_DESCRIPTION,
        [
            Param("par", PAR_PROMPT, "par"),
            Param(
                "alpha_psi",
                "alpha_psi: Flux-control bandwidth (rad/s)",
                "cfg.alpha_psi",
            ),
            Param(
                "alpha_tau",
                "alpha_tau: Torque-control bandwidth (rad/s)",
                "cfg.alpha_tau",
            ),
        ],
        init=PAR_INIT,
    )
    params = mask_params(mask, PAR)
    i = {p: k for k, p in enumerate(params)}
    start = (
        checks(params, ["alpha_psi", "alpha_tau"])
        + _par_code(params)
        + f"im_flux_torque_ctrl_init(&self, par, P({i['alpha_psi']}, 0), "
        f"P({i['alpha_tau']}, 0));\n"
    )
    output = (
        SIGNALS.read(1, "fbk") + "double complex u_s_ref =\n"
        "    im_flux_torque_ctrl_compute_output(&self, InputSignal(0, 0),\n"
        "                                       InputSignal(0, 1), &fbk);\n"
        + complex_out(0, "u_s_ref")
    )
    return CBlock(
        "FluxTorqueController",
        [Port("ref", 2), SIGNALS.port("fbk")],
        [Port("u_s", 2)],
        SIGNALS.class_code("IMFluxTorqueController", start, output, disabled=None),
        params,
        SIGNALS.sfunction("flux_torque_controller"),
        sample_time=None,
        mask=mask,
    )


# Mask initialization of the ObserverBasedVHzController:
# ObserverBasedVHzController.__init__ and create_vhz_observer
VHZ_INIT = (
    "% Pure open-loop V/Hz control with L_M = inf (ObserverBasedVHzController):\n"
    "% the unit observer gain with the weight h = 1, which disables the model-based\n"
    "% correction (create_vhz_observer), the initial flux estimate, and k_u = inf\n"
    "if isinf(par.L_M)\n"
    "  k_o = 1;\n"
    "  h = 1;\n"
    "  psi_s0 = cfg.psi_s_nom;\n"
    "  k_u = inf;\n"
    "else\n"
    "  k_o = [];\n"
    "  h = 0;\n"
    "  psi_s0 = 0;\n"
    "  k_u = cfg.k_u;\n"
    "end\n"
)


def _vhz_controller() -> Subsystem:
    """Observer-based V/Hz controller (ObserverBasedVHzController)."""
    layout = [SIGNALS.index("ref", n) for n in ("psi_s", "tau_M", "u_s")]
    if layout != [0, 1, 2]:
        raise RuntimeError("The layout of ref does not match its multiplexer")
    mask = Mask(
        "ObserverBasedVHzController (motulator)",
        "Observer-based V/Hz controller (ObserverBasedVHzController), or pure "
        "open-loop V/Hz control if par.L_M = inf. The configuration cfg is a struct "
        "with the fields of ObserverBasedVHzControllerCfg ([] = None, i.e., the "
        "default; the default observer gain k_o only). The mask initialization "
        "passes the parameters to the blocks inside it, as "
        "ObserverBasedVHzController. The block tau_M_lpf is the low-pass filter of "
        "the torque estimate, whose output is the torque reference." + PAR_DESCRIPTION,
        [
            Param("par", PAR_PROMPT, "par"),
            Param(
                "cfg",
                "cfg: Configuration (ObserverBasedVHzControllerCfg) as a struct",
                "cfg",
            ),
        ],
        init=VHZ_INIT,
    )
    return Subsystem(
        "ObserverBasedVHzController",
        [Port(ENABLE, 1), Port("w_M", 1), SIGNALS.port("meas"), Port("u_c_ab", 2)],
        [SIGNALS.port("fbk"), SIGNALS.port("ref")],
        [
            _flux_observer(),
            SIGNALS.selector("meas", "u_dc"),
            SIGNALS.selector("fbk", "tau_M"),
            _torque_filter(),
            _reference_generator(),
            _flux_torque_controller(),
            Mux("Mux ref", [Port("ref", 2), Port("u_s", 2)]),
        ],
        [
            (ENABLE, "FluxObserver:enable"),
            (ENABLE, "tau_M_lpf:enable"),
            ("meas", "FluxObserver:meas"),
            ("u_c_ab", "FluxObserver:u_s_ab"),
            ("w_M", "FluxObserver:w_M"),
            ("meas", "meas.u_dc:u"),
            ("FluxObserver:out", "fbk.tau_M:u"),
            ("FluxObserver:out", "ReferenceGenerator:fbk"),
            ("FluxObserver:out", "FluxTorqueController:fbk"),
            ("FluxObserver:out", "fbk"),
            ("fbk.tau_M:y", "tau_M_lpf:tau_M"),
            ("meas.u_dc:y", "ReferenceGenerator:u_dc"),
            ("tau_M_lpf:tau_M_lpf", "ReferenceGenerator:tau_M_ref"),
            ("ReferenceGenerator:ref", "FluxTorqueController:ref"),
            ("ReferenceGenerator:ref", "Mux ref:ref"),
            ("FluxTorqueController:u_s", "Mux ref:u_s"),
            ("Mux ref:y", "ref"),
        ],
        columns=[
            ["meas", "u_c_ab", "w_M", ENABLE],
            ["FluxObserver"],
            ["meas.u_dc", "fbk.tau_M"],
            ["tau_M_lpf"],
            ["ReferenceGenerator"],
            ["FluxTorqueController"],
            ["Mux ref"],
            ["ref", "fbk"],
        ],
        align={
            "meas.u_dc:y": "ReferenceGenerator:u_dc-1",
            "tau_M_lpf:tau_M_lpf": "ReferenceGenerator:tau_M_ref",
            "fbk.tau_M:y": "tau_M_lpf:tau_M",
            "FluxObserver:out": "ReferenceGenerator:fbk+1",
            "meas": "FluxObserver:meas",
            "u_c_ab": "FluxObserver:u_s_ab",
            "FluxTorqueController:ref": "ReferenceGenerator:ref",
            "Mux ref:u_s": "FluxTorqueController:u_s",
            "ref": "Mux ref:y",
            "fbk": "FluxObserver:out",
        },
        lanes={"Mux ref:ref": "over:FluxTorqueController:0", "meas.u_dc:u": "early"},
        tags=[ENABLE],
        mask=mask,
    )


# %%
MONITORED = {
    "w_M_ref": "InputSignal(3, 0)",
    "w_s": "fbk.w_s",
    "tau_M_ref": "ref.tau_M",
    "tau_M": "fbk.tau_M",
    "psi_s_ref": "ref.psi_s",
    "psi_s": "cabs(fbk.psi_s)",
    "psi_R": "cabs(fbk.psi_R)",
    "i_sd": "creal(rot * fbk.i_s)",
    "i_sq": "cimag(rot * fbk.i_s)",
}


def control_system() -> Subsystem:
    """Contents of the control-system block (VHzControlSystem)."""
    measurement = get_measurement(
        SIGNALS,
        CTRL_INPUTS[2:],
        [
            ("i_c_ab", "abc2complex(i_s_abc)"),
            ("u_dc", "fmax(InputSignal(1, 0), U_DC_MIN)"),
        ],
        abc_input("i_s_abc", 0),
    )
    signals = monitor(
        SIGNALS,
        CTRL_OUTPUTS,
        MONITORED,
        [Port("w_M_ref", 1)],
        "/* The current in estimated rotor flux coordinates */\n"
        "double complex rot = cexp(-I * carg(fbk.psi_R));\n",
    )
    return vhz_control_system(
        SIGNALS, NAME, CTRL_INPUTS, measurement, _vhz_controller(), signals
    )


VHZ_BLOCK = control_block(
    NAME,
    "Observer-based V/Hz control of an induction machine drive, or pure open-loop "
    "V/Hz control if L_M = inf (VHzControlSystem with ObserverBasedVHzController). "
    "The speed reference is in mechanical rad/s.",
    CTRL_INPUTS,
    CTRL_OUTPUTS,
    control_system,
    cls="VHzControlSystem",
)


# %%
def init_script(
    variables: list[tuple[str, Any]], values: dict[str, Any], comment: str
) -> str:
    """
    Model initialization: the variables of the system model, and the variables of
    the control system (par, cfg, slew_rate, and pwm) from the parameter values of
    the control system (`im.export_vhz_values`).
    """
    cfg = ["psi_s_nom", "i_s_max", "alpha_psi", "alpha_tau", "alpha_f", "k_u", "k_b"]
    open_loop = values["L_M"] == float("inf")
    return (
        init_header(variables, comment)
        + "% Machine model of the control system (InductionMachineInvGammaPars"
        + (", L_M = inf: pure open-loop V/Hz control)\n" if open_loop else ")\n")
        + "".join(assign(f"par.{n}", values[n]) for n in PAR_FIELDS)
        + "% Observer-based V/Hz control (ObserverBasedVHzControllerCfg)\n"
        + "".join(assign(f"cfg.{n}", values[n]) for n in [*cfg, "T_s"])
        + "% Slew rate of the speed reference (VHzControlSystem), mechanical rad/s^2\n"
        + assign("slew_rate", values["slew_rate"])
        + pwm_init(values)
    )
