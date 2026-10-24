"""
Flux-vector control of induction machine drives: each class is a block.

The control system mirrors the classes of motulator as a hierarchy of blocks, each
with a mask of the arguments of its class (see `_control`):

- VectorControlSystem (the masked subsystem of the control system), containing the
  blocks of its methods (`get_measurement` and `modulate`), the monitored signals,
  the `PWM`, the `SpeedController`, and the `FluxVectorController`
- FluxVectorController(par, cfg), containing the `ReferenceGenerator` and the
  `FluxTorqueController` (with the integral action) of observer-based V/Hz control,
  and the `SpeedFluxObserver` of current-vector control

The mask initialization of the FluxVectorController resolves the defaults of
`FluxVectorController`. The mask of the VectorControlSystem has its arguments from
the model initialization (`init_script`): the vector controller `vector_ctrl` (the
machine model `par` and the configuration `cfg`), the speed controller
`speed_ctrl`, and the PWM `pwm`. The measurements and the feedback signals are those
of current-vector control (`im_current_vector.h`), and the references those of V/Hz
control (`im_flux_vector.h`).

"""

from typing import Any

from motulator_export.plecs._control import (
    PAR_CFG,
    Signals,
    abc_input,
    assign,
    control_block,
    get_measurement,
    init_header,
    inner_ctrl_init,
    monitor,
    pwm_init,
    speed_ctrl_init,
    vector_control_system,
)
from motulator_export.plecs._im_control import CTRL_INPUTS as CVC_INPUTS
from motulator_export.plecs._im_control import (
    PAR_DESCRIPTION,
    PAR_FIELDS,
    PAR_PROMPT,
    _speed_flux_observer,
)
from motulator_export.plecs._im_vhz_control import (
    _flux_torque_controller,
    _reference_generator,
)
from motulator_export.plecs._netlist import Mask, Mux, Param, Port, Subsystem

# The measurements and the feedback signals of current-vector control, with the
# references of V/Hz control
SIGNALS = Signals(
    ("common.c", "im_current_vector.c", "im_flux_vector.c"),
    ("im_flux_vector.h", "im_current_vector.h"),
    {
        "meas": ("IMMeasurements", "im_measurements"),
        "fbk": ("IMObserverOutputs", "im_observer_outputs"),
        "ref": ("IMVHzReferences", "im_vhz_references", "VHZ_REF"),
    },
    "sfun_imfvc",
)

# Name of the control-system block, its inputs, and the monitored signals (mask
# probes)
NAME = "Flux-vector control (IM)"
CTRL_INPUTS = CVC_INPUTS
CTRL_OUTPUTS = {
    "Speed (w_M_ref, w_M)": ["w_M_ref", "w_M"],
    "Torque (tau_M_ref, tau_M)": ["tau_M_ref", "tau_M"],
    "Flux linkage (psi_s_ref, psi_s, psi_R)": ["psi_s_ref", "psi_s", "psi_R"],
    "Current (i_sd, i_sq)": ["i_sd", "i_sq"],
}


# %%
# Mask initialization of the FluxVectorController: FluxVectorController.__init__
FVC_INIT = (
    "% Defaults of alpha_psi and alpha_i (FluxVectorController.__init__)\n"
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
        "default; the default observer gain k_o only). The mask initialization "
        "resolves the defaults and passes the parameters to the blocks inside it, as "
        "FluxVectorController, and the SpeedFluxObserver resolves the defaults of "
        "create_speed_flux_observer." + PAR_DESCRIPTION,
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
            _speed_flux_observer(SIGNALS),
            SIGNALS.selector("meas", "u_dc"),
            _reference_generator(SIGNALS, fvc=True),
            _flux_torque_controller(SIGNALS, fvc=True),
            Mux("Mux ref", [Port("ref", 2), Port("u_s", 2)]),
        ],
        [
            ("meas", "SpeedFluxObserver:meas"),
            ("u_c_ab", "SpeedFluxObserver:u_s_ab"),
            ("u_c_zoh_ab", "SpeedFluxObserver:u_s_zoh_ab"),
            ("meas", "meas.u_dc:u"),
            ("SpeedFluxObserver:out", "ReferenceGenerator:fbk"),
            ("SpeedFluxObserver:out", "FluxTorqueController:fbk"),
            ("SpeedFluxObserver:out", "fbk"),
            ("tau_M_ref", "ReferenceGenerator:tau_M_ref"),
            ("meas.u_dc:y", "ReferenceGenerator:u_dc"),
            ("ReferenceGenerator:ref", "FluxTorqueController:ref"),
            ("ReferenceGenerator:ref", "Mux ref:ref"),
            ("FluxTorqueController:u_s", "Mux ref:u_s"),
            ("Mux ref:y", "ref"),
        ],
        columns=[
            ["tau_M_ref", "meas", "u_c_ab", "u_c_zoh_ab"],
            ["SpeedFluxObserver"],
            ["meas.u_dc"],
            ["ReferenceGenerator"],
            ["FluxTorqueController"],
            ["Mux ref"],
            ["ref", "fbk"],
        ],
        align={
            "meas.u_dc:y": "ReferenceGenerator:u_dc",
            "SpeedFluxObserver:out": "ReferenceGenerator:fbk+1",
            "meas": "SpeedFluxObserver:meas",
            "FluxTorqueController:ref": "ReferenceGenerator:ref",
            "Mux ref:u_s": "FluxTorqueController:u_s",
            "ref": "Mux ref:y",
            "fbk": "SpeedFluxObserver:out",
        },
        lanes={"Mux ref:ref": "over:FluxTorqueController:0", "meas.u_dc:u": "early"},
        # The torque reference by tags, since the DC-bus voltage enters the
        # ReferenceGenerator above it
        tags=["tau_M_ref"],
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
    "psi_R": "cabs(fbk.psi_R)",
    "i_sd": "creal(rot * fbk.i_s)",
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
        "/* The current in estimated rotor flux coordinates */\n"
        "double complex rot = cexp(-I * carg(fbk.psi_R));\n",
    )
    return vector_control_system(
        SIGNALS, NAME, CTRL_INPUTS, measurement, _flux_vector_controller(), signals
    )


FVC_BLOCK = control_block(
    NAME,
    "Speed control of an induction machine drive with flux-vector control "
    "(VectorControlSystem with FluxVectorController and SpeedController).",
    CTRL_INPUTS,
    CTRL_OUTPUTS,
    control_system,
    "FluxVectorController",
)


# %%
# Fields of the configuration of flux-vector control (FluxVectorControllerCfg,
# without k_o), in the order of the model initialization and of the native library
CFG_FIELDS = ["psi_s_nom", "i_s_max", "alpha_tau", "alpha_psi", "alpha_i", "alpha_o"]
CFG_FIELDS += ["tau_M_max", "k_u", "k_b", "J", "sensorless", "T_s"]


def init_script(
    variables: list[tuple[str, Any]], values: dict[str, Any], comment: str
) -> str:
    """
    Model initialization: the variables of the system model, and the structs of the
    control system (par, cfg, vector_ctrl, speed_ctrl, and pwm) from the parameter
    values of the control system (`_im.export_fvc_values`).
    """
    return (
        init_header(variables, comment)
        + "% Machine model of the control system (InductionMachineInvGammaPars)\n"
        + "".join(assign(f"par.{n}", values[n]) for n in PAR_FIELDS)
        + "% Flux-vector control (FluxVectorControllerCfg)\n"
        + "".join(assign(f"cfg.{n}", values[n]) for n in CFG_FIELDS)
        + inner_ctrl_init(
            "vector_ctrl", "FluxVectorController", "Flux-vector controller", PAR_CFG
        )
        + speed_ctrl_init(values)
        + pwm_init(values)
    )
