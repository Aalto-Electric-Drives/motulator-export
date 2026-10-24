"""
Native simulation of synchronous machine drives with flux-vector control,
current-vector control, or observer-based V/Hz control.

The system is simulated by the library `c/native_sm.c`. Supported are the drive
models of `motulator_export.native._drive` with a synchronous machine, and

- flux-vector control (`FluxVectorController`) or current-vector control
  (`CurrentVectorController`) in `VectorControlSystem` with a speed controller, or
- observer-based V/Hz control (`ObserverBasedVHzController`) in `VHzControlSystem`,

with the features of the C port (see `motulator_export.plecs._sm`) and the
duty-ratio error model of the PWM, also with the gates blocked
(`ControlSystem.set_enable`, a constant or a `StepSignal`).

"""

from typing import Any, cast

from motulator.common.control._controllers import PIController
from motulator.drive.control._base import VectorControlSystem, VHzControlSystem
from motulator.drive.control._sm_current_vector import CurrentVectorController
from motulator.drive.control._sm_flux_vector import FluxVectorController
from motulator.drive.model import Drive, SynchronousMachine

from motulator_export.native._build import Signatures
from motulator_export.native._drive import (
    DRIVE_API,
    check_supported_drive,
    set_drive,
    set_mechanics_signals,
)
from motulator_export.native._system import (
    PWM_API,
    NativeSystem,
    c_args,
    c_double,
    c_int,
    c_void_p,
    check_supported_control,
)
from motulator_export.plecs._sm import (
    check_supported_vector_ctrl,
    check_supported_vhz_ctrl,
    cvc_values,
    export_vhz_values,
    fvc_values,
)
from motulator_export.plecs._sm_control import CFG_FIELDS, PAR_FIELDS
from motulator_export.plecs._sm_cvc_control import CFG_FIELDS as CVC_CFG_FIELDS
from motulator_export.plecs._sm_vhz_control import CFG_FIELDS as VHZ_CFG_FIELDS

# Functions of the library setting the parameters of the control system, the
# arguments in the order of PAR_FIELDS and CFG_FIELDS (k_o as k_o0 and k_o1, and the
# slew rate of V/Hz control)
API: Signatures = {
    "set_control_machine": (None, [c_void_p] + [c_double] * 5),
    "control_flux_map": (c_void_p, [c_void_p]),
    "set_control_machine_flux_map": (None, [c_void_p, c_double, c_double]),
    "set_flux_vector_controller": (
        None,
        [c_void_p] + [c_double] * 12 + [c_int, c_double],
    ),
    "set_current_vector_controller": (
        None,
        [c_void_p] + [c_double] * 12 + [c_int, c_double, c_int],
    ),
    "set_speed_controller": (None, [c_void_p] + [c_double] * 4),
    "set_vhz_controller": (None, [c_void_p] + [c_double] * 13),
    "speed_reference": (c_void_p, [c_void_p]),
    **PWM_API,
}


class SynchronousMachineDriveSystem(NativeSystem):
    """
    Synchronous machine drive with flux-vector control, current-vector control, or
    observer-based V/Hz control in the native library.

    Parameters
    ----------
    mdl : Drive
        Continuous-time system model.
    ctrl : VectorControlSystem | VHzControlSystem
        Discrete-time control system.

    """

    def __init__(self, mdl: Any, ctrl: Any) -> None:
        self.check_supported(mdl, ctrl)
        super().__init__("sm", {**DRIVE_API, **API})
        lib, h = self.lib, self.handle
        set_drive(self, cast(Drive, mdl))

        # Control system
        if isinstance(ctrl, VHzControlSystem):
            v, flux_map = export_vhz_values(ctrl)
        elif isinstance(ctrl.vector_ctrl, CurrentVectorController):
            v, flux_map = cvc_values(ctrl.vector_ctrl)
        else:
            ctrl = cast(VectorControlSystem, ctrl)
            v, flux_map = fvc_values(cast(FluxVectorController, ctrl.vector_ctrl))
        if flux_map is not None:
            self.set_gradnet(lib.control_flux_map(h), flux_map)
            lib.set_control_machine_flux_map(h, v["n_p"], v["R_s"])
        else:
            lib.set_control_machine(h, *c_args(v, PAR_FIELDS))
        v["k_o"] = v["k_o"] or [None, None]  # The observer gain k_o0 + k_o1*abs(w_m)
        if isinstance(ctrl, VHzControlSystem):
            lib.set_vhz_controller(h, *c_args(v, [*VHZ_CFG_FIELDS, "slew_rate"]))
        else:
            if isinstance(ctrl.vector_ctrl, CurrentVectorController):
                lib.set_current_vector_controller(h, *c_args(v, CVC_CFG_FIELDS))
            else:
                lib.set_flux_vector_controller(h, *c_args(v, CFG_FIELDS))
            speed_ctrl = cast(PIController, ctrl.speed_ctrl)
            lib.set_speed_controller(
                h, speed_ctrl.k_p, speed_ctrl.k_t, speed_ctrl.alpha_i, speed_ctrl.u_max
            )
        self.set_pwm(ctrl.pwm)

    def set_signals(self, mdl: Any, ctrl: Any) -> None:
        """Set the speed reference and the load torque."""
        mdl = cast(Drive, mdl)
        if ctrl.ext_ref.w_M is None:
            raise ValueError("The speed reference must be set")
        self.set_signal(self.lib.speed_reference(self.handle), ctrl.ext_ref.w_M)
        set_mechanics_signals(self, mdl)

    @staticmethod
    def supports(mdl: Any, ctrl: Any) -> bool:
        """Whether the system is a synchronous machine drive."""
        return isinstance(mdl, Drive) and isinstance(mdl.machine, SynchronousMachine)

    @staticmethod
    def check_supported(mdl: Any, ctrl: Any) -> None:
        """Raise an error if the drive system is not supported."""
        _check_supported_control(ctrl)
        if type(mdl.machine) is not SynchronousMachine:
            raise NotImplementedError("Only SynchronousMachine supported")
        check_supported_drive(mdl)


def _check_supported_control(ctrl: Any) -> None:
    """Raise an error if the control system is not supported."""
    if type(ctrl) is VHzControlSystem:
        check_supported_vhz_ctrl(ctrl.vhz_ctrl)
    elif type(ctrl) is VectorControlSystem:
        check_supported_vector_ctrl(ctrl.vector_ctrl)
        if not isinstance(ctrl.speed_ctrl, PIController):
            raise NotImplementedError(
                "Speed-control mode with a speed controller required"
            )
    else:
        raise NotImplementedError(
            "Only VectorControlSystem and VHzControlSystem supported"
        )
    check_supported_control(ctrl, d_err=True)
