"""
Native simulation of synchronous machine drives with flux-vector control.

The system is simulated by the library `c/native_sm.c`. Supported are the drive
models of `motulator_export.native._drive` with a synchronous machine, and
flux-vector control (`FluxVectorController`) in `VectorControlSystem` with a speed
controller, with the features of the C port (see `motulator_export.plecs._sm`) and the
duty-ratio error model of the PWM (without the minimum pulses, if compensated), the
gates enabled.

"""

from typing import Any, cast

from motulator.common.control._controllers import PIController
from motulator.drive.control._base import VectorControlSystem
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
from motulator_export.plecs._sm import check_supported_fvc, fvc_values
from motulator_export.plecs._sm_control import CFG_FIELDS, PAR_FIELDS

# Functions of the library setting the parameters of the control system, the
# arguments in the order of PAR_FIELDS and CFG_FIELDS (k_o as k_o0 and k_o1)
API: Signatures = {
    "set_control_machine": (None, [c_void_p] + [c_double] * 5),
    "control_flux_map": (c_void_p, [c_void_p]),
    "set_control_machine_flux_map": (None, [c_void_p, c_double, c_double]),
    "set_flux_vector_controller": (
        None,
        [c_void_p] + [c_double] * 12 + [c_int, c_double],
    ),
    "set_speed_controller": (None, [c_void_p] + [c_double] * 4),
    "speed_reference": (c_void_p, [c_void_p]),
    **PWM_API,
}


class SynchronousMachineDriveSystem(NativeSystem):
    """
    Synchronous machine drive with flux-vector control in the native library.

    Parameters
    ----------
    mdl : Drive
        Continuous-time system model.
    ctrl : VectorControlSystem
        Discrete-time control system.

    """

    def __init__(self, mdl: Any, ctrl: Any) -> None:
        self.check_supported(mdl, ctrl)
        mdl, ctrl = cast(Drive, mdl), cast(VectorControlSystem, ctrl)
        super().__init__("sm", {**DRIVE_API, **API})
        lib, h = self.lib, self.handle
        set_drive(self, mdl)

        # Control system
        v, flux_map = fvc_values(cast(FluxVectorController, ctrl.vector_ctrl))
        if flux_map is not None:
            self.set_gradnet(lib.control_flux_map(h), flux_map)
            lib.set_control_machine_flux_map(h, v["n_p"], v["R_s"])
        else:
            lib.set_control_machine(h, *c_args(v, PAR_FIELDS))
        k_o = v["k_o"] or [None, None]  # The observer gain k_o0 + k_o1*abs(w_m)
        lib.set_flux_vector_controller(h, *c_args(v | {"k_o": k_o}, CFG_FIELDS))
        speed_ctrl = cast(PIController, ctrl.speed_ctrl)
        lib.set_speed_controller(
            h, speed_ctrl.k_p, speed_ctrl.k_t, speed_ctrl.alpha_i, speed_ctrl.u_max
        )
        self.set_pwm(ctrl.pwm)

    def set_signals(self, mdl: Any, ctrl: Any) -> None:
        """Set the speed reference and the load torque."""
        mdl, ctrl = cast(Drive, mdl), cast(VectorControlSystem, ctrl)
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
    if type(ctrl) is not VectorControlSystem:
        raise NotImplementedError("Only VectorControlSystem supported")
    if type(ctrl.vector_ctrl) is not FluxVectorController:
        raise NotImplementedError("Only FluxVectorController supported")
    check_supported_fvc(ctrl.vector_ctrl)
    if not isinstance(ctrl.speed_ctrl, PIController):
        raise NotImplementedError("Speed-control mode with a speed controller required")
    check_supported_control(ctrl, d_err=True)
