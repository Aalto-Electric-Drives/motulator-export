"""
Native simulation of induction machine drives with current-vector control or
observer-based V/Hz control.

The system is simulated by the library `c/native_im.c`. Supported are the drive
models of `motulator_export.native._drive` with an induction machine, and

- current-vector control (`CurrentVectorController`) in `VectorControlSystem` with a
  speed controller, or
- observer-based V/Hz control (`ObserverBasedVHzController`) in `VHzControlSystem`,
  including open-loop V/Hz control (`L_M = inf`),

with the features of the C port (see `motulator_export.plecs._im`) and the duty-ratio
error model of the PWM (without the minimum pulses, if compensated), the gates
enabled.

"""

from typing import Any, cast

from motulator.common.control._controllers import PIController
from motulator.drive.control._base import VectorControlSystem, VHzControlSystem
from motulator.drive.control._im_current_vector import CurrentVectorController
from motulator.drive.model import Drive, InductionMachine

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
from motulator_export.plecs._im import (
    check_supported_cvc,
    check_supported_vhz_ctrl,
    cvc_values,
    export_vhz_values,
)
from motulator_export.plecs._im_control import CFG_FIELDS, PAR_FIELDS
from motulator_export.plecs._im_vhz_control import CFG_FIELDS as VHZ_CFG_FIELDS

# Functions of the library setting the parameters of the control system, the
# arguments in the order of PAR_FIELDS and CFG_FIELDS (and the slew rate of V/Hz
# control)
API: Signatures = {
    "set_control_machine": (None, [c_void_p] + [c_double] * 5),
    "set_current_vector_controller": (
        None,
        [c_void_p] + [c_double] * 9 + [c_int, c_double],
    ),
    "set_speed_controller": (None, [c_void_p] + [c_double] * 4),
    "set_vhz_controller": (None, [c_void_p] + [c_double] * 9),
    "speed_reference": (c_void_p, [c_void_p]),
    **PWM_API,
}


class InductionMachineDriveSystem(NativeSystem):
    """
    Induction machine drive with current-vector control or observer-based V/Hz
    control in the native library.

    Parameters
    ----------
    mdl : Drive
        Continuous-time system model.
    ctrl : VectorControlSystem | VHzControlSystem
        Discrete-time control system.

    """

    def __init__(self, mdl: Any, ctrl: Any) -> None:
        self.check_supported(mdl, ctrl)
        super().__init__("im", {**DRIVE_API, **API})
        lib, h = self.lib, self.handle
        set_drive(self, cast(Drive, mdl))

        # Control system
        if isinstance(ctrl, VHzControlSystem):
            v = export_vhz_values(ctrl)
            lib.set_control_machine(h, *c_args(v, PAR_FIELDS))
            lib.set_vhz_controller(h, *c_args(v, [*VHZ_CFG_FIELDS, "slew_rate"]))
        else:
            ctrl = cast(VectorControlSystem, ctrl)
            v = cvc_values(cast(CurrentVectorController, ctrl.vector_ctrl))
            lib.set_control_machine(h, *c_args(v, PAR_FIELDS))
            lib.set_current_vector_controller(h, *c_args(v, CFG_FIELDS))
            speed_ctrl = cast(PIController, ctrl.speed_ctrl)
            lib.set_speed_controller(
                h, speed_ctrl.k_p, speed_ctrl.k_t, speed_ctrl.alpha_i, speed_ctrl.u_max
            )
        self.set_pwm(ctrl.pwm)

    def set_signals(self, mdl: Any, ctrl: Any) -> None:
        """Set the speed reference and the load torque."""
        if ctrl.ext_ref.w_M is None:
            raise ValueError("The speed reference must be set")
        self.set_signal(self.lib.speed_reference(self.handle), ctrl.ext_ref.w_M)
        set_mechanics_signals(self, cast(Drive, mdl))

    @staticmethod
    def supports(mdl: Any, ctrl: Any) -> bool:
        """Whether the system is an induction machine drive."""
        return isinstance(mdl, Drive) and isinstance(mdl.machine, InductionMachine)

    @staticmethod
    def check_supported(mdl: Any, ctrl: Any) -> None:
        """Raise an error if the drive system is not supported."""
        if type(ctrl) is VHzControlSystem:
            check_supported_vhz_ctrl(ctrl.vhz_ctrl)
        elif type(ctrl) is VectorControlSystem:
            check_supported_cvc(ctrl.vector_ctrl)
            if not isinstance(ctrl.speed_ctrl, PIController):
                raise NotImplementedError(
                    "Speed-control mode with a speed controller required"
                )
        else:
            raise NotImplementedError(
                "Only VectorControlSystem and VHzControlSystem supported"
            )
        check_supported_control(ctrl, d_err=True)
        check_supported_drive(cast(Drive, mdl))
