"""
Native simulation of induction machine drives with current-vector control,
flux-vector control, or observer-based V/Hz control.

The system is simulated by the library `c/native_im.c`. Supported are the drive
models of `motulator_export.native._drive` with an induction machine, and

- current-vector control (`CurrentVectorController`) or flux-vector control
  (`FluxVectorController`) in `VectorControlSystem` with a speed controller, or in
  the torque-control mode without it (`speed_ctrl=None`), or
- observer-based V/Hz control (`ObserverBasedVHzController`) in `VHzControlSystem`,
  including open-loop V/Hz control (`L_M = inf`),

with the features of the C port (see `motulator_export.plecs._im`) and the duty-ratio
error model of the PWM, also with the gates blocked (`ControlSystem.set_enable`, a
constant or a `StepSignal`).

"""

from typing import Any, cast

from motulator.drive.control._base import VectorControlSystem, VHzControlSystem
from motulator.drive.control._im_current_vector import CurrentVectorController
from motulator.drive.control._im_flux_vector import FluxVectorController
from motulator.drive.model import Drive, InductionMachine

from motulator_export.native._build import Signatures
from motulator_export.native._drive import (
    DRIVE_API,
    check_supported_drive,
    check_supported_speed_control,
    set_drive,
    set_mechanics_signals,
    set_reference,
    set_speed_control,
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
    check_supported_vector_ctrl,
    check_supported_vhz_ctrl,
    cvc_values,
    export_vhz_values,
    fvc_values,
)
from motulator_export.plecs._im_control import CFG_FIELDS, PAR_FIELDS
from motulator_export.plecs._im_fvc_control import CFG_FIELDS as FVC_CFG_FIELDS
from motulator_export.plecs._im_vhz_control import CFG_FIELDS as VHZ_CFG_FIELDS

# Functions of the library setting the parameters of the control system, the
# arguments in the order of PAR_FIELDS and CFG_FIELDS (and the slew rate of V/Hz
# control)
API: Signatures = {
    "set_control_machine": (None, [c_void_p] + [c_double] * 5),
    "set_current_vector_controller": (
        None,
        [c_void_p] + [c_double] * 9 + [c_int, c_double, c_int],
    ),
    "set_flux_vector_controller": (
        None,
        [c_void_p] + [c_double] * 10 + [c_int, c_double],
    ),
    "set_vhz_controller": (None, [c_void_p] + [c_double] * 9),
    **PWM_API,
}


class InductionMachineDriveSystem(NativeSystem):
    """
    Induction machine drive with current-vector control, flux-vector control, or
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
            if isinstance(ctrl.vector_ctrl, FluxVectorController):
                v = fvc_values(ctrl.vector_ctrl)
                lib.set_control_machine(h, *c_args(v, PAR_FIELDS))
                lib.set_flux_vector_controller(h, *c_args(v, FVC_CFG_FIELDS))
            else:
                v = cvc_values(cast(CurrentVectorController, ctrl.vector_ctrl))
                lib.set_control_machine(h, *c_args(v, PAR_FIELDS))
                lib.set_current_vector_controller(h, *c_args(v, CFG_FIELDS))
            set_speed_control(self, ctrl)
        self.set_pwm(ctrl.pwm)

    def set_signals(self, mdl: Any, ctrl: Any) -> None:
        """
        Set the speed reference (or the torque reference in the torque-control mode)
        and the signals of the mechanics.
        """
        set_reference(self, ctrl)
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
            check_supported_vector_ctrl(ctrl.vector_ctrl)
            check_supported_speed_control(ctrl)
        else:
            raise NotImplementedError(
                "Only VectorControlSystem and VHzControlSystem supported"
            )
        check_supported_control(ctrl, d_err=True)
        check_supported_drive(cast(Drive, mdl))
