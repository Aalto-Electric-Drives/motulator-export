"""
Native simulation of induction machine drives with current-vector control,
flux-vector control, or observer-based or open-loop V/Hz control.

The system is simulated by the library `c/native_im.c`. Supported are the drive
models of `motulator_export.native._drive` with an induction machine, and

- current-vector control (`CurrentVectorController`) or flux-vector control
  (`FluxVectorController`) in `VectorControlSystem` with a speed controller, or in
  the torque-control mode without it (`speed_ctrl=None`), or
- observer-based V/Hz control (`ObserverBasedVHzController`) or open-loop V/Hz
  control (`OpenLoopVHzController` with an affine V/Hz curve, see `_im.vhz_curve`) in
  `VHzControlSystem`,

with the features of the C port (see `motulator_export.plecs._im`) and the duty-ratio
error model of the PWM, also with the gates blocked (`ControlSystem.set_enable`, a
constant, a `StepSignal`, or a function of time).

"""

from typing import Any, cast

from motulator.drive.control._base import VectorControlSystem, VHzControlSystem
from motulator.drive.control._im_current_vector import CurrentVectorController
from motulator.drive.control._im_flux_vector import FluxVectorController
from motulator.drive.control._im_open_loop_vhz import OpenLoopVHzController
from motulator.drive.model import Drive, InductionMachine

from motulator_export._common import check_mechanical_sensor
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
# control), or of the arguments of OpenLoopVHzController
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
    "set_open_loop_vhz_controller": (None, [c_void_p] + [c_double] * 7),
    **PWM_API,
}


class InductionMachineDriveSystem(NativeSystem):
    """
    Induction machine drive with current-vector control, flux-vector control, or
    observer-based or open-loop V/Hz control in the native library.

    Parameters
    ----------
    mdl : Drive
        Continuous-time system model.
    ctrl : VectorControlSystem | VHzControlSystem
        Discrete-time control system.

    """

    def __init__(self, mdl: Any, ctrl: Any, front: Any = None) -> None:
        self.check_supported(mdl, ctrl, front)
        super().__init__("im", {**DRIVE_API, **API})
        lib, h = self.lib, self.handle
        set_drive(self, cast(Drive, mdl), front)

        # Control system
        if isinstance(ctrl, VHzControlSystem):
            v = export_vhz_values(ctrl)
            if isinstance(ctrl.inner_ctrl, OpenLoopVHzController):
                fields = ["n_p", "u_0", "k", "T_s", "slew_rate"]
                lib.set_open_loop_vhz_controller(h, *c_args(v, fields))
            else:
                lib.set_control_machine(h, *c_args(v, PAR_FIELDS))
                lib.set_vhz_controller(h, *c_args(v, [*VHZ_CFG_FIELDS, "slew_rate"]))
        else:
            ctrl = cast(VectorControlSystem, ctrl)
            if isinstance(ctrl.inner_ctrl, FluxVectorController):
                v = fvc_values(ctrl.inner_ctrl)
                lib.set_control_machine(h, *c_args(v, PAR_FIELDS))
                lib.set_flux_vector_controller(h, *c_args(v, FVC_CFG_FIELDS))
            else:
                v = cvc_values(cast(CurrentVectorController, ctrl.inner_ctrl))
                lib.set_control_machine(h, *c_args(v, PAR_FIELDS))
                lib.set_current_vector_controller(h, *c_args(v, CFG_FIELDS))
            lib.set_observer_speed(h, v["observer_w_M"])
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
    def check_supported(mdl: Any, ctrl: Any, front: Any = None) -> None:
        """
        Raise an error if the drive system, fed from the diode bridge `front` if
        given, is not supported.
        """
        if type(ctrl) is VHzControlSystem:
            check_supported_vhz_ctrl(ctrl.inner_ctrl)
        elif type(ctrl) is VectorControlSystem:
            check_supported_vector_ctrl(ctrl.inner_ctrl)
            check_supported_speed_control(ctrl)
        else:
            raise NotImplementedError(
                "Only VectorControlSystem and VHzControlSystem supported"
            )
        check_supported_control(ctrl, d_err=True)
        check_supported_drive(cast(Drive, mdl), front)
        check_mechanical_sensor(cast(Drive, mdl), ctrl)
