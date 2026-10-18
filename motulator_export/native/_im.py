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
from motulator.drive.utils._parameters import InductionMachineInvGammaPars

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
    c_double,
    c_int,
    c_void_p,
    check_supported_control,
    value,
)
from motulator_export.plecs._im import (
    _check_supported_cvc,
    _check_supported_vhz_ctrl,
    export_vhz_values,
)

# Functions of the library setting the parameters of the control system
API: Signatures = {
    "set_control_machine": (None, [c_void_p] + [c_double] * 5),
    "set_current_vector_controller": (None, [c_void_p] + [c_double] * 10 + [c_int]),
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
            lib.set_control_machine(
                h, v["n_p"], v["R_s"], v["R_R"], v["L_sgm"], v["L_M"]
            )
            lib.set_vhz_controller(
                h,
                v["psi_s_nom"],
                v["i_s_max"],
                v["alpha_psi"],
                v["alpha_tau"],
                v["alpha_f"],
                v["k_u"],
                v["k_b"],
                v["T_s"],
                v["slew_rate"],
            )
        else:
            ctrl = cast(VectorControlSystem, ctrl)
            cvc = cast(CurrentVectorController, ctrl.vector_ctrl)
            par = cast(InductionMachineInvGammaPars, cvc.reference_gen.par)
            lib.set_control_machine(h, par.n_p, par.R_s, par.R_R, par.L_sgm, par.L_M)
            cfg = cvc.cfg
            lib.set_current_vector_controller(
                h,
                cfg.psi_s_nom,
                cfg.i_s_max,
                cfg.alpha_c,
                value(cfg.alpha_i),
                value(cfg.alpha_o),
                cfg.w_s_nom,
                cfg.k_u,
                value(cfg.k_fw),
                value(cfg.J),
                cfg.T_s,
                cfg.sensorless,
            )
            speed_ctrl = cast(PIController, ctrl.speed_ctrl)
            lib.set_speed_controller(
                h, speed_ctrl.k_p, speed_ctrl.k_t, speed_ctrl.alpha_i, speed_ctrl.u_max
            )
        self.set_pwm(ctrl.pwm)

    def set_signals(self, mdl: Any, ctrl: Any) -> None:
        """Set the speed reference and the load torque."""
        if ctrl.ext_ref.w_M is None:
            raise ValueError("Speed reference must be set")
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
            _check_supported_vhz_ctrl(ctrl.vhz_ctrl)
        elif type(ctrl) is VectorControlSystem:
            _check_supported_cvc(ctrl.vector_ctrl)
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
