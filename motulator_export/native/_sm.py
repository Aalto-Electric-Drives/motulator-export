"""
Native simulation of synchronous machine drives with flux-vector control.

The system is simulated by the library `c/native_sm.c`. Supported are the drive
models of `motulator_export.native._drive` with a synchronous machine, and
flux-vector control (`FluxVectorController`) in `VectorControlSystem` with a speed
controller, with the features of the C port (see `motulator_export.plecs.sm`) and the
duty-ratio error model of the PWM (without the minimum pulses, if compensated), the
gates enabled.

"""

from typing import Any, cast

from motulator.common.control._controllers import PIController
from motulator.drive.control._base import VectorControlSystem
from motulator.drive.control._sm_flux_vector import FluxVectorController
from motulator.drive.model import Drive, SynchronousMachine
from motulator.drive.utils._parameters import (
    SaturatedSynchronousMachinePars,
    SynchronousMachinePars,
)

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
from motulator_export.plecs.sm import _fit_speed_dependent_gain, export_gradnet

# Functions of the library setting the parameters of the control system
API: Signatures = {
    "set_control_machine": (None, [c_void_p] + [c_double] * 5),
    "control_flux_map": (c_void_p, [c_void_p]),
    "set_control_machine_flux_map": (None, [c_void_p, c_double, c_double]),
    "set_flux_vector_controller": (None, [c_void_p] + [c_double] * 13 + [c_int]),
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
        fvc = cast(FluxVectorController, ctrl.vector_ctrl)
        est_par = fvc.par
        if isinstance(est_par, SaturatedSynchronousMachinePars):
            g = export_gradnet(est_par.psi_s_dq_fcn)
            self.set_gradnet(lib.control_flux_map(h), g)
            lib.set_control_machine_flux_map(h, est_par.n_p, est_par.R_s)
        else:
            lib.set_control_machine(
                h, est_par.n_p, est_par.R_s, est_par.L_d, est_par.L_q, est_par.psi_f
            )
        cfg = fvc.cfg
        k_o = [None, None] if cfg.k_o is None else _speed_gain(cfg.k_o)
        lib.set_flux_vector_controller(
            h,
            cfg.i_s_max,
            cfg.alpha_tau,
            value(cfg.alpha_psi),
            value(cfg.alpha_i),
            value(cfg.alpha_o),
            value(k_o[0]),
            value(k_o[1]),
            value(cfg.psi_s_min),
            cfg.psi_s_max,
            cfg.k_u,
            cfg.k_mtpv,
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
        mdl, ctrl = cast(Drive, mdl), cast(VectorControlSystem, ctrl)
        if ctrl.ext_ref.w_M is None:
            raise ValueError("Speed reference must be set")
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


def _speed_gain(k: Any) -> list[float]:
    """Express the observer gain as k(w_m) = k0 + k1*abs(w_m)."""
    return _fit_speed_dependent_gain(k, "k_o")


def _check_supported_control(ctrl: Any) -> None:
    """Raise an error if the control system is not supported."""
    if type(ctrl) is not VectorControlSystem:
        raise NotImplementedError("Only VectorControlSystem supported")
    fvc = ctrl.vector_ctrl
    if type(fvc) is not FluxVectorController:
        raise NotImplementedError("Only FluxVectorController supported")
    cfg = fvc.cfg
    if cfg.online_ref or cfg.k_f is not None:
        raise NotImplementedError("Only offline references and k_f=None supported")
    if cfg.k_o is not None:
        _speed_gain(cfg.k_o)  # Raises if not supported
    if isinstance(fvc.par, SaturatedSynchronousMachinePars):
        if fvc.par.psi_s_dq_fcn is None:
            raise NotImplementedError("A flux map (psi_s_dq_fcn) is required")
        export_gradnet(fvc.par.psi_s_dq_fcn)  # Raises if not supported
    elif not isinstance(fvc.par, SynchronousMachinePars):
        raise NotImplementedError("Machine model of the control system not supported")
    if not isinstance(ctrl.speed_ctrl, PIController):
        raise NotImplementedError("Speed-control mode with a speed controller required")
    check_supported_control(ctrl, d_err=True)
