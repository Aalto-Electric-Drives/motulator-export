"""
Native simulation of synchronous machine drives with flux-vector control.

The system is simulated by the library `c/native_sm.c`. Currently supported:

- Synchronous machine (`SynchronousMachine`): `SynchronousMachinePars`, or
  `SpatialSaturatedSynchronousMachinePars` with a GradNet current map with spatial
  harmonics (`CurrentMapWithHarmonics`), without core losses
- Mechanical system (`MechanicalSystem`) with a constant friction coefficient
- Converter (`VoltageSourceConverter`) with a constant DC-bus voltage, the dead time,
  and the threshold voltage (with `sign=np.sign`). The PWM model is `ZOH` (the
  default) or `CarrierComparison` (`pwm=True`), with any computational delay and the
  single or double update, starting from their initial states.
- Flux-vector control (`FluxVectorController`) in `VectorControlSystem` with a speed
  controller, with the features of the C port (see `motulator_export.plecs.sm`) and
  the duty-ratio error model of the PWM (without the minimum pulses, if compensated)
- Ideal sensors, the gates enabled

"""

from typing import Any, cast

import motulator.drive.gradnet as gn
import numpy as np
from motulator.common.control._controllers import PIController
from motulator.common.model._converter import VoltageSourceConverter
from motulator.common.model._pwm import ZOH, CarrierComparison
from motulator.drive.control._base import VectorControlSystem
from motulator.drive.control._sm_flux_vector import FluxVectorController
from motulator.drive.model import Drive, MechanicalSystem, SynchronousMachine
from motulator.drive.utils._parameters import (
    SaturatedSynchronousMachinePars,
    SpatialSaturatedSynchronousMachinePars,
    SynchronousMachinePars,
)

from motulator_export.native._build import Signatures
from motulator_export.native._system import (
    GRADNET_API,
    NativeSystem,
    c_double,
    c_int,
    c_void_p,
    value,
)
from motulator_export.plecs._common import _c_define, _check_supported_pwm, gate_enable
from motulator_export.plecs._control import pwm_values
from motulator_export.plecs.sm import _fit_speed_dependent_gain, export_gradnet

DELAY_MAX_LENGTH = _c_define("sim.h", "DELAY_MAX_LENGTH")

# Functions of the library setting the parameters
API: Signatures = {
    "set_machine": (None, [c_void_p] + [c_double] * 5),
    "machine_current_map": (c_void_p, [c_void_p]),
    "set_machine_current_map": (None, [c_void_p, c_double, c_double, c_int]),
    "set_mechanics": (None, [c_void_p, c_double, c_double]),
    "load_torque": (c_void_p, [c_void_p]),
    "set_converter": (None, [c_void_p, c_double, c_double]),
    "set_modulator": (None, [c_void_p, c_int, c_double, c_double, c_int, c_int]),
    "set_control_machine": (None, [c_void_p] + [c_double] * 5),
    "control_flux_map": (c_void_p, [c_void_p]),
    "set_control_machine_flux_map": (None, [c_void_p, c_double, c_double]),
    "set_flux_vector_controller": (None, [c_void_p] + [c_double] * 13 + [c_int]),
    "set_speed_controller": (None, [c_void_p] + [c_double] * 4),
    "set_pwm": (
        None,
        [c_void_p, c_int, c_double, c_double, c_double, c_int, c_double, c_int],
    ),
    "speed_reference": (c_void_p, [c_void_p]),
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
        super().__init__("sm", {**API, **GRADNET_API})
        lib, h = self.lib, self.handle

        # System model
        par = mdl.machine.par
        if isinstance(par, SpatialSaturatedSynchronousMachinePars):
            g = export_gradnet(_current_map(par))
            self.set_gradnet(lib.machine_current_map(h), g)
            lib.set_machine_current_map(h, par.n_p, par.R_s, g["k"])
        else:
            par = cast(SynchronousMachinePars, par)
            lib.set_machine(h, par.n_p, par.R_s, par.L_d, par.L_q, par.psi_f)
        mechanics = cast(MechanicalSystem, mdl.mechanics)
        lib.set_mechanics(h, mechanics.J, mechanics.B_L)
        lib.set_converter(h, mdl.converter.u_dc, mdl.converter.u_th)
        pwm = mdl.pwm
        N = pwm.N if isinstance(pwm, CarrierComparison) else 0
        delay = len(mdl.delay.data)
        lib.set_modulator(h, N > 0, N, pwm.t_d, delay, mdl.single_update)

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
        v = pwm_values(ctrl.pwm)
        lib.set_pwm(
            h,
            v["pwm_overmodulation"] == "MME",
            v["pwm_t_d"],
            v["pwm_u_th"],
            v["pwm_i_0"],
            v["pwm_feedforward"],
            v["pwm_d_min"],
            v["pwm_single_update"],
        )

    def set_signals(self, mdl: Any, ctrl: Any) -> None:
        """Set the speed reference and the load torque."""
        mdl, ctrl = cast(Drive, mdl), cast(VectorControlSystem, ctrl)
        if ctrl.ext_ref.w_M is None:
            raise ValueError("Speed reference must be set")
        self.set_signal(self.lib.speed_reference(self.handle), ctrl.ext_ref.w_M)
        self.set_signal(self.lib.load_torque(self.handle), mdl.mechanics.inp.tau_L)

    @staticmethod
    def supports(mdl: Any, ctrl: Any) -> bool:
        """Whether the system is a synchronous machine drive."""
        return isinstance(mdl, Drive) and isinstance(mdl.machine, SynchronousMachine)

    @staticmethod
    def check_supported(mdl: Any, ctrl: Any) -> None:
        """Raise an error if the drive system is not supported."""
        _check_supported_control(ctrl)
        _check_supported_machine(mdl)
        _check_supported_converter(mdl)


def _current_map(par: SpatialSaturatedSynchronousMachinePars) -> Any:
    """GradNet current map of the machine model, checking its type."""
    if type(par.magnetic_map_fcn) is not gn.CurrentMapWithHarmonics:
        raise NotImplementedError(
            "Only GradNet current maps with spatial harmonics "
            "(CurrentMapWithHarmonics) supported in "
            "SpatialSaturatedSynchronousMachinePars"
        )
    return par.magnetic_map_fcn


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
    _check_supported_pwm(ctrl, d_err=True)
    pwm = ctrl.pwm
    pwm_values(pwm)  # Raises if the duty-ratio error model is not supported
    if pwm.d_err is not None and pwm.feedforward and pwm.d_min > 0:
        raise NotImplementedError(
            "Minimum pulses (d_min) with the compensation of the duty-ratio error not "
            "supported"
        )
    if gate_enable(ctrl) != 1:
        raise NotImplementedError("Gate blocking (set_enable) not supported")


def _check_supported_machine(mdl: Drive) -> None:
    """Raise an error if the machine or the mechanics is not supported."""
    machine = mdl.machine
    if type(machine) is not SynchronousMachine:
        raise NotImplementedError("Only SynchronousMachine supported")
    par = machine.par
    if isinstance(par, SpatialSaturatedSynchronousMachinePars):
        export_gradnet(_current_map(par))  # Raises if not supported
    elif not isinstance(par, SynchronousMachinePars):
        raise NotImplementedError(
            "Only SynchronousMachinePars and SpatialSaturatedSynchronousMachinePars "
            "supported in the machine model"
        )
    if par.G_c != 0:
        raise NotImplementedError("Core losses not supported")
    if machine.i_s_sensor is not None:
        raise NotImplementedError("Sensor models not supported")
    mechanics = mdl.mechanics
    if type(mechanics) is not MechanicalSystem or callable(mechanics.B_L):
        raise NotImplementedError(
            "Only MechanicalSystem with a constant friction coefficient supported"
        )


def _check_supported_converter(mdl: Drive) -> None:
    """Raise an error if the converter, its PWM model, or the delay is not supported."""
    converter, pwm = mdl.converter, mdl.pwm
    if mdl.lc_filter is not None:
        raise NotImplementedError("LC filter not supported")
    if type(converter) is not VoltageSourceConverter or converter.sign is not np.sign:
        raise NotImplementedError(
            "Only VoltageSourceConverter with sign=np.sign supported"
        )
    if mdl.dc_bus is not None or converter.u_dc is None:
        raise NotImplementedError("Only a constant DC-bus voltage supported")
    if converter.u_dc_sensor is not None:
        raise NotImplementedError("Sensor models not supported")
    if type(pwm) not in (ZOH, CarrierComparison):
        raise NotImplementedError(
            "Only the PWM models ZOH and CarrierComparison supported"
        )
    delay = mdl.delay.data
    if len(delay) > DELAY_MAX_LENGTH:
        raise NotImplementedError(f"At most the delay {DELAY_MAX_LENGTH} supported")
    initial = not np.any(delay)
    if isinstance(pwm, CarrierComparison):
        initial &= pwm._rising_edge and pwm._command is None
    if not initial:
        raise NotImplementedError("Only the initial state of the PWM model supported")
