"""
Drive models of the native libraries (`c/drive.c`), shared by the drive systems.

Supported:

- Synchronous machine (`SynchronousMachine`): `SynchronousMachinePars`, or
  `SpatialSaturatedSynchronousMachinePars` with a GradNet current map with spatial
  harmonics (`CurrentMapWithHarmonics`), without core losses
- Induction machine (`InductionMachine`) with constant parameters, without core
  losses
- Mechanical system (`MechanicalSystem`), the friction coefficient being constant or
  a function of the speed, or the external rotor speed (`ExternalRotorSpeed`),
  constant or a function of time
- Converter (`VoltageSourceConverter`) with a constant DC-bus voltage or fed by a
  DC-bus model (`DCBus` or `DiodeBridgeDCBus`, without an external DC current), the
  dead time, and the threshold voltage (with `sign=np.sign`). The PWM model is `ZOH`
  (the default) or `CarrierComparison` (`pwm=True`), with any computational delay and
  the single or double update, starting from their initial states.
- Ideal sensors

"""

from typing import Any, cast

import motulator.drive.gradnet as gn
from motulator.common.control._controllers import PIController
from motulator.drive.control._base import VectorControlSystem
from motulator.drive.model import (
    Drive,
    ExternalRotorSpeed,
    InductionMachine,
    MechanicalSystem,
    SynchronousMachine,
)
from motulator.drive.utils._parameters import (
    InductionMachinePars,
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
    check_supported_converter,
)
from motulator_export.plecs._sm import export_gradnet

# Functions of the libraries of the drives setting the parameters of the model
DRIVE_API: Signatures = {
    "set_machine": (None, [c_void_p] + [c_double] * 5),
    "machine_current_map": (c_void_p, [c_void_p]),
    "set_machine_current_map": (None, [c_void_p, c_double, c_double, c_int]),
    "set_induction_machine": (None, [c_void_p] + [c_double] * 5),
    "set_mechanics": (None, [c_void_p, c_double]),
    "set_external_rotor_speed": (None, [c_void_p]),
    "external_rotor_speed": (c_void_p, [c_void_p]),
    "friction": (c_void_p, [c_void_p]),
    "load_torque": (c_void_p, [c_void_p]),
    "set_speed_controller": (None, [c_void_p] + [c_double] * 4),
    "set_torque_control": (None, [c_void_p]),
    "speed_reference": (c_void_p, [c_void_p]),
    **GRADNET_API,
}


def set_drive(system: NativeSystem, mdl: Drive) -> None:
    """Set the parameters of the drive model `mdl` in the library of `system`."""
    lib, h = system.lib, system.handle
    machine = mdl.machine
    if isinstance(machine, InductionMachine):
        im = cast(InductionMachinePars, machine.par)
        lib.set_induction_machine(h, im.n_p, im.R_s, im.R_r, im.L_ell, im.L_s)
    elif isinstance(machine.par, SpatialSaturatedSynchronousMachinePars):
        par = machine.par
        g = export_gradnet(current_map(par))
        system.set_gradnet(lib.machine_current_map(h), g)
        lib.set_machine_current_map(h, par.n_p, par.R_s, g["k"])
    else:
        par = cast(SynchronousMachinePars, machine.par)
        lib.set_machine(h, par.n_p, par.R_s, par.L_d, par.L_q, par.psi_f)
    if isinstance(mdl.mechanics, ExternalRotorSpeed):
        lib.set_external_rotor_speed(h)
    else:
        lib.set_mechanics(h, cast(MechanicalSystem, mdl.mechanics).J)
    system.set_converter(mdl)


def set_mechanics_signals(system: NativeSystem, mdl: Drive) -> None:
    """
    Set the friction coefficient and the external load torque of the mechanics, or
    the external rotor speed.
    """
    lib, h = system.lib, system.handle
    if isinstance(mdl.mechanics, ExternalRotorSpeed):
        system.set_signal(lib.external_rotor_speed(h), mdl.mechanics.w_M)
        return
    mechanics = cast(MechanicalSystem, mdl.mechanics)
    system.set_signal(lib.friction(h), mechanics.B_L)
    system.set_signal(lib.load_torque(h), mechanics.inp.tau_L)


def set_speed_control(system: NativeSystem, ctrl: VectorControlSystem) -> None:
    """
    Set the speed controller of the vector control system, or its torque-control mode
    without the speed controller (`speed_ctrl = None`).
    """
    lib, h = system.lib, system.handle
    if ctrl.speed_ctrl is None:
        lib.set_torque_control(h)
        return
    speed_ctrl = cast(PIController, ctrl.speed_ctrl)
    lib.set_speed_controller(
        h, speed_ctrl.k_p, speed_ctrl.k_t, speed_ctrl.alpha_i, speed_ctrl.u_max
    )


def set_reference(system: NativeSystem, ctrl: Any) -> None:
    """
    Set the speed reference of the control system, or the torque reference in the
    torque-control mode of a vector control system.
    """
    if getattr(ctrl, "speed_ctrl", True) is None:
        if ctrl.ext_ref.tau_M is None:
            raise ValueError("The torque reference must be set")
        ref = ctrl.ext_ref.tau_M
    else:
        if ctrl.ext_ref.w_M is None:
            raise ValueError("The speed reference must be set")
        ref = ctrl.ext_ref.w_M
    system.set_signal(system.lib.speed_reference(system.handle), ref)


def check_supported_speed_control(ctrl: VectorControlSystem) -> None:
    """
    Raise an error if the speed controller of the vector control system is not a PI
    controller (e.g., SpeedController), or None in the torque-control mode.
    """
    if ctrl.speed_ctrl is not None and not isinstance(ctrl.speed_ctrl, PIController):
        raise NotImplementedError(
            "Only a PI speed controller (e.g., SpeedController) or none supported"
        )


def current_map(par: SpatialSaturatedSynchronousMachinePars) -> Any:
    """GradNet current map of the machine model, checking its type."""
    if type(par.magnetic_map_fcn) is not gn.CurrentMapWithHarmonics:
        raise NotImplementedError(
            "Only GradNet current maps with spatial harmonics "
            "(CurrentMapWithHarmonics) supported in "
            "SpatialSaturatedSynchronousMachinePars"
        )
    return par.magnetic_map_fcn


def check_supported_drive(mdl: Drive) -> None:
    """Raise an error if the machine, the mechanics, or the converter is unsupported."""
    machine = mdl.machine
    par = machine.par
    if type(machine) is InductionMachine:
        if not isinstance(par, InductionMachinePars) or callable(par.L_s):
            raise NotImplementedError("Only InductionMachine with constant parameters")
    elif type(machine) is SynchronousMachine:
        if isinstance(par, SpatialSaturatedSynchronousMachinePars):
            export_gradnet(current_map(par))  # Raises if not supported
        elif not isinstance(par, SynchronousMachinePars):
            raise NotImplementedError(
                "Only SynchronousMachinePars and "
                "SpatialSaturatedSynchronousMachinePars supported in the machine model"
            )
    else:
        raise NotImplementedError(
            "Only SynchronousMachine and InductionMachine supported"
        )
    if par.G_c != 0:
        raise NotImplementedError("Core losses not supported")
    if machine.i_s_sensor is not None:
        raise NotImplementedError("Sensor models not supported")
    if type(mdl.mechanics) not in (MechanicalSystem, ExternalRotorSpeed):
        raise NotImplementedError(
            "Only MechanicalSystem and ExternalRotorSpeed supported"
        )
    if mdl.lc_filter is not None:
        raise NotImplementedError("LC filter not supported")
    check_supported_converter(mdl)
