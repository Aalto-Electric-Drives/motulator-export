"""
Export motulator induction machine drives to PLECS.

This module converts an induction machine drive with current-vector control,
flux-vector control, or observer-based V/Hz control into a PLECS Standalone model, in
the same way as `motulator_export.plecs._sm` does for synchronous machine drives. In
the control system, each class of motulator is a block with a mask of its arguments
(see `_im_control`, `_im_fvc_control`, and `_im_vhz_control`), with the C port in
`c/im_current_vector.c` and `c/im_flux_vector.c`. The parameters are defined in the
initialization commands of the PLECS model as the structs par
(`InductionMachineInvGammaPars`), cfg (`CurrentVectorControllerCfg`,
`FluxVectorControllerCfg`, or `ObserverBasedVHzControllerCfg`), speed_ctrl
(`SpeedController`) or slew_rate (of `VHzControlSystem`), and pwm (`PWM`).
The machine is the PLECS squirrel-cage induction machine, parametrized as the T
model equivalent to the inverse-Γ model of motulator.

Currently supported:

- Induction machine with constant parameters (`InductionMachineInvGammaPars`, or
  `InductionMachinePars` with a constant `L_s`) without core losses
- Mechanical system (`MechanicalSystem` without friction), the converter, and the DC
  bus as in `motulator_export.plecs._sm`
- Current-vector control (`CurrentVectorController`, also with the direct
  discrete-time current controller) or flux-vector control (`FluxVectorController`)
  in the sensorless or sensored mode with the default observer gain `k_o`, and a
  speed controller (`SpeedController`) in `VectorControlSystem`
- Observer-based V/Hz control (`ObserverBasedVHzController`) with the default
  observer gain `k_o` in `VHzControlSystem`, including pure open-loop V/Hz control
  as its special case (`L_M = inf` in the machine model of the control system)
- The MPE or MME overmodulation of the PWM (`overmodulation`, MME by default in V/Hz
  control), the minimum duty ratio (`d_min`), and the single update
  (`single_update`)
- Dead time and threshold voltage of the converter (`t_d` and `u_th` with
  `sign=np.sign`), modeled with the Blanking Time block and the IGBT converter of
  PLECS (with the threshold voltage, the IGBT converter with parasitics, whose
  forward voltages are `u_th`), and their compensation in the PWM
  (`PWM(d_err=InverterNonlinearity(t_d, u_th, sign))` with `sign=np.sign` or the
  smooth `sign=lambda i: np.tanh(i/i_0)`)
- Gate blocking (`set_enable`): the input `enable` of the control system, which
  then runs in standby, and the gates of the IGBT converter blocked

"""

from pathlib import Path
from typing import Any, cast

import numpy as np
from motulator.common.model._converter import DiodeBridgeDCBus
from motulator.common.model._pwm import CarrierComparison
from motulator.drive.control._base import VectorControlSystem, VHzControlSystem
from motulator.drive.control._im_current_vector import CurrentVectorController
from motulator.drive.control._im_flux_vector import (
    FluxVectorController,
    ObserverBasedVHzController,
)
from motulator.drive.model import Drive, InductionMachine
from motulator.drive.utils._parameters import (
    InductionMachineInvGammaPars,
    InductionMachinePars,
)

from motulator_export._common import StepSignal, gate_enable
from motulator_export.plecs._common import (
    BLANKING_DX,
    DC_DX,
    ENABLE,
    GATE_DX,
    MACH,
    ControlBlock,
    _add_blanking_time,
    _add_control_system,
    _add_converter,
    _add_dc_bus,
    _add_delay,
    _add_pwm,
    _write_model,
)
from motulator_export.plecs._control import INIT_COMMENT, T_S_EXPR, VALUES, pwm_values
from motulator_export.plecs._drive import (
    EXTERNAL_SPEED_J,
    MACHINE_FRAME,
    MACHINE_PROBES,
    MACHINE_TERMINALS,
    MDL_OUTPUTS,
    _add_drive_outputs,
    _add_mechanics,
    _add_phase_lines,
    _check_supported_plant,
    _check_supported_speed_control,
    _check_supported_vhz_control,
    drive_references,
    external_speed,
    mechanics_and_converter_variables,
    speed_ctrl_values,
    torque_control,
)
from motulator_export.plecs._im_control import (
    CFG_FIELDS,
    CVC_BLOCK,
    CVC_TORQUE_BLOCK,
    PAR_FIELDS,
    init_script,
)
from motulator_export.plecs._im_fvc_control import CFG_FIELDS as FVC_CFG_FIELDS
from motulator_export.plecs._im_fvc_control import FVC_BLOCK, FVC_TORQUE_BLOCK
from motulator_export.plecs._im_fvc_control import init_script as fvc_init_script
from motulator_export.plecs._im_vhz_control import CFG_FIELDS as VHZ_CFG_FIELDS
from motulator_export.plecs._im_vhz_control import VHZ_BLOCK
from motulator_export.plecs._im_vhz_control import init_script as vhz_init_script
from motulator_export.plecs._rpc import simulate_plecs
from motulator_export.plecs._schematic import _probe, _Schematic, _terminals

__all__ = ["simulate", "write_model"]


def check_supported(mdl: Drive, ctrl: VectorControlSystem | VHzControlSystem) -> None:
    """Raise an error if the drive system is not supported."""
    par = mdl.machine.par
    if (
        not isinstance(mdl.machine, InductionMachine)
        or not isinstance(par, InductionMachinePars)
        or callable(par.L_s)
    ):
        raise NotImplementedError("Only InductionMachine with constant parameters")
    if par.G_c != 0:
        raise NotImplementedError("Core losses not supported")
    _check_supported_plant(mdl, dead_time=True)
    _check_supported_control(ctrl)


def _check_supported_control(ctrl: VectorControlSystem | VHzControlSystem) -> None:
    """Raise an error if the control system is not supported."""
    if isinstance(ctrl, VHzControlSystem):
        check_supported_vhz_ctrl(ctrl.vhz_ctrl)
        _check_supported_vhz_control(ctrl)
        return
    check_supported_vector_ctrl(ctrl.vector_ctrl)
    _check_supported_speed_control(ctrl, dead_time=True)


def check_supported_vector_ctrl(vector_ctrl: Any) -> None:
    """
    Raise an error if the vector controller (current-vector or flux-vector control)
    is not supported.
    """
    if isinstance(vector_ctrl, FluxVectorController):
        check_supported_fvc(vector_ctrl)
    else:
        check_supported_cvc(vector_ctrl)


def check_supported_fvc(fvc: Any) -> None:
    """Raise an error if the flux-vector controller is not supported."""
    if not isinstance(fvc, FluxVectorController):
        raise NotImplementedError("Only FluxVectorController supported")
    if fvc.cfg.k_o is not None:
        raise NotImplementedError("Only the default observer gain k_o supported")
    if not isinstance(fvc.reference_gen.par, InductionMachineInvGammaPars):
        raise NotImplementedError("Only InductionMachineInvGammaPars supported")


def check_supported_cvc(cvc: Any) -> None:
    """Raise an error if the current-vector controller is not supported."""
    if not isinstance(cvc, CurrentVectorController):
        raise NotImplementedError("Only CurrentVectorController supported")
    if cvc.cfg.k_o is not None:
        raise NotImplementedError("Only the default observer gain k_o supported")
    if not isinstance(cvc.reference_gen.par, InductionMachineInvGammaPars):
        raise NotImplementedError("Only InductionMachineInvGammaPars supported")


def check_supported_vhz_ctrl(vhz: Any) -> None:
    """Raise an error if the V/Hz controller is not supported."""
    if not isinstance(vhz, ObserverBasedVHzController):
        raise NotImplementedError("Only ObserverBasedVHzController supported")
    if vhz.cfg.k_o is not None:
        raise NotImplementedError("Only the default observer gain k_o supported")
    if not isinstance(vhz.reference_gen.par, InductionMachineInvGammaPars):
        raise NotImplementedError("Only InductionMachineInvGammaPars supported")


def control_block(ctrl: VectorControlSystem | VHzControlSystem) -> ControlBlock:
    """
    Control-system block of current-vector or flux-vector control (also in the
    torque-control mode), or V/Hz control.
    """
    if isinstance(ctrl, VHzControlSystem):
        return VHZ_BLOCK
    torque = torque_control(ctrl)
    if isinstance(ctrl.vector_ctrl, FluxVectorController):
        return FVC_TORQUE_BLOCK if torque else FVC_BLOCK
    return CVC_TORQUE_BLOCK if torque else CVC_BLOCK


def export_vhz_values(ctrl: VHzControlSystem) -> dict[str, Any]:
    """
    Get the parameter values of the V/Hz control system in the motulator API (see
    `_im_vhz_control.init_script`).
    """
    vhz = cast(ObserverBasedVHzController, ctrl.vhz_ctrl)
    values = _fields(vhz.reference_gen.par, PAR_FIELDS)
    values |= _fields(vhz.cfg, VHZ_CFG_FIELDS)
    values["slew_rate"] = ctrl.rate_limiter.rate_limit
    return values | pwm_values(ctrl.pwm)


def export_values(ctrl: VectorControlSystem) -> dict[str, Any]:
    """
    Get the parameter values of the control system in the motulator API, None for
    the defaults (see `_im_control.init_script` and `_im_fvc_control.init_script`).
    """
    if isinstance(ctrl.vector_ctrl, FluxVectorController):
        values = fvc_values(ctrl.vector_ctrl)
    else:
        values = cvc_values(cast(CurrentVectorController, ctrl.vector_ctrl))
    return values | speed_ctrl_values(ctrl) | pwm_values(ctrl.pwm)


def fvc_values(fvc: FluxVectorController) -> dict[str, Any]:
    """
    Get the parameter values of the flux-vector controller (see `export_values`):
    its machine model (`PAR_FIELDS`) and its configuration (`FVC_CFG_FIELDS`).
    """
    values = _fields(fvc.reference_gen.par, PAR_FIELDS)
    values |= _fields(fvc.cfg, FVC_CFG_FIELDS)
    values["sensorless"] = int(fvc.cfg.sensorless)
    return values


def cvc_values(cvc: CurrentVectorController) -> dict[str, Any]:
    """
    Get the parameter values of the current-vector controller (see `export_values`):
    its machine model (`PAR_FIELDS`) and its configuration (`CFG_FIELDS`).
    """
    values = _fields(cvc.reference_gen.par, PAR_FIELDS) | _fields(cvc.cfg, CFG_FIELDS)
    values["sensorless"] = int(cvc.cfg.sensorless)
    values["discrete"] = int(cvc.cfg.discrete)
    return values


def _fields(obj: Any, names: list[str]) -> dict[str, Any]:
    """Values of the fields of an object."""
    return {name: getattr(obj, name) for name in names}


def export_plant_variables(mdl: Drive) -> list[tuple[str, Any]]:
    """Workspace variables of the system model (inverse-Γ machine parameters)."""
    par = InductionMachineInvGammaPars.from_gamma_pars(
        cast(InductionMachinePars, mdl.machine.par)
    )
    variables = [(f"machine.{n}", v) for n, v in _fields(par, PAR_FIELDS).items()]
    return variables + mechanics_and_converter_variables(mdl)


def _add_im(sch: _Schematic, J: str = "mechanics.J") -> None:
    """
    Add the PLECS squirrel-cage induction machine (including the inertia J, see
    EXTERNAL_SPEED_J).

    The inverse-Γ model of motulator corresponds to the T model of PLECS with zero
    rotor leakage inductance.

    """
    machine = {
        "Rs": "machine.R_s",
        "Lls": "machine.L_sgm",
        "Rr": "machine.R_R",
        "Llr": "0",
        "Lm": "machine.L_M",
        "J": J,
        "F": "0",
        "p": "machine.n_p",
        "wm0": "mechanics.w_M0",
        "thm0": "mechanics.theta_M0",
        "is0": "[0 0]",
        "psisdq0": "[0 0]",
    }
    sch.component(
        "Reference",
        "Machine",
        MACH,
        machine,
        direction="up",
        label="east",
        src_component="Components/Electrical/Machines/Squirrel-Cage IM",
        extra=MACHINE_FRAME,
        trailer=_terminals(MACHINE_TERMINALS),
    )


def write_model(
    path: str | Path,
    mdl: Drive,
    ctrl: VectorControlSystem | VHzControlSystem,
    t_stop: float,
    outputs: bool = False,
    library: bool = False,
) -> Path:
    """
    Write a PLECS model of the induction machine drive, see `plecs.write_model`.

    The control system is current-vector or flux-vector control
    (`VectorControlSystem`) or observer-based V/Hz control (`VHzControlSystem`),
    whose inputs are `enable` (the gate enable of `ctrl`), the speed reference, the
    phase currents, the DC-bus voltage, and in vector control the measured rotor
    speed.

    """
    path = Path(path)
    check_supported(mdl, ctrl)
    block = control_block(ctrl)
    variables = export_plant_variables(mdl)
    if isinstance(ctrl, VHzControlSystem):
        values = export_vhz_values(ctrl)
        init = vhz_init_script(variables, values, INIT_COMMENT)
    elif isinstance(ctrl.vector_ctrl, FluxVectorController):
        values = export_values(ctrl)
        init = fvc_init_script(variables, values, INIT_COMMENT)
    else:
        values = export_values(ctrl)
        init = init_script(variables, values, INIT_COMMENT)
    ref, mech_signal = drive_references(mdl, ctrl, values["T_s"])
    T_s = T_S_EXPR
    sch = _Schematic()
    sources: list[tuple[str, StepSignal | float | str]] = [
        (ENABLE, gate_enable(ctrl, values["T_s"])),
        ("tau_M_ref" if torque_control(ctrl) else "w_M_ref", ref),
        ("i_s_abc", _probe("Machine", MACHINE_PROBES[:1])),
        ("u_dc meas.", _probe("u_dc", ["Measured voltage"])),
    ]
    if block is not VHZ_BLOCK:
        sources += [("w_M", _probe("Machine", MACHINE_PROBES[1:2]))]
    y_sources = _add_control_system(sch, block, sources, VALUES, library)
    _add_delay(sch, T_s, block)
    _add_pwm(sch, T_s, mdl.single_update)
    t_d = cast(CarrierComparison, mdl.pwm).t_d
    if t_d > 0:
        _add_blanking_time(sch)
    # The blanking time, the diode bridge, and its grid need space between the PWM and
    # the DC bus
    sch.dx = 320 if isinstance(mdl.dc_bus, DiodeBridgeDCBus) else 0
    sch.dx += BLANKING_DX if t_d > 0 else 0
    sch.dx += DC_DX + GATE_DX
    _add_converter(sch, T_s, t_d, mdl.converter.u_th)
    _add_dc_bus(sch, mdl.dc_bus)
    external = external_speed(mdl)
    _add_im(sch, EXTERNAL_SPEED_J if external else "mechanics.J")
    _add_phase_lines(sch)
    _add_mechanics(sch, mech_signal, inertia=False, external=external)
    y_probes = _add_drive_outputs(sch, block, outputs, y_sources)
    size = (880 + sch.dx, max(520, y_probes + 50))
    return _write_model(path, init, t_stop, T_s, sch, size, outputs)


def simulate(
    path: str | Path,
    t_eval: np.ndarray,
    mdl: Drive,
    ctrl: VectorControlSystem | VHzControlSystem,
) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
    """
    Simulate the PLECS model of an induction machine drive, see `plecs.simulate`.
    The control system `ctrl` defines the monitored signals.
    """
    return simulate_plecs(
        path, t_eval, mdl_outputs=MDL_OUTPUTS, ctrl_outputs=control_block(ctrl).signals
    )
