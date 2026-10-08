"""
Export motulator induction machine drives to PLECS.

This module converts an induction machine drive with current-vector control or
observer-based V/Hz control into a PLECS Standalone model, in the same way as
`motulator_export.plecs.sm` does for synchronous machine drives. In the control
system, each class of motulator is a block with a mask of its arguments (see
`_im_control` and `_im_vhz_control`), with the C port in `c/im_current_vector.c`
and `c/im_flux_vector.c`. The parameters are defined in the initialization commands
of the PLECS model as the structs par (`InductionMachineInvGammaPars`), cfg
(`CurrentVectorControllerCfg` or `ObserverBasedVHzControllerCfg`), speed_ctrl
(`SpeedController`) or slew_rate (of `VHzControlSystem`), and pwm (`PWM`).
The machine is the PLECS squirrel-cage induction machine, parametrized as the T
model equivalent to the inverse-Γ model of motulator.

Currently supported:

- Induction machine with constant parameters (`InductionMachineInvGammaPars`, or
  `InductionMachinePars` with a constant `L_s`) without core losses
- Mechanical system (`MechanicalSystem` without friction), the converter, and the DC
  bus as in `motulator_export.plecs.sm`
- Current-vector control (`CurrentVectorController`) in the sensorless or sensored
  mode with the default observer gain `k_o`, and a speed controller
  (`SpeedController`) in `VectorControlSystem`
- Observer-based V/Hz control (`ObserverBasedVHzController`) with the default
  observer gain `k_o` in `VHzControlSystem`, including pure open-loop V/Hz control
  as its special case (`L_M = inf` in the machine model of the control system)
- The MPE or MME overmodulation of the PWM (`overmodulation`, MME by default in V/Hz
  control) and the minimum duty ratio (`d_min`)
- Dead time of the converter (`t_d` with `sign=np.sign`), modeled with the Blanking
  Time block and the IGBT converter of PLECS, and its compensation in the PWM
  (`PWM(d_err=lambda i, d: dead_time_error(i, d, t_d, T_s, sign))` with
  `sign=np.sign` or the smooth `sign=lambda i: np.tanh(i/i_0)`)

"""

from dataclasses import fields
from pathlib import Path
from typing import Any, cast

import numpy as np
from motulator.common.model._converter import FrequencyConverter
from motulator.common.model._pwm import CarrierComparison
from motulator.drive.control._base import VectorControlSystem, VHzControlSystem
from motulator.drive.control._im_current_vector import (
    CurrentVectorController,
    CurrentVectorControllerCfg,
)
from motulator.drive.control._im_flux_vector import ObserverBasedVHzController
from motulator.drive.model import Drive, InductionMachine
from motulator.drive.utils._parameters import (
    InductionMachineInvGammaPars,
    InductionMachinePars,
)

from motulator_export.plecs._common import (
    BLANKING_DX,
    DC_DX,
    ENABLE,
    MACH,
    ControlBlock,
    StepSignal,
    _add_blanking_time,
    _add_control_system,
    _add_converter,
    _add_dc_bus,
    _add_delay,
    _add_pwm,
    _check_supported_pwm,
    _write_model,
)
from motulator_export.plecs._control import INIT_COMMENT, VALUES, pwm_values
from motulator_export.plecs._drive import (
    MACHINE_FRAME,
    MACHINE_PROBES,
    MACHINE_TERMINALS,
    MDL_OUTPUTS,
    _add_drive_outputs,
    _add_mechanics,
    _check_supported_plant,
    _check_supported_speed_control,
    mechanics_and_converter_variables,
    speed_ctrl_values,
)
from motulator_export.plecs._im_control import CVC_BLOCK, init_script
from motulator_export.plecs._im_vhz_control import VHZ_BLOCK
from motulator_export.plecs._im_vhz_control import init_script as vhz_init_script
from motulator_export.plecs._rpc import simulate_plecs
from motulator_export.plecs._schematic import _probe, _Schematic, _terminals


def _check_supported(mdl: Drive, ctrl: VectorControlSystem | VHzControlSystem) -> None:
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
        _check_supported_vhz(ctrl)
        return
    cvc = ctrl.vector_ctrl
    if not isinstance(cvc, CurrentVectorController):
        raise NotImplementedError("Only CurrentVectorController supported")
    if cvc.cfg.k_o is not None:
        raise NotImplementedError("Only the default observer gain k_o supported")
    if cvc.cfg.discrete:
        raise NotImplementedError("Direct discrete-time current control not supported")
    if not isinstance(cvc.reference_gen.par, InductionMachineInvGammaPars):
        raise NotImplementedError("Only InductionMachineInvGammaPars supported")
    _check_supported_speed_control(ctrl, dead_time=True)


def _check_supported_vhz(ctrl: VHzControlSystem) -> None:
    """Raise an error if the V/Hz control system is not supported."""
    vhz = ctrl.vhz_ctrl
    if not isinstance(vhz, ObserverBasedVHzController):
        raise NotImplementedError("Only ObserverBasedVHzController supported")
    if vhz.cfg.k_o is not None:
        raise NotImplementedError("Only the default observer gain k_o supported")
    if not isinstance(vhz.reference_gen.par, InductionMachineInvGammaPars):
        raise NotImplementedError("Only InductionMachineInvGammaPars supported")
    if type(ctrl).modulate is not VHzControlSystem.modulate:
        raise NotImplementedError("Overridden modulate method not supported")
    _check_supported_pwm(ctrl.pwm, d_err=True)


def control_block(ctrl: VectorControlSystem | VHzControlSystem) -> ControlBlock:
    """Control-system block of current-vector control or V/Hz control."""
    return VHZ_BLOCK if isinstance(ctrl, VHzControlSystem) else CVC_BLOCK


def export_vhz_values(ctrl: VHzControlSystem) -> dict[str, Any]:
    """
    Get the parameter values of the V/Hz control system in the motulator API (see
    `_im_vhz_control.init_script`).
    """
    vhz = cast(ObserverBasedVHzController, ctrl.vhz_ctrl)
    cfg = vhz.cfg
    par = cast(InductionMachineInvGammaPars, vhz.reference_gen.par)
    return {
        "n_p": par.n_p,
        "R_s": par.R_s,
        "R_R": par.R_R,
        "L_sgm": par.L_sgm,
        "L_M": par.L_M,
        "psi_s_nom": cfg.psi_s_nom,
        "i_s_max": cfg.i_s_max,
        "alpha_psi": cfg.alpha_psi,
        "alpha_tau": cfg.alpha_tau,
        "alpha_f": cfg.alpha_f,
        "k_u": cfg.k_u,
        "k_b": cfg.k_b,
        "T_s": cfg.T_s,
        "slew_rate": ctrl.rate_limiter.rate_limit,
        **pwm_values(ctrl.pwm, cfg.T_s),
    }


def export_values(
    ctrl: VectorControlSystem, speed_ctrl_args: dict[str, float]
) -> dict[str, Any]:
    """
    Get the parameter values of the control system in the motulator API, None for
    the defaults (see `_im_control.init_script`).
    """
    cvc = cast(CurrentVectorController, ctrl.vector_ctrl)
    cfg, par = cvc.cfg, cast(InductionMachineInvGammaPars, cvc.reference_gen.par)
    # alpha_o is resolved in CurrentVectorControllerCfg.__post_init__
    default = CurrentVectorControllerCfg(
        **{
            f.name: getattr(cfg, f.name)
            for f in fields(cfg)
            if f.name not in ("alpha_o",) and f.init
        }
    ).alpha_o
    return {
        "n_p": par.n_p,
        "R_s": par.R_s,
        "R_R": par.R_R,
        "L_sgm": par.L_sgm,
        "L_M": par.L_M,
        "psi_s_nom": cfg.psi_s_nom,
        "i_s_max": cfg.i_s_max,
        "alpha_c": cfg.alpha_c,
        "alpha_i": cfg.alpha_i,
        "alpha_o": None if cfg.alpha_o == default else cfg.alpha_o,
        "w_s_nom": cfg.w_s_nom,
        "k_u": cfg.k_u,
        "k_fw": cfg.k_fw or None,
        "J": cfg.J,
        "sensorless": int(cfg.sensorless),
        "T_s": cfg.T_s,
        **speed_ctrl_values(ctrl, speed_ctrl_args),
        **pwm_values(ctrl.pwm, cfg.T_s),
    }


def _plant_variables(mdl: Drive) -> list[tuple[str, Any]]:
    """Workspace variables of the system model (inverse-Γ machine parameters)."""
    par = InductionMachineInvGammaPars.from_gamma_pars(
        cast(InductionMachinePars, mdl.machine.par)
    )
    variables: list[tuple[str, Any]] = [
        ("machine.n_p", par.n_p),
        ("machine.R_s", par.R_s),
        ("machine.R_R", par.R_R),
        ("machine.L_sgm", par.L_sgm),
        ("machine.L_M", par.L_M),
    ]
    return variables + mechanics_and_converter_variables(mdl)


def _add_im(sch: _Schematic) -> None:
    """
    Add the PLECS squirrel-cage induction machine (including the inertia).

    The inverse-Γ model of motulator corresponds to the T model of PLECS with zero
    rotor leakage inductance.

    """
    machine = {
        "Rs": "machine.R_s",
        "Lls": "machine.L_sgm",
        "Rr": "machine.R_R",
        "Llr": "0",
        "Lm": "machine.L_M",
        "J": "mechanics.J",
        "F": "0",
        "p": "machine.n_p",
        "wm0": "0",
        "thm0": "0",
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
    w_M_ref: StepSignal,
    tau_L: StepSignal,
    t_stop: float,
    speed_ctrl_args: dict[str, float] | None = None,
    outputs: bool = False,
    enable: StepSignal | float = 1.0,
) -> Path:
    """
    Write a PLECS model of the induction machine drive, see `sm.write_model`.

    The control system is current-vector control (`VectorControlSystem`, whose speed
    controller is given by `speed_ctrl_args`) or observer-based V/Hz control
    (`VHzControlSystem`, without `speed_ctrl_args`), whose inputs are `enable`, the
    speed reference, the phase currents, and the DC-bus voltage.

    """
    path = Path(path)
    _check_supported(mdl, ctrl)
    block = control_block(ctrl)
    variables = _plant_variables(mdl)
    if isinstance(ctrl, VHzControlSystem):
        init = vhz_init_script(variables, export_vhz_values(ctrl), INIT_COMMENT)
    else:
        if speed_ctrl_args is None:
            raise ValueError("speed_ctrl_args is needed in current-vector control")
        values = export_values(ctrl, speed_ctrl_args)
        init = init_script(variables, values, INIT_COMMENT)
    T_s = VALUES["T_s"]  # The model refers to cfg.T_s
    sch = _Schematic()
    sources: list[tuple[str, StepSignal | float | str]] = [
        (ENABLE, enable),
        ("w_M_ref", w_M_ref),
        ("i_s_abc", _probe("Machine", MACHINE_PROBES[:1])),
        ("u_dc meas.", _probe("u_dc", ["Measured voltage"])),
    ]
    if block is CVC_BLOCK:
        sources += [("w_M", _probe("Machine", MACHINE_PROBES[1:2]))]
    _add_control_system(sch, block, sources, VALUES, init)
    _add_delay(sch, T_s, block)
    _add_pwm(sch, T_s)
    t_d = cast(CarrierComparison, mdl.pwm).t_d
    if t_d > 0:
        _add_blanking_time(sch)
    # The blanking time, the diode bridge, and its grid need space between the PWM and
    # the DC bus
    sch.dx = 320 if isinstance(mdl.converter, FrequencyConverter) else 0
    sch.dx += BLANKING_DX if t_d > 0 else 0
    sch.dx += DC_DX
    _add_converter(sch, t_d)
    _add_dc_bus(sch, mdl.converter)
    _add_im(sch)
    for k in range(3):
        sch.wire(("Converter", k + 1), ("Machine", k + 1))
    _add_mechanics(sch, tau_L, inertia=False)
    _add_drive_outputs(sch, block, outputs)
    size = (880 + sch.dx, 520)
    return _write_model(path, init, t_stop, T_s, sch, size, outputs)


def simulate(
    path: str | Path,
    t_eval: np.ndarray,
    ctrl: VectorControlSystem | VHzControlSystem | None = None,
) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
    """
    Simulate the PLECS model of an induction machine drive, see `sm.simulate`. The
    control system `ctrl` defines the monitored signals, by default those of
    current-vector control.
    """
    block = CVC_BLOCK if ctrl is None else control_block(ctrl)
    return simulate_plecs(
        path, t_eval, mdl_outputs=MDL_OUTPUTS, ctrl_outputs=block.signals
    )
