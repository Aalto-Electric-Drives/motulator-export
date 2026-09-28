"""
Export motulator induction machine drives to PLECS.

This module converts an induction machine drive with current-vector control into a
PLECS Standalone model, in the same way as `motulator_plecs.sm` does for synchronous
machine drives. The control system is a masked subsystem, whose parameters are
those of `InductionMachineInvGammaPars`, `CurrentVectorControllerCfg`, and
`SpeedController`, and contains a C-Script block with the C port in
`c/im_current_vector.c`. The machine is the PLECS squirrel-cage induction machine,
parametrized as the T model equivalent to the inverse-Γ model of motulator.

Currently supported:

- Induction machine with constant parameters (`InductionMachineInvGammaPars`, or
  `InductionMachinePars` with a constant `L_s`) without core losses
- Mechanical system (`MechanicalSystem` without friction), the converter, and the DC
  bus as in `motulator_plecs.sm`
- Current-vector control (`CurrentVectorController`) in the sensorless or sensored
  mode with the default observer gain `k_o`, and a speed controller
  (`SpeedController`) in `VectorControlSystem`

"""

from dataclasses import fields
from pathlib import Path
from typing import Any, cast

import numpy as np
from motulator.common.model._converter import FrequencyConverter
from motulator.drive.control._base import VectorControlSystem
from motulator.drive.control._im_current_vector import (
    CurrentVectorController,
    CurrentVectorControllerCfg,
)
from motulator.drive.model import Drive, InductionMachine
from motulator.drive.utils._parameters import (
    InductionMachineInvGammaPars,
    InductionMachinePars,
)

from motulator_plecs._common import (
    C_DIR,
    C_PARAMS,
    DUTY_RATIO_CODE,
    MACH,
    ControlBlock,
    MaskParam,
    StepSignal,
    _add_control_system,
    _add_converter,
    _add_dc_bus,
    _add_delay,
    _add_pwm,
    _write_model,
    cfg_assignments,
    monitored_code,
    parameter_checks,
)
from motulator_plecs._drive import (
    MACHINE_FRAME,
    MACHINE_PROBES,
    MACHINE_TERMINALS,
    MDL_OUTPUTS,
    SPEED_MASK_PARAMS,
    _add_drive_outputs,
    _add_mechanics,
    _check_supported_plant,
    _check_supported_speed_control,
    mechanics_and_converter_variables,
    speed_controller_code,
    speed_ctrl_values,
)
from motulator_plecs._rpc import simulate_plecs
from motulator_plecs._schematic import _probe, _Schematic, _terminals

# Inputs of the control system and the monitored signals (mask probes)
CTRL_INPUTS = ["w_M_ref", "i_s_abc", "u_dc", "w_M"]
CTRL_OUTPUTS = {
    "Speed (w_M_ref, w_M)": ["w_M_ref", "w_M"],
    "Torque (tau_M_ref, tau_M)": ["tau_M_ref", "tau_M"],
    "Flux linkage (psi_s, psi_R)": ["psi_s", "psi_R"],
    "Current (i_sd_ref, i_sd, i_sq_ref, i_sq)": [
        "i_sd_ref",
        "i_sd",
        "i_sq_ref",
        "i_sq",
    ],
}

TAB_PAR = "Machine model (InductionMachineInvGammaPars)"
TAB_CFG = "Current-vector control (CurrentVectorControllerCfg)"
MASK_PARAMS = [
    MaskParam("n_p", "n_p: Number of pole pairs", TAB_PAR, "", True),
    MaskParam("R_s", "R_s: Stator resistance (Ω)", TAB_PAR, "", True),
    MaskParam("R_R", "R_R: Rotor resistance (Ω)", TAB_PAR, "", True),
    MaskParam("L_sgm", "L_sgm: Leakage inductance (H)", TAB_PAR, "", True),
    MaskParam("L_M", "L_M: Magnetizing inductance (H)", TAB_PAR, "", True),
    MaskParam(
        "psi_s_nom", "psi_s_nom: Nominal stator flux linkage (Vs)", TAB_CFG, "", True
    ),
    MaskParam("i_s_max", "i_s_max: Maximum stator current (A)", TAB_CFG, "", True),
    MaskParam(
        "alpha_c", "alpha_c: Current-control bandwidth (rad/s)", TAB_CFG, "cfg.alpha_c"
    ),
    MaskParam(
        "alpha_i",
        "alpha_i: Integral-action bandwidth (rad/s), [] = alpha_c",
        TAB_CFG,
        "cfg.alpha_i",
    ),
    MaskParam(
        "alpha_o",
        "alpha_o: Speed estimation pole (rad/s), [] = default",
        TAB_CFG,
        "cfg.alpha_o",
    ),
    MaskParam(
        "w_s_nom",
        "w_s_nom: Nominal stator angular frequency (rad/s)",
        TAB_CFG,
        "cfg.w_s_nom",
    ),
    MaskParam("k_u", "k_u: Voltage utilization factor", TAB_CFG, "cfg.k_u"),
    MaskParam(
        "k_fw", "k_fw: Field-weakening gain (1/H), [] = default", TAB_CFG, "cfg.k_fw"
    ),
    MaskParam(
        "J", "J: Inertia (kgm²) for the speed observer, [] = not used", TAB_CFG, "cfg.J"
    ),
    MaskParam(
        "sensorless",
        "sensorless: Sensorless mode (1) or sensored mode (0)",
        TAB_CFG,
        "cfg.sensorless",
    ),
    MaskParam("T_s", "T_s: Sampling period (s)", TAB_CFG, "cfg.T_s", True),
    *SPEED_MASK_PARAMS,
]


def _check_supported(mdl: Drive, ctrl: VectorControlSystem) -> None:
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
    _check_supported_plant(mdl)
    cvc = ctrl.vector_ctrl
    if not isinstance(cvc, CurrentVectorController):
        raise NotImplementedError("Only CurrentVectorController supported")
    if cvc.cfg.k_o is not None:
        raise NotImplementedError("Only the default observer gain k_o supported")
    if not isinstance(cvc.reference_gen.par, InductionMachineInvGammaPars):
        raise NotImplementedError("Only InductionMachineInvGammaPars supported")
    _check_supported_speed_control(ctrl)


def export_mask_values(
    ctrl: VectorControlSystem, speed_ctrl_args: dict[str, float]
) -> dict[str, Any]:
    """Get the mask parameter values of the control system in the motulator API."""
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


def _control_cscript_code() -> dict[str, str]:
    """Code sections of the C-Script block of the control system."""
    declarations = (
        "/* Generated by motulator_plecs.im. The control algorithms are in the\n"
        " * included C files. The parameters come from the mask of the subsystem. */\n"
        f'#include "{C_DIR}/common.c"\n'
        f'#include "{C_DIR}/im_current_vector.c"\n'
        "\n" + C_PARAMS + "\n"
        "static IMVectorControlSystem ctrl;\n"
    )
    i = {m.variable: k for k, m in enumerate(MASK_PARAMS)}
    start = (
        "/* Check the parameters of the mask */\n"
        + parameter_checks(MASK_PARAMS)
        + "\n"
        "/* Machine model parameters (InductionMachineInvGammaPars) */\n"
        "InductionMachineInvGammaPars par = {\n"
        f"    P({i['n_p']}, 0), P({i['R_s']}, 0), P({i['R_R']}, 0), "
        f"P({i['L_sgm']}, 0), P({i['L_M']}, 0)}};\n"
        "\n"
        "/* Current-vector controller configuration (CurrentVectorControllerCfg),\n"
        " * the defaults are used for empty parameters */\n"
        "IMCurrentVectorControllerCfg cfg = im_current_vector_controller_cfg(\n"
        f"    P({i['psi_s_nom']}, 0), P({i['i_s_max']}, 0));\n"
        + cfg_assignments(MASK_PARAMS)
        + "\n"
        + speed_controller_code(i)
        + "\n"
        "im_vector_control_system_init(&ctrl, par, &cfg, speed_ctrl);\n"
    )
    monitored = {
        "w_M_ref": "ctrl.ref.w_M",
        "w_M": "ctrl.fbk.w_M",
        "tau_M_ref": "ctrl.ref.tau_M",
        "tau_M": "ctrl.fbk.tau_M",
        "psi_s": "cabs(ctrl.fbk.psi_s)",
        "psi_R": "cabs(ctrl.fbk.psi_R)",
        "i_sd_ref": "creal(rot * ctrl.ref.i_s)",
        "i_sd": "creal(rot * ctrl.fbk.i_s)",
        "i_sq_ref": "cimag(rot * ctrl.ref.i_s)",
        "i_sq": "cimag(rot * ctrl.fbk.i_s)",
    }
    output = (
        "/* Measurements */\n"
        "double w_M_ref = InputSignal(0, 0);\n"
        "double i_s_abc[3] = {InputSignal(1, 0), InputSignal(1, 1),\n"
        "                     InputSignal(1, 2)};\n"
        "IMMeasurements meas = {abc2complex(i_s_abc), InputSignal(2, 0),\n"
        "                       InputSignal(3, 0)};\n"
        "\n"
        "im_vector_control_system_compute_output(&ctrl, &meas, w_M_ref);\n"
        "\n"
        + DUTY_RATIO_CODE
        + monitored_code(
            CTRL_OUTPUTS,
            monitored,
            "Monitored signals, the currents in estimated rotor flux coordinates",
            "double complex rot = cexp(-I * carg(ctrl.fbk.psi_R));\n",
        )
    )
    update = "im_vector_control_system_update(&ctrl);\n"
    return {
        "Declarations": declarations,
        "StartFcn": start,
        "OutputFcn": output,
        "UpdateFcn": update,
    }


CVC_BLOCK = ControlBlock(
    name="Current-vector control",
    mask_type="Current-vector control (motulator)",
    description=(
        "Speed control of an induction machine drive with current-vector control. "
        "The parameters correspond to the motulator API: "
        "InductionMachineInvGammaPars, CurrentVectorControllerCfg, and "
        "SpeedController. Empty parameters ([]) correspond to None, i.e., the "
        "defaults of motulator."
    ),
    mask_params=MASK_PARAMS,
    inputs=CTRL_INPUTS,
    input_widths=[1, 3, 1, 1],
    outputs=CTRL_OUTPUTS,
    code=_control_cscript_code,
)


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
    ctrl: VectorControlSystem,
    w_M_ref: StepSignal,
    tau_L: StepSignal,
    t_stop: float,
    speed_ctrl_args: dict[str, float],
    outputs: bool = False,
) -> Path:
    """Write a PLECS model of the induction machine drive, see `sm.write_model`."""
    path = Path(path)
    _check_supported(mdl, ctrl)
    values = export_mask_values(ctrl, speed_ctrl_args)
    sch = _Schematic()
    sources: list[tuple[str, StepSignal | float | str]] = [
        ("w_M_ref", w_M_ref),
        ("i_s_abc", _probe("Machine", MACHINE_PROBES[:1])),
        ("u_dc meas.", _probe("u_dc", ["Measured voltage"])),
        ("w_M", _probe("Machine", MACHINE_PROBES[1:2])),
    ]
    _add_control_system(sch, CVC_BLOCK, sources, values)
    _add_delay(sch, values["T_s"], CVC_BLOCK)
    _add_pwm(sch, values["T_s"])
    # The diode bridge and its grid need space between the PWM and the DC bus
    sch.dx = 320 if isinstance(mdl.converter, FrequencyConverter) else 0
    _add_converter(sch)
    _add_dc_bus(sch, mdl.converter)
    _add_im(sch)
    for k in range(3):
        sch.wire(("Converter", k + 1), ("Machine", k + 1))
    _add_mechanics(sch, tau_L, inertia=False)
    _add_drive_outputs(sch, CVC_BLOCK, outputs)
    size = (1000 + sch.dx, 560)
    variables = _plant_variables(mdl)
    return _write_model(
        path, CVC_BLOCK, variables, t_stop, values["T_s"], sch, size, outputs
    )


def simulate(
    path: str | Path, t_eval: np.ndarray
) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
    """Simulate the PLECS model of an induction machine drive, see `sm.simulate`."""
    return simulate_plecs(
        path, t_eval, mdl_outputs=MDL_OUTPUTS, ctrl_outputs=CVC_BLOCK.signals
    )
