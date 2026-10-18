"""
Export synchronous machine drives to Simulink.

This module writes a Simulink model of a motulator synchronous machine drive, in the
same way as `motulator_export.plecs.sm` writes a PLECS model. In the control system,
each class of motulator is a block with a mask of its arguments, and the blocks of
the classes are the C-Script code of the PLECS model wrapped in S-functions (`code`
"c", the default), or MATLAB Function blocks running the MATLAB port of motulator
(`code` "matlab"). The system model is built from basic Simulink blocks, so no
toolboxes are needed.

The model is written as MATLAB scripts (and the sources of the S-functions or the
MATLAB functions): the script `build_<model>.m` builds the model with
`build_drive.m` of this directory, and the script `init_<model>.m` defines the
parameters (the system model, and the structs par, cfg, speed_ctrl, and pwm of the
control system, as in the motulator API), which the model runs after loading and at
the start of each simulation. The C port uses the C99 complex type, so the
S-functions need gcc (Linux), Xcode clang (macOS), or MinGW-w64 (Windows), not MSVC.
The MATLAB Function blocks need no C compiler.

Currently supported:

- Synchronous machine: `SynchronousMachinePars`, modeled in rotor coordinates with
  the stator flux linkage as the state, as in motulator, or
  `SpatialSaturatedSynchronousMachinePars` with a GradNet current map with spatial
  harmonics, modeled by the C-Script code of the PLECS model in an S-function (or by
  its MATLAB port in a MATLAB Function block)
- Mechanical system (`MechanicalSystem` without friction)
- Converter: carrier comparison (`pwm=True` in `Drive`) with a stiff DC bus
  (`VoltageSourceConverter`). The carrier comparison uses the zero-crossing
  detection of Simulink. The computational delay of one sampling period is a Unit
  Delay block.
- Flux-vector control (`FluxVectorController`) with `SynchronousMachinePars` or
  `SaturatedSynchronousMachinePars` with a GradNet flux map in the sensorless or
  sensored mode, and a speed controller (`SpeedController`) in
  `VectorControlSystem`, with the dead-time compensation in the PWM

The model can be simulated from Python via the MATLAB Engine API for Python
(`simulate`).

"""

from pathlib import Path
from typing import cast

import numpy as np
from motulator.drive.control._base import VectorControlSystem
from motulator.drive.model import Drive
from motulator.drive.utils._parameters import SpatialSaturatedSynchronousMachinePars

from motulator_export.plecs import _sm_control, sm
from motulator_export.plecs._common import StepSignal
from motulator_export.simulink._common import write_init
from motulator_export.simulink._drive import (
    check_supported_converter,
    simulate_drive,
    write_drive_model,
)
from motulator_export.simulink._mfunction import MFunction, gradnet_machine_mfunction
from motulator_export.simulink._sfunction import SFunction, gradnet_machine_sfunction

BLOCK = sm.FVC_BLOCK


def _check_supported(mdl: Drive, ctrl: VectorControlSystem) -> None:
    """Raise an error if the drive system is not supported."""
    sm._check_supported(mdl, ctrl)  # The PLECS export supports a superset
    check_supported_converter(mdl, ctrl)


def write_model(
    path: str | Path,
    mdl: Drive,
    ctrl: VectorControlSystem,
    w_M_ref: StepSignal,
    tau_L: StepSignal,
    t_stop: float,
    speed_ctrl_args: dict[str, float],
    code: str = "c",
) -> Path:
    """
    Write the MATLAB scripts that build the Simulink model of the drive system.

    Running the script `build_<model>.m` in MATLAB compiles the S-functions and
    saves the model (and the compiled S-functions) in the folder of the script.
    With `code` "matlab", the blocks of the control system are MATLAB Function
    blocks, whose functions are copied into the model, which needs no compiler.

    Parameters
    ----------
    path : str | Path
        Path of the model (.slx). The scripts `build_<model>.m` and
        `init_<model>.m` are written in the same folder, with the sources of the
        S-functions (or of the MATLAB functions).
    mdl : Drive
        Continuous-time system model.
    ctrl : VectorControlSystem
        Discrete-time control system. The control system supports the gate blocking
        (`set_enable`) by its input `enable`, but the system model does not, so the
        gates must not be blocked (the input is 1).
    w_M_ref : StepSignal
        Speed reference (mechanical rad/s).
    tau_L : StepSignal
        External load torque (Nm).
    t_stop : float
        Simulation stop time (s).
    speed_ctrl_args : dict[str, float]
        Arguments of `SpeedController` used in `ctrl`.
    code : str, optional
        Code of the blocks of the control system: "c" (the C port in S-functions,
        the default) or "matlab" (the MATLAB port in MATLAB Function blocks).

    Returns
    -------
    Path
        Path of the build script.

    """
    _check_supported(mdl, ctrl)
    path = Path(path)
    values, flux_map = sm.export_values(ctrl, speed_ctrl_args)
    variables = sm.export_plant_variables(mdl)
    functions: list[SFunction | MFunction] = []
    if sm._has_gradnet_plant(mdl):
        machine = "gn"
        if code == "matlab":
            # The initial state of the integrator of the stator flux linkage
            par = cast(SpatialSaturatedSynchronousMachinePars, mdl.machine.par)
            variables.append(("machine.psi_f", par.psi_f))
            functions.append(gradnet_machine_mfunction())
        else:
            functions.append(gradnet_machine_sfunction())
    else:
        machine = "sm"
    if flux_map is not None:
        variables += [(f"est_flux_map.{f}", flux_map[f]) for f in sm.GRADNET_FIELDS]
    script = write_init(
        path, lambda comment: _sm_control.init_script(variables, values, comment)
    )
    return write_drive_model(
        path, BLOCK, script, machine, w_M_ref, tau_L, t_stop, functions, code
    )


def simulate(
    path: str | Path, t_eval: np.ndarray, build: bool = True
) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
    """
    Simulate the Simulink model via the MATLAB Engine API for Python.

    Parameters
    ----------
    path : str | Path
        Path of the model (.slx).
    t_eval : ndarray
        Output times (s).
    build : bool, optional
        Run the build script `build_<model>.m` first, defaults to True.

    Returns
    -------
    mdl : dict[str, ndarray]
        Machine signals (`MDL_OUTPUTS`) and the time "t".
    ctrl : dict[str, ndarray]
        Monitored signals of the control system and the time "t".

    """
    return simulate_drive(path, t_eval, BLOCK, build)
