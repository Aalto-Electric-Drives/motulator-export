"""
Export induction machine drives to Simulink.

This module writes a Simulink model of a motulator induction machine drive with
current-vector control, in the same way as `motulator_plecs.simulink.sm` does for
synchronous machine drives. The machine is modeled with the inverse-Γ model in
stator coordinates, which is equivalent to the Γ model of `InductionMachine` in
motulator.

Currently supported: the induction machines and control systems of
`motulator_plecs.im`, with the converter of `motulator_plecs.simulink.sm`.

"""

from pathlib import Path

import numpy as np
from motulator.drive.control._base import VectorControlSystem
from motulator.drive.model import Drive

from motulator_plecs import im
from motulator_plecs._common import StepSignal
from motulator_plecs.simulink._drive import (
    check_supported_converter,
    simulate_drive,
    write_drive_model,
)

BLOCK = im.CVC_BLOCK


def write_model(
    path: str | Path,
    mdl: Drive,
    ctrl: VectorControlSystem,
    w_M_ref: StepSignal,
    tau_L: StepSignal,
    t_stop: float,
    speed_ctrl_args: dict[str, float],
) -> Path:
    """Write a MATLAB script that builds the Simulink model, see `sm.write_model`."""
    im._check_supported(mdl, ctrl)
    check_supported_converter(mdl)
    values = im.export_mask_values(ctrl, speed_ctrl_args)
    variables = im._plant_variables(mdl)
    return write_drive_model(
        path, BLOCK, values, variables, "im", w_M_ref, tau_L, t_stop
    )


def simulate(
    path: str | Path, t_eval: np.ndarray, build: bool = True
) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
    """Simulate the Simulink model, see `sm.simulate`."""
    return simulate_drive(path, t_eval, BLOCK, build)
