"""
Export induction machine drives to Simulink.

This module writes a Simulink model of a motulator induction machine drive with
current-vector control or observer-based V/Hz control, in the same way as
`motulator_export.simulink._sm` does for synchronous machine drives. The machine
is modeled with the inverse-Γ model in stator coordinates, which is equivalent to
the Γ model of `InductionMachine` in motulator.

Currently supported: the induction machines and control systems of
`motulator_export.plecs._im`, with the converter of `motulator_export.simulink._sm`.

"""

from pathlib import Path

import numpy as np
from motulator.drive.control._base import VectorControlSystem, VHzControlSystem
from motulator.drive.model import Drive

from motulator_export.plecs import _im, _im_control, _im_vhz_control
from motulator_export.plecs._drive import drive_references
from motulator_export.simulink._common import write_init
from motulator_export.simulink._drive import (
    check_supported_converter,
    simulate_drive,
    write_drive_model,
)


def write_model(
    path: str | Path,
    mdl: Drive,
    ctrl: VectorControlSystem | VHzControlSystem,
    t_stop: float,
    code: str = "c",
) -> Path:
    """
    Write the MATLAB scripts that build the Simulink model of the induction machine
    drive, see `simulink.write_model`. The control system is current-vector control
    (`VectorControlSystem`) or observer-based V/Hz control (`VHzControlSystem`).
    """
    _im._check_supported(mdl, ctrl)
    check_supported_converter(mdl, ctrl)
    path = Path(path)
    variables = _im._plant_variables(mdl)
    # The references are checked before anything is written
    if isinstance(ctrl, VHzControlSystem):
        vhz_values = _im.export_vhz_values(ctrl)
        w_M_ref, tau_L = drive_references(mdl, ctrl, vhz_values["T_s"])
        script = write_init(
            path,
            lambda comment: _im_vhz_control.init_script(variables, vhz_values, comment),
        )
    else:
        values = _im.export_values(ctrl)
        w_M_ref, tau_L = drive_references(mdl, ctrl, values["T_s"])
        script = write_init(
            path, lambda comment: _im_control.init_script(variables, values, comment)
        )
    block = _im.control_block(ctrl)
    return write_drive_model(
        path, block, script, "im", w_M_ref, tau_L, t_stop, code=code
    )


def simulate(
    path: str | Path,
    t_eval: np.ndarray,
    mdl: Drive,
    ctrl: VectorControlSystem | VHzControlSystem,
    build: bool = True,
) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
    """
    Simulate the Simulink model of an induction machine drive, see
    `simulink.simulate`. The control system `ctrl` defines the monitored signals.
    """
    return simulate_drive(path, t_eval, _im.control_block(ctrl), build)
