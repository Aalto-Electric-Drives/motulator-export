"""
Export induction machine drives to Simulink.

This module writes a Simulink model of a motulator induction machine drive with
current-vector control or observer-based V/Hz control, in the same way as
`motulator_export.simulink.sm` does for synchronous machine drives. The machine
is modeled with the inverse-Γ model in stator coordinates, which is equivalent to
the Γ model of `InductionMachine` in motulator.

Currently supported: the induction machines and control systems of
`motulator_export.plecs.im`, with the converter of `motulator_export.simulink.sm`.

"""

from pathlib import Path

import numpy as np
from motulator.drive.control._base import VectorControlSystem, VHzControlSystem
from motulator.drive.model import Drive

from motulator_export.plecs import _im_control, _im_vhz_control, im
from motulator_export.plecs._common import StepSignal
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
    w_M_ref: StepSignal,
    tau_L: StepSignal,
    t_stop: float,
    speed_ctrl_args: dict[str, float] | None = None,
    enable: StepSignal | float = 1.0,
) -> Path:
    """
    Write the MATLAB scripts that build the Simulink model, see `sm.write_model`.

    The control system is current-vector control (`VectorControlSystem`, whose speed
    controller is given by `speed_ctrl_args`) or observer-based V/Hz control
    (`VHzControlSystem`, without `speed_ctrl_args`), see `plecs.im.write_model`.

    """
    im._check_supported(mdl, ctrl)
    check_supported_converter(mdl)
    path = Path(path)
    variables = im._plant_variables(mdl)
    if isinstance(ctrl, VHzControlSystem):
        vhz_values = im.export_vhz_values(ctrl)
        script = write_init(
            path,
            lambda comment: _im_vhz_control.init_script(variables, vhz_values, comment),
        )
    else:
        if speed_ctrl_args is None:
            raise ValueError("speed_ctrl_args is needed in current-vector control")
        values = im.export_values(ctrl, speed_ctrl_args)
        script = write_init(
            path, lambda comment: _im_control.init_script(variables, values, comment)
        )
    block = im.control_block(ctrl)
    return write_drive_model(
        path, block, script, "im", w_M_ref, tau_L, t_stop, enable=enable
    )


def simulate(
    path: str | Path,
    t_eval: np.ndarray,
    build: bool = True,
    ctrl: VectorControlSystem | VHzControlSystem | None = None,
) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
    """
    Simulate the Simulink model, see `sm.simulate`. The control system `ctrl` defines
    the monitored signals, by default those of current-vector control.
    """
    block = im.CVC_BLOCK if ctrl is None else im.control_block(ctrl)
    return simulate_drive(path, t_eval, block, build)
