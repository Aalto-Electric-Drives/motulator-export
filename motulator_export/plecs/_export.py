"""Export of motulator systems to PLECS, for all the supported system types."""

from pathlib import Path

import numpy as np
from motulator.drive.control._base import VectorControlSystem, VHzControlSystem
from motulator.drive.model import Drive
from motulator.grid.control._base import GridConverterControlSystem
from motulator.grid.model import GridConverterSystem

from motulator_export.plecs import _grid, _im, _sm
from motulator_export.plecs._common import system_type

_MODULES = {"sm": _sm, "im": _im, "grid": _grid}


def write_model(
    path: str | Path,
    mdl: Drive | GridConverterSystem,
    ctrl: VectorControlSystem | VHzControlSystem | GridConverterControlSystem,
    t_stop: float,
    outputs: bool = False,
) -> Path:
    """
    Write a PLECS model of the system.

    The references of the control system (e.g., `set_speed_ref`), the external load
    torque (`set_external_load_torque`), and the gate enable (`set_enable`) are read
    from `ctrl` and `mdl`, each a constant or a `StepSignal`. The model starts from
    the state of `mdl`, e.g., its initial speed. A system that is not supported
    raises `NotImplementedError`, and an invalid argument `ValueError`, before any
    file is written.

    Parameters
    ----------
    path : str | Path
        Path of the model file (.plecs). The model refers to the C sources of the
        package by a path relative to the model file.
    mdl : Drive | GridConverterSystem
        Continuous-time system model.
    ctrl : VectorControlSystem | VHzControlSystem | GridConverterControlSystem
        Discrete-time control system.
    t_stop : float
        Simulation stop time (s).
    outputs : bool, optional
        Add the output ports "mdl" and "ctrl" for `simulate`, defaults to False.

    Returns
    -------
    Path
        Path of the written model file.

    """
    return _MODULES[system_type(mdl, ctrl)].write_model(
        path, mdl, ctrl, t_stop, outputs
    )


def simulate(
    path: str | Path,
    t_eval: np.ndarray,
    mdl: Drive | GridConverterSystem,
    ctrl: VectorControlSystem | VHzControlSystem | GridConverterControlSystem,
) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
    """
    Simulate the PLECS model of the system via the RPC interface of PLECS Standalone.

    The model must have the output ports (`write_model` with ``outputs=True``).

    Parameters
    ----------
    path : str | Path
        Path of the model file.
    t_eval : ndarray
        Output times (s).
    mdl : Drive | GridConverterSystem
        Continuous-time system model of the written model.
    ctrl : VectorControlSystem | VHzControlSystem | GridConverterControlSystem
        Discrete-time control system of the written model, which defines the
        monitored signals.

    Returns
    -------
    mdl : dict[str, ndarray]
        Signals of the system model (the output port "mdl") and the time "t".
    ctrl : dict[str, ndarray]
        Monitored signals of the control system (the output port "ctrl") and the time
        "t".

    """
    return _MODULES[system_type(mdl, ctrl)].simulate(path, t_eval, mdl, ctrl)
