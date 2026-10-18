"""Export of motulator systems to Simulink, for all the supported system types."""

from pathlib import Path
from typing import Any

import numpy as np

from motulator_export.plecs._common import system_type
from motulator_export.simulink import _grid, _im, _sm
from motulator_export.simulink._common import CODES

_MODULES = {"sm": _sm, "im": _im, "grid": _grid}


def write_model(
    path: str | Path, mdl: Any, ctrl: Any, t_stop: float, code: str = "c"
) -> Path:
    """
    Write the MATLAB scripts that build the Simulink model of the system.

    Running the script `build_<model>.m` in MATLAB compiles the S-functions of the
    blocks of the control system and saves the model in the folder of the script.
    The model runs the script `init_<model>.m`, which defines the parameters in the
    base workspace, after loading and at the start of each simulation. With `code`
    "matlab", the blocks are MATLAB Function blocks running the MATLAB port of the
    same code, whose functions are copied into the model, which needs no C compiler.

    The system model, the control system, and their references are those of
    `plecs.write_model`. The system model of Simulink does not block the gates, so
    the gate enable of the control system must be 1 (the input `enable` of the
    exported control system is 1).

    Parameters
    ----------
    path : str | Path
        Path of the model (.slx). The scripts `build_<model>.m` and `init_<model>.m`
        are written in the same folder, with the sources of the S-functions
        (`sfun_*.c`) or the MATLAB functions (`mfun_*.m`) of the blocks.
    mdl : Drive | GridConverterSystem
        Continuous-time system model.
    ctrl : VectorControlSystem | VHzControlSystem | GridConverterControlSystem
        Discrete-time control system.
    t_stop : float
        Simulation stop time (s).
    code : str, optional
        Code of the blocks of the control system: "c" (the C port in S-functions,
        the default) or "matlab" (the MATLAB port in MATLAB Function blocks).

    Returns
    -------
    Path
        Path of the build script.

    """
    if code not in CODES:  # Checked before anything is written
        raise ValueError(f"Unknown code {code}, not in {CODES}")
    return _MODULES[system_type(mdl, ctrl)].write_model(path, mdl, ctrl, t_stop, code)


def simulate(
    path: str | Path, t_eval: np.ndarray, mdl: Any, ctrl: Any, build: bool = True
) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
    """
    Simulate the Simulink model of the system via the MATLAB Engine API for Python.

    Parameters
    ----------
    path : str | Path
        Path of the model (.slx), next to its scripts.
    t_eval : ndarray
        Output times (s).
    mdl : Drive | GridConverterSystem
        Continuous-time system model of the written model.
    ctrl : VectorControlSystem | VHzControlSystem | GridConverterControlSystem
        Discrete-time control system of the written model, which defines the
        monitored signals.
    build : bool, optional
        Run the build script `build_<model>.m` first, defaults to True.

    Returns
    -------
    mdl : dict[str, ndarray]
        Signals of the system model (the output port "mdl") and the time "t".
    ctrl : dict[str, ndarray]
        Monitored signals of the control system (the output port "ctrl") and the time
        "t".

    """
    return _MODULES[system_type(mdl, ctrl)].simulate(path, t_eval, mdl, ctrl, build)
