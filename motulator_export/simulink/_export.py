"""Export of motulator systems to Simulink, for all the supported system types."""

from pathlib import Path

import numpy as np
from motulator.drive.control._base import VectorControlSystem, VHzControlSystem
from motulator.drive.model import Drive
from motulator.grid.control._base import GridConverterControlSystem
from motulator.grid.model import GridConverterSystem

from motulator_export._common import add_space_vectors, system_type
from motulator_export.simulink import _grid, _im, _sm
from motulator_export.simulink._render import CODES

_MODULES = {"sm": _sm, "im": _im, "grid": _grid}


def write_model(
    path: str | Path,
    mdl: Drive | GridConverterSystem,
    ctrl: VectorControlSystem | VHzControlSystem | GridConverterControlSystem,
    t_stop: float,
    code: str = "c",
) -> Path:
    """
    Write the MATLAB scripts that build the Simulink model of the system.

    Running `build_<model>.m` in MATLAB builds the model and saves it in the same
    folder. The model runs `init_<model>.m`, which defines the parameters, after
    loading and at the start of each simulation. The references are read from `mdl`
    and `ctrl` as in `plecs.write_model`, but the gate enable must be 1, since the
    system model of Simulink does not block the gates.

    Parameters
    ----------
    path : str | Path
        Path of the model (.slx). The scripts `build_<model>.m` and `init_<model>.m`
        are written in the same folder, with the sources of the S-functions
        (`sfun_*.c`) or the MATLAB functions (`mfun_*.m`) of the blocks, which the
        model needs.
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
    path: str | Path,
    t_eval: np.ndarray,
    mdl: Drive | GridConverterSystem,
    ctrl: VectorControlSystem | VHzControlSystem | GridConverterControlSystem,
    build: bool = True,
) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
    """
    Simulate the Simulink model of the system via the MATLAB Engine API for Python.

    Parameters
    ----------
    path : str | Path
        Path of the model (.slx), next to its scripts, or of its build script
        (`build_<model>.m`, returned by `write_model`).
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

    Notes
    -----
    The signals are those of `plecs.simulate`.

    """
    module = _MODULES[system_type(mdl, ctrl)]
    signals = module.simulate(model_path(path), t_eval, mdl, ctrl, build)
    return add_space_vectors(signals[0]), signals[1]


def model_path(path: str | Path) -> Path:
    """Path of the model (.slx), also given by that of its build script."""
    path = Path(path)
    if path.suffix == ".m" and path.stem.startswith("build_"):
        return path.with_name(path.stem.removeprefix("build_") + ".slx")
    return path
