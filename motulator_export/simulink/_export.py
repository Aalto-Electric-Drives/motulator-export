"""
Export of motulator systems to Simulink, for all the supported system types.

The model is built in MATLAB from the blocks of the library of motulator, by a
script that builds the system with the MATLAB classes of motulator (see `_script`).
The supported systems are those of the PLECS export (which supports a superset,
see `plecs._sm`, `plecs._im`, and `plecs._grid`) with the converter modeled by the
carrier comparison with a stiff DC bus, without the dead time, the threshold
voltage, and the gate blocking of the converter. The control systems may compensate
for the dead time and the threshold voltage, as on the hardware.

"""

from pathlib import Path
from typing import Any, cast

import numpy as np
from motulator.common.model._converter import VoltageSourceConverter
from motulator.common.model._pwm import CarrierComparison
from motulator.drive.control._base import VectorControlSystem, VHzControlSystem
from motulator.drive.model import Drive
from motulator.grid.control._base import GridConverterControlSystem
from motulator.grid.model import GridConverterSystem

from motulator_export._common import add_space_vectors, system_type
from motulator_export.plecs import _grid, _im, _sm
from motulator_export.plecs._drive import MDL_OUTPUTS
from motulator_export.simulink._common import check_supported_model
from motulator_export.simulink._common import simulate as _simulate
from motulator_export.simulink._render import CODES
from motulator_export.simulink._script import write_script


def check_supported_converter(
    mdl: Drive, ctrl: VectorControlSystem | VHzControlSystem
) -> None:
    """
    Raise an error if the converter or the DC bus of a drive is not supported.

    Only the stiff DC bus (no DC-bus model) is supported. The dead time and the
    threshold voltage are not modeled in the system model, but their compensation in
    the control system is supported. The system model updates the duty ratios twice
    per carrier period, and its gates are not blocked (see `check_supported_model`).

    """
    if type(mdl.converter) is not VoltageSourceConverter:
        raise NotImplementedError("Only VoltageSourceConverter supported")
    if mdl.dc_bus is not None:
        raise NotImplementedError("DC-bus model not supported, only a stiff DC bus")
    if cast(CarrierComparison, mdl.converter.modulator).t_d != 0 or mdl.converter.u_th:
        raise NotImplementedError(
            "Dead time and threshold voltage not supported in the system model"
        )
    check_supported_model(mdl, ctrl)


def _check_supported(kind: str, mdl: Drive | GridConverterSystem, ctrl: object) -> None:
    """Raise an error if the system of the type `kind` is not supported."""
    if kind == "grid":
        grid = cast(GridConverterSystem, mdl)
        grid_ctrl = cast(GridConverterControlSystem, ctrl)
        _grid.check_supported(grid, grid_ctrl)
        check_supported_model(grid, grid_ctrl)
        return
    drive = cast(Drive, mdl)
    drive_ctrl = cast(VectorControlSystem | VHzControlSystem, ctrl)
    if kind == "sm":
        _sm.check_supported(drive, drive_ctrl)
    else:
        _im.check_supported(drive, drive_ctrl)
    check_supported_converter(drive, drive_ctrl)


def write_model(
    path: str | Path,
    mdl: Drive | GridConverterSystem,
    ctrl: VectorControlSystem | VHzControlSystem | GridConverterControlSystem,
    t_stop: float,
    code: str = "c",
) -> Path:
    """
    Write the MATLAB script that builds the Simulink model of the system.

    The script `build_<model>.m` builds the system with the MATLAB classes of
    motulator, as a motulator script does, and the model from the blocks of the
    library of motulator, and saves the model in the same folder. The parameters of
    the model are in its model workspace. The references are read from `mdl` and
    `ctrl` as in `plecs.write_model`, but the gate enable must be 1, since the system
    model of Simulink does not block the gates.

    Parameters
    ----------
    path : str | Path
        Path of the model (.slx). The script `build_<model>.m` is written in the
        same folder.
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
    _check_supported(system_type(mdl, ctrl), mdl, ctrl)
    return write_script(path, mdl, ctrl, t_stop, code)


def simulate(
    path: str | Path,
    t_eval: np.ndarray,
    mdl: Drive | GridConverterSystem,
    ctrl: VectorControlSystem | VHzControlSystem | GridConverterControlSystem,
    build: bool = True,
    eng: Any = None,
) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
    """
    Simulate the Simulink model of the system via the MATLAB Engine API for Python.

    Parameters
    ----------
    path : str | Path
        Path of the model (.slx), next to its build script, or of its build script
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
    eng : MatlabEngine, optional
        MATLAB engine to use, e.g., for several simulations. By default, a new one
        is started, and quit at the end.

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
    kind = system_type(mdl, ctrl)
    if kind == "grid":
        grid = cast(GridConverterSystem, mdl)
        grid_ctrl = cast(GridConverterControlSystem, ctrl)
        outputs = _grid.mdl_outputs(grid)
        signals = _grid.control_block(grid_ctrl).signals
    else:
        drive_ctrl = cast(VectorControlSystem | VHzControlSystem, ctrl)
        machine = _sm if kind == "sm" else _im
        outputs, signals = MDL_OUTPUTS, machine.control_block(drive_ctrl).signals
    results = _simulate(model_path(path), t_eval, outputs, signals, build, eng)
    return add_space_vectors(results[0]), results[1]


def model_path(path: str | Path) -> Path:
    """Path of the model (.slx), also given by that of its build script."""
    path = Path(path)
    if path.suffix == ".m" and path.stem.startswith("build_"):
        return path.with_name(path.stem.removeprefix("build_") + ".slx")
    return path
