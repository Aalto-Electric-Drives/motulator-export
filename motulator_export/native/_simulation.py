"""Native simulation with the interface of the simulation of motulator."""

import os
import warnings
from collections.abc import Mapping
from dataclasses import replace
from types import SimpleNamespace
from typing import Any, cast

import numpy as np
from motulator.common.control._base import ControlSystem
from motulator.common.model import (
    CompositeModel,
    CompositeModelTimeSeries,
    DiodeBridge,
    SolverCfg,
    VoltageSourceConverter,
)
from motulator.common.model._base import Model, ModelTimeSeries
from motulator.common.model._simulation import SimulationResults
from tqdm import tqdm

from motulator_export._common import split_system
from motulator_export.native._grid import GridConverterNativeSystem
from motulator_export.native._im import InductionMachineDriveSystem
from motulator_export.native._sm import SynchronousMachineDriveSystem
from motulator_export.native._system import ERRORS, NativeSystem

# Supported systems, each with the class method `supports(mdl, ctrl)`
SYSTEMS = [
    SynchronousMachineDriveSystem,
    InductionMachineDriveSystem,
    GridConverterNativeSystem,
]

# Number of runs between the updates of the progress bar
_CHUNKS = 100


class Simulation:
    """
    Native simulation.

    Simulates the system model and the control system built in motulator in C, with
    the interface and the results of `motulator.common.model.Simulation`. As in
    motulator, the results are also stored in the time series of `mdl` and `ctrl`,
    and the states of `mdl` are updated. The control system runs in C from its
    initial state, so the states of `ctrl` are not used or updated. A drive fed from a
    diode bridge is given as a composite model of the diode bridge and the drive, with
    the control system of the drive by its name, as in motulator.

    The parameters are read at the first call of `simulate`, and the references and
    the load torque at each call. They can be constants, `StepSignal` objects
    (fastest), or functions of time.

    Parameters
    ----------
    mdl : Model
        Continuous-time system model: a drive, a grid converter system, or a composite
        model (`CompositeModel`) of a diode bridge (`GridConverterSystem` with
        `DiodeBridge`) and a drive sharing its DC bus, in this order.
    ctrl : ControlSystem | Mapping[str, ControlSystem]
        Discrete-time control system, of a composite model by the name of the drive.
    show_progress : bool, optional
        Show progress during simulation, defaults to True.
    cfg : SolverCfg, optional
        Solver configuration parameters. Only the method "RK45" is supported.
    t_start : float, optional
        First sampling instant (s) of the control system, defaults to the initial
        time of the model.

    """

    def __init__(
        self,
        mdl: Model,
        ctrl: ControlSystem | Mapping[str, ControlSystem],
        show_progress: bool = True,
        cfg: SolverCfg | None = None,
        t_start: float | None = None,
    ) -> None:
        if os.environ.get("BUILDING_DOCS") == "1":
            show_progress = False
        self.show_progress = show_progress
        self.cfg = cfg if cfg is not None else SolverCfg()
        self.mdl = mdl
        self.ctrl = ctrl
        # The model and the control system of the drive or the grid converter, and the
        # diode bridge feeding the drive
        self._parts = split_system(mdl, ctrl)
        sub_mdl, sub_ctrl, front = self._parts
        system = next((s for s in SYSTEMS if s.supports(sub_mdl, sub_ctrl)), None)
        if system is None:
            raise NotImplementedError(
                "Only machine drives (Drive), also fed from a diode bridge, and grid "
                "converter systems (GridConverterSystem) supported"
            )
        system.check_supported(sub_mdl, sub_ctrl, front)
        self._solver_cfg = _solver_cfg(self.cfg)
        if t_start is not None:
            if not mdl.t0 <= t_start < np.inf:
                raise ValueError("t_start must not be earlier than the model time")
            sub_ctrl.t = t_start
        self._system_type = system
        self._t_start = mdl.t0 if t_start is None else t_start
        self._system: NativeSystem | None = None

    def simulate(self, t_stop: float = 1.0, N_eval: int = 0) -> SimulationResults:
        """
        Simulate the system.

        The simulation runs until the first sampling instant after `t_stop`. A
        subsequent call continues the simulation.

        Parameters
        ----------
        t_stop : float, optional
            Simulation stop time, defaults to 1.
        N_eval : int, optional
            Number of evenly spaced data points to be returned by the solver for each
            integration interval, i.e., for each interval between the sampling and
            switching instants. Defaults to 0, in which case the number and spacing of
            points is selected by the solver.

        """
        if not np.isfinite(t_stop):
            raise ValueError("t_stop must be finite")
        if N_eval < 0:
            raise ValueError("N_eval must be nonnegative")
        mdl = self.mdl
        sub_mdl, ctrl, front = self._parts
        system = self._system
        if system is None:
            system = self._system_type(sub_mdl, ctrl, front)
            system.start(mdl, self._t_start, ctrl.t, self._solver_cfg)
            self._system = system
        system.set_signals(sub_mdl, ctrl)
        system.set_enable(ctrl)

        progress_bar = None
        if self.show_progress:
            progress_bar = tqdm(
                total=t_stop,
                desc="Simulation",
                unit="s",
                bar_format="{l_bar}{bar}| {n:.2f}/{total:.2f} {unit}",
            )
        # The runs end at sampling instants, so a run in parts equals a single run
        status, t0 = 0, mdl.t0
        try:
            for t in np.linspace(t_stop / _CHUNKS, t_stop, _CHUNKS):
                if t <= t0 and t < t_stop:
                    continue
                status = system.run(float(t), int(N_eval))
                t0, _ = system.state()
                if progress_bar is not None:
                    progress_bar.n = min(t0, t_stop)
                    progress_bar.refresh()
                if status != 0:
                    break
        finally:
            if progress_bar is not None:
                progress_bar.close()
            # Save the results, and the model ends at the states of the simulation,
            # with the switching state of the converter (set by the simulation of
            # motulator at the switching instants)
            _save(system, mdl, ctrl)
            _set_final_state(system, mdl)
        if status != 0:
            msg = f"Simulation stopped at {mdl.t0:.2f} s: {ERRORS[status]}"
            warnings.warn(msg, RuntimeWarning, stacklevel=2)
        return _results(mdl, self.ctrl, status == 0)


def _solver_cfg(cfg: SolverCfg) -> SolverCfg:
    """Check the solver configuration, limiting the relative tolerance as scipy."""
    if cfg.method != "RK45":
        raise NotImplementedError("Only the solver method RK45 supported")
    if not cfg.atol >= 0:
        raise ValueError("atol must be nonnegative")
    if cfg.max_step is not None and not cfg.max_step > 0:
        raise ValueError("max_step must be positive")
    rtol_min = 100 * np.finfo(float).eps
    if cfg.rtol < rtol_min:
        msg = f"rtol is too small, setting rtol = {rtol_min}"
        warnings.warn(msg, UserWarning, stacklevel=3)
    return replace(cfg, rtol=max(cfg.rtol, rtol_min))


def _set_final_state(system: NativeSystem, mdl: Model) -> None:
    """
    Set the model to the states of the simulation, with the switching state of the
    converter (set by the simulation of motulator at the switching instants) and the
    diode states of the converters.
    """
    t0, x = system.state()
    for model in _models(mdl):
        model.t0 = t0
    mdl.set_states(list(x))
    for converter in mdl.converters:
        if isinstance(converter, VoltageSourceConverter):
            inp = converter.inp
            inp.q_c_ab, inp.s_c_ab, inp.enabled, converter._diodes = (
                system.converter_state()
            )
        elif isinstance(converter, DiodeBridge):
            converter._diodes = system.bridge_state()
    mdl.set_outputs(mdl.t0)


def _results(
    mdl: Model, ctrl: ControlSystem | Mapping[str, ControlSystem], success: bool
) -> SimulationResults:
    """
    Results of the simulation, of a composite model with the results of the control
    system by the name of the drive, and None for the diode bridge, as in motulator.
    """
    if isinstance(mdl, CompositeModel):
        ctrls = cast(Mapping[str, ControlSystem], ctrl)
        ctrl_ts = {name: c.post_process() for name, c in ctrls.items()}
        ctrl_res = SimpleNamespace(**{name: ctrl_ts.get(name) for name in mdl.models})
        return SimulationResults(CompositeModelTimeSeries(mdl), ctrl_res, success)
    ctrl = cast(ControlSystem, ctrl)
    return SimulationResults(ModelTimeSeries(mdl), ctrl.post_process(), success)


def _models(mdl: Model) -> list[Model]:
    """The model, and the models of a composite model, which have their own clocks."""
    return [mdl, *mdl.models.values()] if isinstance(mdl, CompositeModel) else [mdl]


def _save(system: NativeSystem, mdl: Model, ctrl: ControlSystem) -> None:
    """
    Save the logged data of the system into the time series of the model and the
    control system, as `Model.save` and `ControlSystem.save` do.
    """
    # Model: the time, the states, the switching state and the current-direction
    # vector of the converter, its gate enable, and its voltage with the gates blocked
    # (NaN with the gates enabled), followed by the voltage of the diode bridge
    data = system.fetch(control=False)
    n = len(system.fields(control=False))
    m = 1 + 2 * n
    z = data[:, 1:m:2] + 1j * data[:, 2:m:2]
    for model in _models(mdl):
        model._t.extend(data[:, 0])
    index = 0
    for subsystem in mdl.subsystems:
        index = subsystem.extend_state_history(z.T, index)  # type: ignore
    for converter in mdl.converters:
        if isinstance(converter, VoltageSourceConverter):
            history = converter._history
            history["q_c_ab"].extend(data[:, m] + 1j * data[:, m + 1])
            history["s_c_ab"].extend(data[:, m + 2] + 1j * data[:, m + 3])
            history["enabled"].extend(bool(e) for e in data[:, m + 4])
            history["u_c_ab"].extend(data[:, m + 5] + 1j * data[:, m + 6])
        else:
            u_c_ab = data[:, m + 7] + 1j * data[:, m + 8]
            converter.extend_history(len(data), list(u_c_ab))

    # Control system: the controller time and the signal groups
    data = system.fetch(control=True)
    ctrl._t.extend(data[:, 0])
    for group, name, values in _fields(system.fields(control=True), data):
        ctrl._history.setdefault(group, {}).setdefault(name, []).extend(values)


def _fields(fields: list[str], data: np.ndarray) -> list[tuple[str, str, Any]]:
    """
    Values of the logged fields, whose rows start with the time (see
    `log_fields_width` in `c/sim.c`): the group, the name, and the values of each
    field.
    """
    result, col = [], 1
    for field in fields:
        path, _, kind = field.partition(":")
        group, name = path.split(".")
        if kind == "c":  # Complex
            width, values = 2, data[:, col] + 1j * data[:, col + 1]
        elif kind == "n":  # None, not logged
            width, values = 0, [None] * len(data)
        elif kind == "b":  # Bool
            width, values = 1, data[:, col] != 0
        elif kind:  # Array, as rows
            width = int(kind)
            values = list(data[:, col : col + width])
        else:
            width, values = 1, data[:, col]
        result.append((group, name, values))
        col += width
    return result
