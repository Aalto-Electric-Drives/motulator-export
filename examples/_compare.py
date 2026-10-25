"""
Helpers of the comparison scripts: the simulation of motulator with tight tolerances,
the output times of the exported models, the maximum differences, and the plots of
the drives.

The exported model gives its signals at the solver steps of motulator and just after
the sampling instants, where the monitored signals of the control system have been
updated.

"""

import atexit
import sys
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
from motulator.common.model import Simulation, SimulationResults, SolverCfg

from motulator_export import plecs, simulink


def simulate_motulator(mdl: Any, ctrl: Any, t_stop: float) -> SimulationResults:
    """Simulate the system in motulator with the tolerances of 1e-9."""
    sim = Simulation(mdl, ctrl, cfg=SolverCfg(rtol=1e-9, atol=1e-9))
    return sim.simulate(t_stop=t_stop)


def write_plecs_model(path: Path, mdl: Any, ctrl: Any, t_stop: float) -> Path:
    """
    Write the PLECS model and a temporary copy of it with the output ports for
    `plecs.simulate` (removed at exit), returning the path of the copy.
    """
    plecs.write_model(path, mdl, ctrl, t_stop)
    print(f"Wrote {path}")
    tmp = plecs.write_model(
        path.with_stem(path.stem + "_tmp"), mdl, ctrl, t_stop, outputs=True
    )
    atexit.register(tmp.unlink, missing_ok=True)
    return tmp


def simulate_plecs(
    path: Path, t_eval: np.ndarray, mdl: Any, ctrl: Any
) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
    """Simulate in PLECS, exiting if its RPC interface is not available."""
    try:
        return plecs.simulate(path, t_eval, mdl, ctrl)
    except ConnectionRefusedError:
        print("PLECS RPC interface not available, skipping the comparison.")
        sys.exit()


def write_simulink_model(name: str, mdl: Any, ctrl: Any, t_stop: float) -> Path:
    """
    Write the build script of the Simulink model `simulink/<name>.slx`, or with
    --matlab that of `simulink/<name>_matlab.slx` with MATLAB Function blocks,
    returning the path of the model.
    """
    code = "matlab" if "--matlab" in sys.argv else "c"
    suffix = "_matlab" if code == "matlab" else ""
    path = Path(__file__).parent / "simulink" / f"{name}{suffix}.slx"
    path.parent.mkdir(exist_ok=True)
    print(f"Wrote {simulink.write_model(path, mdl, ctrl, t_stop, code=code)}")
    return path


def require_matlab_engine() -> None:
    """Exit if the MATLAB Engine API for Python is not available."""
    try:
        import matlab.engine  # noqa: F401, PLC0415  # pyright: ignore[reportMissingImports]
    except ImportError:
        print("MATLAB Engine API for Python not available, skipping the comparison.")
        sys.exit()


class Comparison:
    """
    Comparison of the signals of an exported model with motulator.

    Parameters
    ----------
    res : SimulationResults
        Results of motulator, which runs to the end of the last sampling period.
    t_stop : float
        Stop time of the exported model (s).
    control : bool, optional
        Also compare the signals of the control system, defaults to True: the
        exported model gives its signals also just after the sampling instants.

    Attributes
    ----------
    in_mdl, in_ctrl : ndarray
        Solver steps and sampling instants of motulator up to `t_stop`.
    t_mdl, t_ctrl : ndarray
        Solver steps of motulator and the times just after the sampling instants,
        up to `t_stop` (s).
    t_eval : ndarray
        Output times of the exported model (s).

    """

    def __init__(
        self, res: SimulationResults, t_stop: float, control: bool = True
    ) -> None:
        T_s = res.ctrl.ref.T_s[0]
        self.in_mdl = res.mdl.t <= t_stop
        self.in_ctrl = res.ctrl.t + 0.5 * T_s <= t_stop
        self.t_mdl = res.mdl.t[self.in_mdl]
        self.t_ctrl = res.ctrl.t[self.in_ctrl] + 0.5 * T_s
        times = [self.t_mdl, self.t_ctrl] if control else [self.t_mdl]
        self.t_eval = np.unique(np.concatenate(times))

    @staticmethod
    def at(signals: dict[str, np.ndarray], name: str, t: np.ndarray) -> np.ndarray:
        """
        Values of an exported signal at the times `t`, which should be among the
        times "t" of the signals (e.g., Simulink also gives the stop time).
        """
        tol = 1e-12 * np.maximum(np.abs(t), 1.0)
        k = np.searchsorted(signals["t"], t - tol)
        if k.max() >= len(signals["t"]) or np.any(np.abs(signals["t"][k] - t) > tol):
            raise ValueError("The times are not among the times of the signals")
        return signals[name][k]

    def print_differences(
        self,
        tool: str,
        exported: tuple[dict[str, np.ndarray], dict[str, np.ndarray]],
        mdl: dict[str, np.ndarray],
        ctrl: dict[str, np.ndarray] | None = None,
    ) -> None:
        """
        Print the maximum differences of the signals of the exported model from
        motulator: those of the system model `mdl` at the solver steps and those of
        the control system `ctrl` just after the sampling instants, by the names of
        the exported signals.
        """
        print(f"Maximum differences ({tool} - motulator):")
        for name, value in mdl.items():
            exp = self.at(exported[0], name, self.t_mdl)
            print(f"  mdl.{name}: {np.max(np.abs(exp - value[self.in_mdl])):.3g}")
        for name, value in (ctrl or {}).items():
            exp = self.at(exported[1], name, self.t_ctrl)
            print(f"  ctrl.{name}: {np.max(np.abs(exp - value[self.in_ctrl])):.3g}")


def drive_signals(res: SimulationResults) -> dict[str, np.ndarray]:
    """Signals of the system model of a drive in motulator, by the exported names."""
    machine = res.mdl.machine
    return {
        "w_M": res.mdl.mechanics.w_M,
        "tau_M": machine.tau_M,
        "i_s_ab": machine.i_s_ab,
    }


def plot_drive(
    cmp: Comparison, res: SimulationResults, mdl: dict[str, np.ndarray], tool: str
) -> None:
    """
    Plot the speed, the torque, and the alpha component of the stator current of a
    drive in motulator and in the exported model (the signals `mdl`).
    """
    t, in_mdl = cmp.t_mdl, cmp.in_mdl
    labels = [
        (r"$\omega_\mathrm{M}$", "Speed (rad/s)"),
        (r"$\tau_\mathrm{M}$", "Torque (Nm)"),
        (r"$i_\alpha$", "Current (A)"),
    ]
    fig, axs = plt.subplots(3, 1, sharex=True, figsize=(8, 7))
    signals = drive_signals(res).items()
    for ax, (name, value), (symbol, unit) in zip(axs, signals, labels, strict=True):
        ax.plot(t, np.real(value[in_mdl]), label=f"{symbol} (motulator)")
        ax.plot(t, np.real(cmp.at(mdl, name, t)), "--", label=f"{symbol} ({tool})")
        ax.set_ylabel(unit)
        ax.legend(loc="upper right")
        ax.grid(True)
    axs[-1].set_xlabel("Time (s)")
    fig.tight_layout()
    plt.show()


def plot_grid(
    cmp: Comparison,
    res: SimulationResults,
    exported: tuple[dict[str, np.ndarray], dict[str, np.ndarray]],
    tool: str,
) -> None:
    """
    Plot the powers and the alpha component of the converter current of a grid
    converter in motulator and in the exported model.
    """
    fbk, (mdl, ctrl) = res.ctrl.fbk, exported
    fig, axs = plt.subplots(2, 1, sharex=True, figsize=(8, 6))
    for name, symbol in (("p_g", r"$p_\mathrm{g}$"), ("q_g", r"$q_\mathrm{g}$")):
        axs[0].plot(res.ctrl.t, getattr(fbk, name), label=f"{symbol} (motulator)")
        axs[0].plot(ctrl["t"], ctrl[name], "--", label=f"{symbol} ({tool})")
    axs[0].set_ylabel("Power (W, VAr)")
    t, symbol = cmp.t_mdl, r"$i_{c\alpha}$"
    i_c_ab = res.mdl.ac_filter.i_c_ab[cmp.in_mdl]
    axs[1].plot(t, np.real(i_c_ab), label=f"{symbol} (motulator)")
    i_c_ab = cmp.at(mdl, "i_c_ab", t)
    axs[1].plot(t, np.real(i_c_ab), "--", label=f"{symbol} ({tool})")
    axs[1].set_ylabel("Current (A)")
    axs[1].set_xlabel("Time (s)")
    for ax in axs:
        ax.legend(loc="upper right")
        ax.grid(True)
    fig.tight_layout()
    plt.show()
