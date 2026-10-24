"""Export of motulator systems to PLECS, for all the supported system types."""

from pathlib import Path

import numpy as np
from motulator.drive.control._base import VectorControlSystem, VHzControlSystem
from motulator.drive.model import Drive
from motulator.grid.control._base import GridConverterControlSystem
from motulator.grid.model import GridConverterSystem

from motulator_export._common import add_space_vectors, system_type
from motulator_export.plecs import _grid, _im, _sm

_MODULES = {"sm": _sm, "im": _im, "grid": _grid}


def write_model(
    path: str | Path,
    mdl: Drive | GridConverterSystem,
    ctrl: VectorControlSystem | VHzControlSystem | GridConverterControlSystem,
    t_stop: float,
    outputs: bool = False,
    library: bool = False,
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
    library : bool, optional
        Refer to the blocks of the library of the package (`motulator_lib.plecs`),
        the control system and the GradNet machine, instead of copying them into the
        model, defaults to False. The model refers to the library by a path relative
        to the model file, and PLECS copies the blocks from it when it loads the
        model.

    Returns
    -------
    Path
        Path of the written model file.

    """
    return _MODULES[system_type(mdl, ctrl)].write_model(
        path, mdl, ctrl, t_stop, outputs, library
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

    Notes
    -----
    The signals of the system model are, in SI units:

    - Drives: the phase currents `i_a`, `i_b`, and `i_c`, the stator current
      `i_s_ab` (a complex space vector in stationary coordinates, as in motulator),
      the rotor speed `w_M`, the rotor angle `theta_M`, and the torque `tau_M`.
    - Grid converters: the converter currents `i_c_a`, `i_c_b`, `i_c_c`, and
      `i_c_ab`, and, with an LCL filter, the grid currents `i_g_a`, `i_g_b`,
      `i_g_c`, and `i_g_ab`.

    The monitored signals of the control system are real, the magnitudes of the
    space vectors (e.g., `psi_s`) and the components of the currents in the
    coordinates of the controller (e.g., `i_d`):

    - Flux-vector control: `w_M_ref`, `w_M`, `tau_M_ref`, `tau_M`, `psi_s_ref`,
      `psi_s`, `theta_m`, `i_d`, and `i_q`.
    - Current-vector control: `w_M_ref`, `w_M`, `tau_M_ref`, `tau_M`, `psi_s`,
      `psi_R`, `i_sd_ref`, `i_sd`, `i_sq_ref`, and `i_sq`.
    - Observer-based V/Hz control of an induction machine: `w_M_ref`, `w_s`,
      `tau_M_ref`, `tau_M`, `psi_s_ref`, `psi_s`, `psi_R`, `i_sd`, and `i_sq`.
    - Observer-based V/Hz control of a synchronous machine: `w_M_ref`, `w_c`,
      `tau_M_ref`, `tau_M`, `psi_s_ref`, `psi_s`, `theta_m`, `i_d`, and `i_q`.
    - Grid-following control: `p_g_ref`, `p_g`, `q_g_ref`, `q_g`, `i_c_d_ref`,
      `i_c_d`, `i_c_q_ref`, `i_c_q`, `u_g`, `w_g`, and `theta_c`.
    - Grid-forming control: `p_g_ref`, `p_g`, `q_g`, `v_c_ref`, `v_c`, `i_c_d_ref`,
      `i_c_d`, `i_c_q_ref`, `i_c_q`, and `theta_c` (the angle of the converter
      voltage).

    """
    signals = _MODULES[system_type(mdl, ctrl)].simulate(path, t_eval, mdl, ctrl)
    return add_space_vectors(signals[0]), signals[1]
