"""
Export synchronous machine drives to Simulink (pilot).

This module writes a MATLAB script that builds a Simulink model of a motulator
synchronous machine drive. The control system is the C port of the motulator
control algorithms (the same code as in the PLECS models), wrapped in the C MEX
S-function `sfun_sm_fvc.c`, in a masked subsystem whose parameters are the same as
in the motulator API. The system model is built from basic Simulink blocks, so no
toolboxes are needed.

The generated script sets the parameters and calls `build_sm_fvc.m` in this
directory, which compiles the S-function, builds the model, and saves it. The C
port uses the C99 complex type, so the S-function needs gcc (Linux), Xcode clang
(macOS), or MinGW-w64 (Windows), not MSVC.

Currently supported (a subset of the PLECS export of `motulator_plecs.sm`):

- Synchronous machine: `SynchronousMachinePars`, modeled in rotor coordinates with
  the stator flux linkage as the state, as in motulator
- Mechanical system (`MechanicalSystem` without friction)
- Converter: carrier comparison (`pwm=True` in `Drive`) with a stiff DC bus
  (`VoltageSourceConverter`). The carrier comparison uses the zero-crossing
  detection of Simulink. The computational delay of one sampling period is a Unit
  Delay block.
- Flux-vector control (`FluxVectorController`) with `SynchronousMachinePars` in the
  sensorless or sensored mode, and a speed controller (`SpeedController`) in
  `VectorControlSystem`

The model can be simulated from Python via the MATLAB Engine API for Python
(`simulate`).

"""

import os
import re
from pathlib import Path
from typing import Any

import numpy as np
from motulator.common.model._converter import VoltageSourceConverter
from motulator.drive.control._base import VectorControlSystem
from motulator.drive.model import Drive
from motulator.drive.utils._parameters import SynchronousMachinePars

from motulator_plecs import sm
from motulator_plecs._common import StepSignal, _fmt_mask
from motulator_plecs._drive import MDL_OUTPUTS
from motulator_plecs._schematic import _fmt

SIMULINK_SOURCES = Path(__file__).parent
SFUNCTION = SIMULINK_SOURCES / "sfun_sm_fvc.c"

# Mask parameters of the control system, in the order of the S-function parameters
# (the GradNet flux map is not supported yet)
MASK_PARAMS = [m for m in sm.MASK_PARAMS if m.variable != "psi_s_dq_fcn"]
PARAM_NAMES = [m.variable for m in MASK_PARAMS]

# Monitored controller signals: the second output of the S-function
CTRL_SIGNALS = sm.FVC_BLOCK.signals

# ASCII replacements in the mask prompts
ASCII = str.maketrans({"Ω": "Ohm", "²": "^2"})


def sfunction_param_names() -> list[str]:
    """Names of the S-function parameters (the enum in `sfun_sm_fvc.c`)."""
    enum = re.search(r"enum \{(.*?)NUM_PARAMS", SFUNCTION.read_text(), re.S)
    if enum is None:
        raise RuntimeError("Parameter enum not found in the S-function")
    names = re.findall(r"PRM_(\w+),", enum.group(1))
    # The C names are in upper case
    lower = {n.lower(): n for n in PARAM_NAMES}
    return [lower.get(n.lower(), n) for n in names]


def _check_supported(mdl: Drive, ctrl: VectorControlSystem) -> None:
    """Raise an error if the drive system is not supported."""
    sm._check_supported(mdl, ctrl)  # The PLECS export supports a superset
    if type(mdl.machine.par) is not SynchronousMachinePars:
        raise NotImplementedError("Only SynchronousMachinePars supported")
    if type(mdl.converter) is not VoltageSourceConverter:
        raise NotImplementedError("Only VoltageSourceConverter supported")


def export_mask_values(
    ctrl: VectorControlSystem, speed_ctrl_args: dict[str, float]
) -> dict[str, Any]:
    """
    Get the mask parameter values of the control system in the motulator API.

    Parameters
    ----------
    ctrl : VectorControlSystem
        Discrete-time control system.
    speed_ctrl_args : dict[str, float]
        Arguments of `SpeedController` used in `ctrl`.

    Returns
    -------
    dict[str, Any]
        Mask parameter values, None for the defaults.

    """
    values, flux_map = sm.export_mask_values(ctrl, speed_ctrl_args)
    if flux_map is not None:
        raise NotImplementedError("GradNet flux map not supported")
    return {n: values[n] for n in PARAM_NAMES}


def _m_str(text: str) -> str:
    """MATLAB character vector literal."""
    return "'" + text.replace("'", "''") + "'"


def _m_step(sig: StepSignal) -> str:
    """Parameters [time, before, after] of a Step block."""
    if not np.isscalar(sig.time):
        raise NotImplementedError("Only single steps supported")
    return _fmt([sig.time, sig.before, sig.after])


def write_model(
    path: str | Path,
    mdl: Drive,
    ctrl: VectorControlSystem,
    w_M_ref: StepSignal,
    tau_L: StepSignal,
    t_stop: float,
    speed_ctrl_args: dict[str, float],
) -> Path:
    """
    Write a MATLAB script that builds the Simulink model of the drive system.

    Running the script in MATLAB compiles the S-function and saves the model (and
    the compiled S-function) in the folder of the script.

    Parameters
    ----------
    path : str | Path
        Path of the model (.slx). The script is written in the same folder, named
        `build_<model>.m`.
    mdl : Drive
        Continuous-time system model.
    ctrl : VectorControlSystem
        Discrete-time control system.
    w_M_ref : StepSignal
        Speed reference (mechanical rad/s).
    tau_L : StepSignal
        External load torque (Nm).
    t_stop : float
        Simulation stop time (s).
    speed_ctrl_args : dict[str, float]
        Arguments of `SpeedController` used in `ctrl`.

    Returns
    -------
    Path
        Path of the written script.

    """
    path = Path(path)
    _check_supported(mdl, ctrl)
    values = export_mask_values(ctrl, speed_ctrl_args)
    variables = sm.export_plant_variables(mdl)
    init = ", ...\n    ".join(_m_str(f"{n} = {_fmt(v)};") for n, v in variables)
    mask = "".join(
        "    "
        + ", ".join(
            _m_str(v)
            for v in (
                m.variable,
                m.prompt.translate(ASCII),
                m.tab,
                _fmt_mask(values[m.variable]),
            )
        )
        + "\n"
        for m in MASK_PARAMS
    )
    try:
        src = Path(os.path.relpath(SIMULINK_SOURCES, path.resolve().parent))
    except ValueError:  # Different drives (Windows)
        src = SIMULINK_SOURCES
    src_parts = ", ".join(_m_str(p) for p in src.parts)
    text = (
        f"% Build the Simulink model {path.stem}.slx in the folder of this script.\n"
        "% Generated by motulator_plecs.simulink.sm from motulator. The parameters of\n"
        "% the control system are in the mask of the subsystem 'Flux-vector\n"
        "% control', and those of the system model in the model workspace.\n"
        "%\n"
        "% The S-function is compiled with mex, which needs a C99 compiler with\n"
        "% complex.h (gcc, clang, or MinGW-w64, not MSVC), see mex -setup C.\n"
        "\n"
        "here = fileparts(mfilename('fullpath'));\n"
        f"addpath(fullfile(here, {src_parts}));\n"
        "\n"
        f"s.name = {_m_str(path.stem)};\n"
        "s.folder = here;\n"
        "s.init = strjoin({ ...\n"
        f"    {init}}}, newline);\n"
        "% {variable, prompt, tab, value}\n"
        "s.mask = { ...\n" + mask + "    };\n"
        f"s.w_M_ref = {_m_step(w_M_ref)};  % [time, before, after]\n"
        f"s.tau_L = {_m_step(tau_L)};\n"
        f"s.T_s = {_fmt(values['T_s'])};\n"
        f"s.t_stop = {_fmt(t_stop)};\n"
        "\n"
        "build_sm_fvc(s);\n"
    )
    script = path.with_name(f"build_{path.stem}.m")
    script.write_text(text)
    return script


def simulate(
    path: str | Path, t_eval: np.ndarray, build: bool = True
) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
    """
    Simulate the Simulink model via the MATLAB Engine API for Python.

    Parameters
    ----------
    path : str | Path
        Path of the model (.slx).
    t_eval : ndarray
        Output times (s).
    build : bool, optional
        Run the build script `build_<model>.m` first, defaults to True.

    Returns
    -------
    mdl : dict[str, ndarray]
        Machine signals (`MDL_OUTPUTS`) and the time "t".
    ctrl : dict[str, ndarray]
        Controller signals and the time "t".

    """
    # MATLAB Engine API for Python
    import matlab  # noqa: PLC0415  # pyright: ignore[reportMissingImports]
    import matlab.engine  # noqa: PLC0415  # pyright: ignore[reportMissingImports]

    path = Path(path).resolve()
    eng: Any = matlab.engine.start_matlab()
    try:
        if build:
            eng.run(str(path.with_name(f"build_{path.stem}.m")), nargout=0)
        eng.addpath(str(path.parent), nargout=0)
        eng.workspace["t_eval"] = matlab.double(np.asarray(t_eval, float).tolist())
        eng.eval(
            f"out = sim({_m_str(path.stem)}, 'OutputOption', "
            "'SpecifiedOutputTimes', 'OutputTimes', 't_eval');",
            nargout=0,
        )
        t = np.asarray(eng.eval("out.tout")).ravel()
        values = np.asarray(eng.eval("out.yout")).T
    finally:
        eng.quit()
    n_mdl = len(MDL_OUTPUTS)
    if values.shape[0] != n_mdl + len(CTRL_SIGNALS):
        raise RuntimeError(f"Unexpected number of output signals: {values.shape}")
    mdl = {"t": t, **dict(zip(MDL_OUTPUTS, values[:n_mdl], strict=True))}
    ctrl = {"t": t, **dict(zip(CTRL_SIGNALS, values[n_mdl:], strict=True))}
    return mdl, ctrl
