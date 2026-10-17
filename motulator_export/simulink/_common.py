"""
Common parts of the Simulink export: the build scripts and the simulation.

A model is exported as a MATLAB script, which sets the fields of a struct `s` (the
control system and the signals) and calls a builder function of this directory
(e.g., `build_drive.m`). The builder compiles the S-functions (or reads the MATLAB
functions of the MATLAB Function blocks), builds the model with the Simulink API,
and saves it in the folder of the script. The parameters are defined by the script
`init_<model>.m` in the base workspace, which the model runs after loading and at
the start of each simulation.

"""

import os
from collections.abc import Callable, Sequence
from pathlib import Path
from typing import Any

import numpy as np
from motulator.common.model._base import Model
from motulator.drive.control._base import DriveControlSystem
from motulator.grid.control._base import GridConverterControlSystem

from motulator_export.plecs._common import (
    MONITOR,
    ControlBlock,
    StepSignal,
    _fmt_mask,
    gate_enable,
)
from motulator_export.plecs._schematic import _fmt
from motulator_export.simulink._mfunction import MFunction, path_functions
from motulator_export.simulink._render import (
    ASCII,
    CODES,
    mfunctions,
    netlist_rows,
    sfunction_params,
    sfunctions,
    with_signals,
)
from motulator_export.simulink._sfunction import SFunction

SIMULINK_SOURCES = Path(__file__).parent


def check_supported_model(
    mdl: Model, ctrl: DriveControlSystem | GridConverterControlSystem
) -> None:
    """
    Raise an error if the system model updates the duty ratios once per carrier
    period or the control system blocks the gates (`set_enable`), which the ideal
    converter of the system model does not model.
    """
    if mdl.single_update:
        raise NotImplementedError("Single update not supported in the system model")
    if gate_enable(ctrl) != 1:
        raise NotImplementedError(
            "Gate blocking (set_enable) not supported in the system model"
        )


def _m_str(text: str) -> str:
    """MATLAB character vector, joined from its lines if it has several."""
    if "\n" in text:
        lines = text.rstrip("\n").split("\n")
        items = ", ...\n    ".join(map(_m_str, lines))
        return "strjoin({ ...\n    " + items + "}, newline)"
    return "'" + text.replace("'", "''") + "'"


def _m(value: Any) -> str:
    """MATLAB literal of a string, a number, or a (nested) list."""
    if isinstance(value, str):
        return _m_str(value)
    if isinstance(value, Sequence):
        if any(isinstance(v, str) for v in value):  # Cell array
            return "{" + ", ".join(map(_m, value)) + "}"
        rows = [v for v in value if isinstance(v, Sequence) and not isinstance(v, str)]
        if not rows or len(rows) < len(value):
            return _fmt(list(value))  # Numeric vector
        if all(_is_numeric(r) for r in rows):
            return "[" + "; ".join(" ".join(_fmt(x) for x in r) for r in rows) + "]"
        cells = "".join("    " + ", ".join(map(_m, r)) + "\n" for r in rows)
        return "{ ...\n" + cells + "    }"
    return _fmt(value)


def _is_numeric(values: Sequence[Any]) -> bool:
    """Whether all the values are numbers."""
    return all(isinstance(v, (int, float, np.number)) for v in values)


def _m_steps(sig: StepSignal) -> list[list[float]]:
    """
    Rows [time, before, after] of the steps of a step signal (see add_step.m).

    The first row steps from `before` to the first level, and the others from zero
    to the increments of the levels, so that the sum of the steps is the signal.

    """
    times = np.atleast_1d(sig.time)
    levels = [sig.before, *np.atleast_1d(sig.after)]
    rows = [[float(times[0]), float(levels[0]), float(levels[1])]]
    rows += [
        [float(t), 0.0, float(levels[k + 2] - levels[k + 1])]
        for k, t in enumerate(times[1:])
    ]
    return rows


def _m_source(sig: StepSignal | float) -> list[list[float]] | float:
    """Steps of a step signal (see `_m_steps`), or a constant."""
    return _m_steps(sig) if isinstance(sig, StepSignal) else float(sig)


def control_fields(
    block: ControlBlock,
    sfuns: Sequence[SFunction],
    values: dict[str, Any],
    code: str = "c",
) -> list[tuple[str, Any]]:
    """
    Fields of the struct of the control system (see add_control_system.m), given the
    S-functions of its blocks (none for the MATLAB Function blocks of `code`
    "matlab").
    """
    mask = [
        [m.variable, m.prompt.translate(ASCII), m.tab, _fmt_mask(values[m.variable])]
        for m in block.mask_params
    ]
    top = with_signals(block.netlist(), MONITOR)
    rows = netlist_rows(top, code=code)
    return [
        ("name", block.name),
        ("sfunction", [f.name for f in sfuns]),
        ("mask_type", block.mask_type),
        ("description", block.description),
        ("mask", mask),
        ("mask_init", block.mask_init),
        ("inputs", block.inputs),
        ("netlist.blocks", rows["blocks"]),
        ("netlist.aligns", rows["aligns"]),
        ("netlist.lines", rows["lines"]),
        ("netlist.masks", rows["masks"]),
        ("sfunctions", sfunction_params(top) if code == "c" else []),
    ]


def write_init(path: Path, init: Callable[[str], str]) -> str:
    """
    Write the script `init_<model>.m` defining the parameters of the model, given
    the function of the model initialization from its comment. Returns the name of
    the script.
    """
    name = path.stem
    comment = (
        f"%INIT_{name.upper()} Parameters of the Simulink model {name}.\n"
        "%   Generated by motulator_export.simulink from motulator. The model\n"
        "%   runs this script after loading and at the start of each simulation.\n"
        "%   The blocks of the control system have masks of the arguments of\n"
        "%   their classes, which refer to the structs of the control system\n"
        "%   below, as in the motulator API.\n"
    )
    script = path.with_name(f"init_{name}.m")
    script.write_text(init(comment), encoding="utf-8")
    return script.stem


def scope_indices(
    scope: list[tuple[str, list[str]]], mdl_outputs: list[str], ctrl_signals: list[str]
) -> list[list[Any]]:
    """
    Scope signals as indices in [mdl; ctrl].

    The signals are given by names with the prefix "mdl." or "ctrl.".

    """
    names = [f"mdl.{n}" for n in mdl_outputs] + [f"ctrl.{n}" for n in ctrl_signals]
    return [[title, [names.index(n) + 1 for n in signals]] for title, signals in scope]


def write_script(
    path: Path,
    block: ControlBlock,
    values: dict[str, Any],
    fields: list[tuple[str, Any]],
    builder: str,
    T_s: float | str,
    t_stop: float,
    other_functions: Sequence[SFunction | MFunction] = (),
    code: str = "c",
) -> Path:
    """
    Write the S-functions or the MATLAB functions of the blocks and the MATLAB
    script building the Simulink model.

    Parameters
    ----------
    path : Path
        Path of the model (.slx). The S-functions (or the MATLAB functions) and the
        script `build_<model>.m` are written in the same folder.
    block : ControlBlock
        Control-system block, whose C blocks are S-functions or MATLAB Function
        blocks.
    values : dict[str, Any]
        Mask parameter values of the control-system block.
    fields : list[tuple[str, Any]]
        Other fields of the struct `s`, e.g., the script "init_script".
    builder : str
        Builder function, e.g., "build_drive".
    T_s : float | str
        Sampling period (s), or its expression.
    t_stop : float
        Simulation stop time (s).
    other_functions : Sequence[SFunction | MFunction], optional
        Other S-functions (compiled by the builder) or MATLAB functions of the model,
        e.g., of the system model.
    code : str, optional
        Code of the blocks of the control system: "c" (the C port in S-functions,
        the default) or "matlab" (the MATLAB port in MATLAB Function blocks).

    Returns
    -------
    Path
        Path of the script.

    """
    if code not in CODES:
        raise ValueError(f"Unknown code {code}, not in {CODES}")
    netlist = block.netlist()
    ctrl_sfuns = sfunctions(netlist) if code == "c" else []
    ctrl_mfuns = mfunctions(netlist) if code == "matlab" else []
    # The functions called by the mask initializations (e.g., sm_reference_luts.m)
    # are written next to the model with both codes
    for f in (*ctrl_sfuns, *ctrl_mfuns, *path_functions(netlist), *other_functions):
        f.write(path.parent)
    try:
        src = Path(os.path.relpath(SIMULINK_SOURCES, path.resolve().parent))
    except ValueError:  # Different drives (Windows)
        src = SIMULINK_SOURCES
    src_parts = ", ".join(_m(p) for p in src.parts)
    control = control_fields(block, ctrl_sfuns, values, code)
    all_fields = [
        ("name", path.stem),
        ("code", code),
        *fields,
        *((f"control.{n}", v) for n, v in control),
        ("T_s", T_s),
        ("t_stop", t_stop),
    ]
    if code == "c":
        comment = (
            "% The S-functions of the blocks of the control system are compiled with\n"
            "% mex, which needs a C99 compiler with complex.h (gcc, clang, or\n"
            "% MinGW-w64, not MSVC), see mex -setup C.\n"
        )
    else:
        comment = (
            "% The blocks of the control system are MATLAB Function blocks running\n"
            "% the MATLAB port of motulator (the functions mfun_*.m of this folder,\n"
            "% copied into the blocks), so no C compiler is needed.\n"
        )
    text = (
        f"% Build the Simulink model {path.stem}.slx in the folder of this script.\n"
        "% Generated by motulator_export.simulink from motulator. The parameters\n"
        f"% are defined by the script init_{path.stem}.m.\n"
        "%\n" + comment + "\n"
        "here = fileparts(mfilename('fullpath'));\n"
        f"addpath(fullfile(here, {src_parts}));\n"
        "\n"
        "s = struct();\n"
        "s.folder = here;\n"
        + "".join(f"s.{n} = {_m(v)};\n" for n, v in all_fields)
        + "\n"
        f"{builder}(s);\n"
    )
    script = path.with_name(f"build_{path.stem}.m")
    script.write_text(text, encoding="utf-8")
    return script


def simulate(
    path: str | Path,
    t_eval: np.ndarray,
    mdl_outputs: list[str],
    ctrl_signals: list[str],
    build: bool = True,
) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
    """
    Simulate a Simulink model via the MATLAB Engine API for Python.

    Parameters
    ----------
    path : str | Path
        Path of the model (.slx).
    t_eval : ndarray
        Output times (s).
    mdl_outputs : list[str]
        Names of the signals of the output port "mdl".
    ctrl_signals : list[str]
        Names of the signals of the output port "ctrl".
    build : bool, optional
        Run the build script `build_<model>.m` first, defaults to True.

    Returns
    -------
    mdl : dict[str, ndarray]
        Signals of the system model and the time "t".
    ctrl : dict[str, ndarray]
        Monitored signals of the control system and the time "t".

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
        # Without the Engine API, pyright takes the folder matlab (the MATLAB port)
        times = np.asarray(t_eval, float).tolist()
        eng.workspace["t_eval"] = matlab.double(times)  # pyright: ignore[reportAttributeAccessIssue]
        eng.eval(
            f"out = sim({_m(path.stem)}, 'OutputOption', "
            "'SpecifiedOutputTimes', 'OutputTimes', 't_eval');",
            nargout=0,
        )
        t = np.asarray(eng.eval("out.tout")).ravel()
        values = np.asarray(eng.eval("out.yout")).T
    finally:
        eng.quit()
    n_mdl = len(mdl_outputs)
    if values.shape[0] != n_mdl + len(ctrl_signals):
        raise RuntimeError(f"Unexpected number of output signals: {values.shape}")
    mdl = {"t": t, **dict(zip(mdl_outputs, values[:n_mdl], strict=True))}
    ctrl = {"t": t, **dict(zip(ctrl_signals, values[n_mdl:], strict=True))}
    return mdl, ctrl
