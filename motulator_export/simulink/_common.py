"""
Common parts of the Simulink export: the checks of the system model, the data of the
blocks of the control systems for the library (see `_library`), and the simulation
via the MATLAB Engine API for Python.

"""

from collections.abc import Sequence
from pathlib import Path
from typing import Any

import numpy as np
from motulator.common.model._base import Model
from motulator.drive.control._base import DriveControlSystem
from motulator.grid.control._base import GridConverterControlSystem

from motulator_export._common import gate_enable
from motulator_export.plecs._common import MONITOR, ControlBlock, _fmt_mask
from motulator_export.plecs._schematic import _fmt
from motulator_export.simulink._render import (
    ASCII,
    netlist_rows,
    sfunction_params,
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
        # The package of motulator and its library on the path
        eng.addpath(str(SIMULINK_SOURCES), nargout=0)
        eng.eval("motulator.setup();", nargout=0)
        if build:
            eng.run(str(path.with_name(f"build_{path.stem}.m")), nargout=0)
        else:
            eng.load_system(str(path), nargout=0)
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
