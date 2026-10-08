"""
Test the checks of the inputs of the exports: the step signals and the supported
systems. PLECS and MATLAB are not needed.

Run from the repository root:

    pytest tests/test_export.py

"""

# %%
from math import inf
from pathlib import Path

import motulator.grid.model as grid_model
import pytest

from motulator_export import StepSignal, plecs, sampled_step, simulink
from tests.test_simulink import BASE, gfl_system, gfm_system


@pytest.mark.parametrize(
    ("time", "after"),
    [([0.1, 0.2], [1.0]), (0.1, [1.0, 2.0]), ([0.2, 0.1], [1.0, 2.0]), ([], [])],
    ids=["missing_level", "extra_level", "decreasing", "empty"],
)
def test_step_signal_errors(time: object, after: object) -> None:
    """The step times and the levels should match, the times increasing."""
    with pytest.raises(ValueError, match="step"):
        StepSignal(time, after)  # type: ignore[arg-type]


def test_step_signal() -> None:
    """Several steps sum their increments."""
    sig = StepSignal([0.1, 0.2], [1.0, 3.0], before=-1.0)
    assert [sig(t) for t in (0.0, 0.15, 0.3)] == [-1.0, 1.0, 3.0]


@pytest.mark.parametrize("T_s", [0.0, -1e-4, inf])
def test_sampled_step_errors(T_s: float) -> None:
    """The sampling period should be positive and finite."""
    with pytest.raises(ValueError, match="T_s"):
        sampled_step(StepSignal(0.1, 1.0), T_s)


def _grid_model(lcl: bool) -> grid_model.GridConverterSystem:
    """Grid converter system with an LCL or an L filter."""
    if lcl:
        ac_filter = grid_model.LCLFilter(
            L_fc=0.073 * BASE.L, L_fg=0.073 * BASE.L, C_f=0.043 * BASE.C
        )
    else:
        ac_filter = grid_model.LFilter(L_f=0.15 * BASE.L)
    return grid_model.GridConverterSystem(
        grid_model.VoltageSourceConverter(u_dc=650),
        ac_filter,
        grid_model.ThreePhaseSource(w_g=BASE.w, e_g=BASE.u),
        pwm=True,
    )


@pytest.mark.parametrize("gfl", [True, False], ids=["gfl_l_filter", "gfm_lcl_filter"])
def test_unsupported_grid_filter(tmp_path: Path, gfl: bool) -> None:
    """Grid-following control needs the LCL filter (the PCC voltage measured after
    it), and grid-forming control the L filter."""
    ctrl = (gfl_system() if gfl else gfm_system(False))[0]()[1]
    mdl = _grid_model(lcl=not gfl)
    refs = {"q_g_ref": StepSignal(0.04, 4e3)} if gfl else {"v_c_ref": BASE.u}
    for tool, path in ((plecs, "grid.plecs"), (simulink, "grid.slx")):
        with pytest.raises(NotImplementedError, match="LCLFilter"):
            tool.grid.write_model(
                tmp_path / path, mdl, ctrl, 0.1, StepSignal(0.02, 5e3), **refs
            )
    assert not list(tmp_path.iterdir())  # Nothing is written
