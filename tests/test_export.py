"""
Test the checks of the inputs of the exports: the step signals and the supported
systems. PLECS and MATLAB are not needed.

Run from the repository root:

    pytest tests/test_export.py

"""

# %%
import re
from math import inf
from pathlib import Path
from typing import Any

import motulator.grid.model as grid_model
import numpy as np
import pytest
from motulator.drive import model

from motulator_export import StepSignal, plecs, sampled_step, simulink
from motulator_export.plecs._drive import _add_mechanics
from motulator_export.plecs._schematic import _Schematic
from tests.test_simulink import (
    BASE,
    IM_SPEED,
    SM_SPEED,
    gfl_system,
    gfm_system,
    im_system,
    sm_system,
)


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


@pytest.mark.parametrize("gfl", [True, False], ids=["gfl", "gfm"])
def test_missing_reference(tmp_path: Path, gfl: bool) -> None:
    """Grid-following control needs q_g_ref, and grid-forming control v_c_ref, which
    are checked before anything is written, so an existing model is kept."""
    ctrl = (gfl_system() if gfl else gfm_system(False))[0]()[1]
    for tool, path in ((plecs, "grid.plecs"), (simulink, "grid.slx")):
        with pytest.raises(ValueError, match="q_g_ref" if gfl else "v_c_ref"):
            tool.grid.write_model(
                tmp_path / path, _grid_model(lcl=gfl), ctrl, 0.1, StepSignal(0.02, 5e3)
            )
    assert not list(tmp_path.iterdir())  # Nothing is written


@pytest.mark.parametrize("several", [False, True], ids=["one_step", "two_steps"])
def test_load_torque_steps(several: bool) -> None:
    """In PLECS, several steps of the load torque should be summed by a Gain block in
    front of the load, as the steps of the references."""
    sch = _Schematic()
    tau_L = StepSignal([0.1, 0.2], [3.0, 7.0]) if several else StepSignal(0.1, 3.0)
    _add_mechanics(sch, tau_L, inertia=True)
    load = [(("Load", 2), [])]
    if several:
        assert sch.connections[(("tau_L", 1), "Signal")] == [(("tau_L sum", 1), [])]
        assert sch.connections[(("tau_L sum", 2), "Signal")] == load
        gain = next(item for item in sch.items if '"tau_L sum"' in item)
        assert re.search(r'Variable\s+"K"\s+Value\s+"\[1\.0 1\.0\]"', gain), gain
    else:
        assert sch.connections[(("tau_L", 1), "Signal")] == load


def _drive(sm: bool) -> tuple[Any, Any, dict[str, float]]:
    """Drive with carrier comparison, its control system, and its speed controller."""
    if sm:
        mdl, ctrl = sm_system(True, 0.0)[0]()
    else:
        mdl, ctrl = im_system(True, 0.0, 0.0)[0]()
    drive = model.Drive(mdl.machine, mdl.mechanics, mdl.converter, pwm=True)
    return drive, ctrl, SM_SPEED if sm else IM_SPEED


DRIVE_REFS = (StepSignal(0.1, 50.0), StepSignal(0.2, 5.0), 0.3)


def test_initial_mechanics(tmp_path: Path) -> None:
    """The initial speed and angle of the mechanics should be exported."""
    mdl, ctrl, speed = _drive(sm=True)
    mdl.mechanics.state.w_M = 50.0 + 0j
    mdl.mechanics.state.exp_j_theta_M = np.exp(0.25j)
    # The rotor angle of the machine model corresponds to that of the mechanics
    mdl.machine.state.exp_j_theta_m = np.exp(0.25j * mdl.machine.par.n_p)
    plecs.sm.write_model(tmp_path / "drive.plecs", mdl, ctrl, *DRIVE_REFS, speed)
    simulink.sm.write_model(tmp_path / "drive.slx", mdl, ctrl, *DRIVE_REFS, speed)
    model_plecs = (tmp_path / "drive.plecs").read_text(encoding="utf-8")
    for text in (model_plecs, (tmp_path / "init_drive.m").read_text(encoding="utf-8")):
        assert "mechanics.w_M0 = 50;" in text
        assert "mechanics.theta_M0 = 0.25;" in text
    assert '"mechanics.w_M0"' in model_plecs  # The initial speed of the machine


@pytest.mark.parametrize(
    "case", ["sm_flux", "sm_angle", "im_flux", "grid_current", "grid_angle"]
)
def test_unsupported_initial_states(tmp_path: Path, case: str) -> None:
    """The states other than those of the mechanics and the capacitor voltage of the
    LCL filter should be at their initial values, since they are not exported."""
    if case.startswith(("sm", "im")):
        mdl, ctrl, speed = _drive(sm=case.startswith("sm"))
        if case == "sm_flux":
            mdl.machine.state.psi_s_dq = 0.4 + 0.1j
        elif case == "sm_angle":  # Not n_p times the angle of the mechanics
            mdl.mechanics.state.exp_j_theta_M = 1j
        else:
            mdl.machine.state.psi_r_ab = 0.1 + 0j
        match = {"sm_flux": "psi_s_dq", "sm_angle": "exp_j_theta_m"}.get(
            case, "psi_r_ab"
        )
        for tool, path in ((plecs, "drive.plecs"), (simulink, "drive.slx")):
            module = tool.sm if case.startswith("sm") else tool.im
            with pytest.raises(NotImplementedError, match=match):
                module.write_model(tmp_path / path, mdl, ctrl, *DRIVE_REFS, speed)
    else:
        ctrl = gfl_system()[0]()[1]
        mdl: Any = _grid_model(lcl=True)
        if case == "grid_current":
            mdl.ac_filter.state.i_g_ab = 1 + 0j
        else:
            mdl.ac_source.state.exp_j_theta_g = 1j
        match = "i_g_ab" if case == "grid_current" else "exp_j_theta_g"
        for tool, path in ((plecs, "grid.plecs"), (simulink, "grid.slx")):
            with pytest.raises(NotImplementedError, match=match):
                tool.grid.write_model(
                    tmp_path / path,
                    mdl,
                    ctrl,
                    0.1,
                    StepSignal(0.02, 5e3),
                    q_g_ref=StepSignal(0.04, 4e3),
                )
    assert not list(tmp_path.iterdir())  # Nothing is written


def test_unsupported_k_comp(tmp_path: Path) -> None:
    """Only the angle advance k_comp = 1.5 of the continuous-time controller designs
    is supported."""
    ctrl = gfm_system(False)[0]()[1]
    ctrl.k_comp = 0.0
    for tool, path in ((plecs, "grid.plecs"), (simulink, "grid.slx")):
        with pytest.raises(NotImplementedError, match="k_comp"):
            tool.grid.write_model(
                tmp_path / path,
                _grid_model(lcl=False),
                ctrl,
                0.1,
                StepSignal(0.02, 5e3),
                v_c_ref=BASE.u,
            )
    assert not list(tmp_path.iterdir())  # Nothing is written
