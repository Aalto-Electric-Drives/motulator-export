"""
Test the checks of the inputs of the exports: the step signals, the references, and
the supported systems. PLECS and MATLAB are not needed.

Run from the repository root:

    pytest tests/test_export.py

"""

# %%
import re
from math import inf, pi
from pathlib import Path
from typing import Any

import motulator.drive.control.sm as sm_control
import motulator.drive.gradnet as gn
import motulator.grid.control as grid_control
import motulator.grid.model as grid_model
import numpy as np
import pytest
from motulator.drive import model

from motulator_export import StepSignal, plecs, simulink
from motulator_export.plecs._common import sampled_step
from motulator_export.plecs._drive import _add_mechanics
from motulator_export.plecs._schematic import _Schematic
from tests.test_simulink import BASE, CURRENT_MAP, im_system, sm_system

TOOLS = [(plecs, "model.plecs"), (simulink, "model.slx")]
T_STOP = 0.3


@pytest.mark.parametrize(
    ("time", "after"),
    [
        ([0.1, 0.2], [1.0]),
        (0.1, [1.0, 2.0]),
        ([0.2, 0.1], [1.0, 2.0]),
        ([], []),
        (inf, 1.0),
        (0.1, np.nan),
    ],
    ids=["missing_level", "extra_level", "decreasing", "empty", "inf_time", "nan"],
)
def test_step_signal_errors(time: object, after: object) -> None:
    """The step times and the levels should match, the times increasing, and all
    finite."""
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


@pytest.mark.parametrize("time", [inf, np.nan, 1e9])
def test_sampled_step_time_errors(time: float) -> None:
    """The step time should be finite and below the wrap-around of the controller
    time (1e9 s), which the sampling repeats."""
    with pytest.raises(ValueError, match="step times"):
        sampled_step(StepSignal(time, 1.0), 1e-4)


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


def _grid_ctrl(gfl: bool, refs: bool = True) -> grid_control.GridConverterControlSystem:
    """
    Grid-following or grid-forming control system, with the active power reference
    and, if `refs`, the reactive power reference or the AC voltage reference.
    """
    if gfl:
        cfg = grid_control.CurrentVectorControllerCfg(
            i_max=1.5 * BASE.i, L=0.073 * BASE.L, T_s=100e-6
        )
        ctrl = grid_control.GridConverterControlSystem(
            grid_control.CurrentVectorController(cfg)
        )
        if refs:
            ctrl.set_reactive_power_ref(StepSignal(0.04, 4e3))
    else:
        gfm_cfg = grid_control.ObserverBasedGridFormingControllerCfg(
            i_max=1.3 * BASE.i,
            L=0.35 * BASE.L,
            R=0.05 * BASE.Z,
            R_a=0.2 * BASE.Z,
            u_nom=BASE.u,
            w_nom=BASE.w,
        )
        ctrl = grid_control.GridConverterControlSystem(
            grid_control.ObserverBasedGridFormingController(gfm_cfg)
        )
        if refs:
            ctrl.set_ac_voltage_ref(BASE.u)
    ctrl.set_power_ref(StepSignal(0.02, 5e3))
    return ctrl


def _grid(gfl: bool) -> tuple[Any, Any]:
    """Grid converter system and its control system with their usual filter."""
    return _grid_model(lcl=gfl), _grid_ctrl(gfl)


@pytest.mark.parametrize("gfl", [True, False], ids=["gfl_l_filter", "gfm_lcl_filter"])
def test_unsupported_grid_filter(tmp_path: Path, gfl: bool) -> None:
    """Grid-following control needs the LCL filter (the PCC voltage measured after
    it), and grid-forming control the L filter."""
    mdl, ctrl = _grid_model(lcl=not gfl), _grid_ctrl(gfl)
    for tool, path in TOOLS:
        with pytest.raises(NotImplementedError, match="LCLFilter"):
            tool.write_model(tmp_path / path, mdl, ctrl, 0.1)
    assert not list(tmp_path.iterdir())  # Nothing is written


@pytest.mark.parametrize("gfl", [True, False], ids=["gfl", "gfm"])
def test_missing_reference(tmp_path: Path, gfl: bool) -> None:
    """Grid-following control needs the reactive power reference, and grid-forming
    control the AC voltage reference, which are checked before anything is written,
    so an existing model is kept."""
    mdl, ctrl = _grid_model(lcl=gfl), _grid_ctrl(gfl, refs=False)
    match = "reactive power reference" if gfl else "AC voltage reference"
    for tool, path in TOOLS:
        with pytest.raises(ValueError, match=match):
            tool.write_model(tmp_path / path, mdl, ctrl, 0.1)
    assert not list(tmp_path.iterdir())  # Nothing is written


def _drive(sm: bool, dc_bus: str = "") -> tuple[Any, Any]:
    """
    Drive with carrier comparison and its control system, with the speed reference
    and the load torque. The DC bus is stiff, or a capacitor (`dc_bus` "capacitor")
    or a diode bridge with LC ("diode") feeding the converter.
    """
    if sm:
        mdl, ctrl = sm_system(True, 0.0)[0]()
    else:
        mdl, ctrl = im_system(True, 0.0, 0.0)[0]()
    converter = mdl.converter
    bus = None
    if dc_bus:
        converter = model.VoltageSourceConverter()
        if dc_bus == "diode":
            bus = model.DiodeBridgeDCBus(
                C_dc=235e-6, L_dc=2e-3, U_g=400, f_g=50, converters=[converter]
            )
        else:
            bus = model.DCBus(C_dc=235e-6, u_dc=540, converters=[converter])
    drive = model.Drive(mdl.machine, mdl.mechanics, converter, pwm=True, dc_bus=bus)
    ctrl.set_speed_ref(StepSignal(0.1, 50.0))
    drive.mechanics.set_external_load_torque(StepSignal(0.2, 5.0))
    return drive, ctrl


@pytest.mark.parametrize("system", ["sm", "im", "grid"])
def test_references(tmp_path: Path, system: str) -> None:
    """
    The references and the load torque are read from the control system and the
    mechanics: constants (Constant blocks) or step signals (Step blocks, the
    references of the control system switching at the same sampling instants as in
    motulator); a function of time is not supported, and nothing is written.
    """
    if system == "grid":
        mdl, ctrl = _grid(gfl=False)
        ctrl.set_power_ref(2e3)  # Constant
        ctrl.set_ac_voltage_ref(StepSignal(0.03, 0.9 * BASE.u, before=BASE.u))
        set_bad = ctrl.set_power_ref
    else:
        mdl, ctrl = _drive(sm=system == "sm")
        ctrl.set_speed_ref(50.0)  # Constants
        mdl.mechanics.set_external_load_torque(3.0)
        set_bad = mdl.mechanics.set_external_load_torque
    plecs.write_model(tmp_path / "model.plecs", mdl, ctrl, 0.1)
    simulink.write_model(tmp_path / "model.slx", mdl, ctrl, 0.1)
    text = (tmp_path / "model.plecs").read_text(encoding="utf-8")
    script = (tmp_path / "build_model.m").read_text(encoding="utf-8")
    constant = r'Type\s+Constant\s+Name\s+"%s"[^}]*Value\s+"%s"'
    if system == "grid":
        # The step of the AC voltage reference switches at the sample 240 (0.03 s)
        # of T_s = 125 us, i.e., halfway between the samples 239 and 240
        step = r'Type\s+Step\s+Name\s+"v_c_ref"[^}]*"Time"\s+Value\s+"0\.0299375'
        assert re.search(step, text)
        assert re.search(constant % ("p_g_ref", r"2000\.0"), text)
        assert "'v_c_ref', 'step', [0.0299375" in script
        assert "'p_g_ref', 'constant', 2000.0" in script
    else:
        assert re.search(constant % ("w_M_ref", r"50\.0"), text)
        assert re.search(constant % ("tau_L", r"3\.0"), text)
        assert "s.w_M_ref = 50.0;" in script and "s.tau_L = 3.0;" in script
    files = set(tmp_path.iterdir())
    for bad, error in [(lambda t: 1.0, NotImplementedError), (inf, ValueError)]:
        set_bad(bad)
        for tool, path in TOOLS:
            with pytest.raises(
                error, match="StepSignal" if bad is not inf else "finite"
            ):
                tool.write_model(tmp_path / f"other_{path}", mdl, ctrl, 0.1)
    assert set(tmp_path.iterdir()) == files  # Nothing is written


def test_sampled_references(tmp_path: Path) -> None:
    """
    The step signals of the control system (the speed reference) switch at the same
    sampling instant as in motulator (halfway between the samples), while the load
    torque of the mechanics acts in continuous time at its step time.
    """
    mdl, ctrl = _drive(sm=True)  # Steps at 0.1 s and 0.2 s, T_s = 125 us
    plecs.write_model(tmp_path / "model.plecs", mdl, ctrl, T_STOP)
    simulink.write_model(tmp_path / "model.slx", mdl, ctrl, T_STOP)
    text = (tmp_path / "model.plecs").read_text(encoding="utf-8")
    script = (tmp_path / "build_model.m").read_text(encoding="utf-8")
    step = r'Type\s+Step\s+Name\s+"%s"[^}]*"Time"\s+Value\s+"%s"'
    assert re.search(step % ("w_M_ref", r"0\.0999375"), text)
    assert re.search(step % ("tau_L", r"0\.2"), text)
    assert "s.w_M_ref = [0.0999375 0.0 50.0];" in script
    assert "s.tau_L = [0.2 0.0 5.0];" in script


def test_unsupported_system(tmp_path: Path) -> None:
    """Only the drives and the grid converter systems with their control systems are
    supported, and the code of the Simulink blocks is "c" or "matlab"; nothing is
    written otherwise."""
    mdl, ctrl = _drive(sm=True)
    for tool, path in TOOLS:
        with pytest.raises(NotImplementedError, match="GridConverterSystem"):
            tool.write_model(tmp_path / path, mdl.mechanics, ctrl, 0.1)
        with pytest.raises(NotImplementedError, match="VectorControlSystem"):
            tool.write_model(tmp_path / path, mdl, _grid_ctrl(gfl=True), 0.1)
    with pytest.raises(ValueError, match="code"):
        simulink.write_model(tmp_path / "model.slx", mdl, ctrl, 0.1, code="python")
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


def test_initial_mechanics(tmp_path: Path) -> None:
    """The initial speed and angle of the mechanics should be exported."""
    mdl, ctrl = _drive(sm=True)
    mdl.mechanics.state.w_M = 50.0 + 0j
    mdl.mechanics.state.theta_M = 0.25 + 0j
    # The rotor angle of the machine model corresponds to that of the mechanics,
    # modulo 2*pi
    mdl.machine.state.theta_m = complex(0.25 * mdl.machine.par.n_p - 2 * pi)
    plecs.write_model(tmp_path / "drive.plecs", mdl, ctrl, T_STOP)
    simulink.write_model(tmp_path / "drive.slx", mdl, ctrl, T_STOP)
    model_plecs = (tmp_path / "drive.plecs").read_text(encoding="utf-8")
    for text in (model_plecs, (tmp_path / "init_drive.m").read_text(encoding="utf-8")):
        assert "mechanics.w_M0 = 50;" in text
        assert "mechanics.theta_M0 = 0.25;" in text
    assert '"mechanics.w_M0"' in model_plecs  # The initial speed of the machine


def test_modified_speed_controller(tmp_path: Path) -> None:
    """The gains of the speed controller should be those of its arguments, which the
    exported controller has; modified gains are rejected before anything is written."""
    mdl, ctrl = _drive(sm=True)
    ctrl.speed_ctrl.k_p *= 2
    for tool, path in TOOLS:
        with pytest.raises(NotImplementedError, match="gains"):
            tool.write_model(tmp_path / path, mdl, ctrl, T_STOP)
    assert not list(tmp_path.iterdir())  # Nothing is written


@pytest.mark.parametrize("alpha_i", [None, 10.0], ids=["default", "given"])
def test_speed_controller(tmp_path: Path, alpha_i: float | None) -> None:
    """The arguments of the speed controller should be exported from the controller,
    the default integral action bandwidth as [] (None), so that the exported
    controller computes the same gains as motulator."""
    mdl, ctrl = _drive(sm=True)
    ctrl.speed_ctrl = sm_control.SpeedController(
        J=0.02, alpha_s=30.0, alpha_i=alpha_i, tau_M_max=12.0
    )
    plecs.write_model(tmp_path / "drive.plecs", mdl, ctrl, T_STOP)
    simulink.write_model(tmp_path / "drive.slx", mdl, ctrl, T_STOP)
    for name in ("drive.plecs", "init_drive.m"):
        text = (tmp_path / name).read_text(encoding="utf-8")
        for line in (
            "speed_ctrl.J = 0.02;",
            "speed_ctrl.alpha_s = 30;",
            "speed_ctrl.alpha_i = [];"
            if alpha_i is None
            else "speed_ctrl.alpha_i = 10;",
            "speed_ctrl.tau_M_max = 12;",
        ):
            assert line in text, (name, line)


@pytest.mark.parametrize(
    "case", ["sm_flux", "sm_angle", "im_flux", "grid_current", "grid_angle"]
)
def test_unsupported_initial_states(tmp_path: Path, case: str) -> None:
    """The states other than those of the mechanics and the capacitor voltage of the
    LCL filter should be at their initial values, since they are not exported."""
    if case.startswith(("sm", "im")):
        mdl, ctrl = _drive(sm=case.startswith("sm"))
        if case == "sm_flux":
            mdl.machine.state.psi_s_dq = 0.4 + 0.1j
        elif case == "sm_angle":  # Not n_p times the angle of the mechanics
            mdl.mechanics.state.theta_M = complex(0.5 * pi)
        else:
            mdl.machine.state.psi_r_ab = 0.1 + 0j
        match = {"sm_flux": "psi_s_dq", "sm_angle": "theta_m"}.get(case, "psi_r_ab")
    else:
        mdl, ctrl = _grid(gfl=True)
        if case == "grid_current":
            mdl.ac_filter.state.i_g_ab = 1 + 0j
        else:
            mdl.ac_source.state.exp_j_theta_g = 1j
        match = "i_g_ab" if case == "grid_current" else "exp_j_theta_g"
    for tool, path in TOOLS:
        with pytest.raises(NotImplementedError, match=match):
            tool.write_model(tmp_path / path, mdl, ctrl, 0.1)
    assert not list(tmp_path.iterdir())  # Nothing is written


@pytest.mark.parametrize("case", ["sm_flux", "mechanics_speed", "lcl_voltage"])
def test_non_finite_states(tmp_path: Path, case: str) -> None:
    """The states should be finite (e.g., not NaN after a failed simulation), also
    those that are exported."""
    if case == "lcl_voltage":
        mdl, ctrl = _grid(gfl=True)
        mdl.ac_filter.state.u_f_ab = complex(np.nan, 0)
        match = "u_f_ab"
    else:
        mdl, ctrl = _drive(sm=True)
        if case == "sm_flux":
            mdl.machine.state.psi_s_dq = complex(np.nan, 0)
        else:
            mdl.mechanics.state.w_M = complex(np.nan, 0)
        match = "psi_s_dq" if case == "sm_flux" else "w_M"
    for tool, path in TOOLS:
        with pytest.raises(ValueError, match=f"{match} is not finite"):
            tool.write_model(tmp_path / path, mdl, ctrl, 0.1)
    assert not list(tmp_path.iterdir())  # Nothing is written


@pytest.mark.parametrize("sm", [True, False], ids=["sm", "im"])
@pytest.mark.parametrize("dc_bus", ["capacitor", "diode"])
def test_dc_bus(tmp_path: Path, sm: bool, dc_bus: str) -> None:
    """The DC-bus models of the drives should be exported to PLECS, starting from the
    DC-bus voltage of their state, and rejected in Simulink (only a stiff DC bus)."""
    mdl, ctrl = _drive(sm, dc_bus)
    mdl.dc_bus.state.u_dc = 500.0 + 0j
    plecs.write_model(tmp_path / "drive.plecs", mdl, ctrl, T_STOP)
    text = (tmp_path / "drive.plecs").read_text(encoding="utf-8")
    assert "dc_bus.u_dc0 = 500;" in text
    assert '"dc_bus.C_dc"' in text and "converter.u_dc" not in text
    assert ('"dc_bus.L_dc"' in text) == (dc_bus == "diode")
    with pytest.raises(NotImplementedError, match="DC-bus model"):
        simulink.write_model(tmp_path / "drive.slx", mdl, ctrl, T_STOP)
    assert not (tmp_path / "drive.slx").exists()


@pytest.mark.parametrize("case", ["shared", "external_current", "inductor_current"])
def test_unsupported_dc_bus(tmp_path: Path, case: str) -> None:
    """The DC bus should feed only the converter of the drive, without an external
    current, and the states of the diode bridge should be at their initial values."""
    mdl, ctrl = _drive(True, "diode" if case == "inductor_current" else "cap")
    if case == "shared":  # E.g., with an active front end
        mdl.dc_bus.converters.append(model.VoltageSourceConverter())
        match = "shared"
    elif case == "external_current":
        mdl.dc_bus.set_external_dc_current(1.0)
        match = "External DC current"
    else:
        mdl.dc_bus.state.i_L = 1.0 + 0j
        match = "i_L"
    with pytest.raises(NotImplementedError, match=match):
        plecs.write_model(tmp_path / "drive.plecs", mdl, ctrl, T_STOP)
    assert not list(tmp_path.iterdir())  # Nothing is written


def test_unsupported_grid_dc_bus(tmp_path: Path) -> None:
    """Only the stiff DC bus is supported in the grid converter systems."""
    ref, ctrl = _grid(gfl=True)
    converter = grid_model.VoltageSourceConverter()
    dc_bus = grid_model.DCBus(C_dc=1e-3, u_dc=650, converters=[converter])
    mdl = grid_model.GridConverterSystem(
        converter, ref.ac_filter, ref.ac_source, pwm=True, dc_bus=dc_bus
    )
    for tool, path in TOOLS:
        with pytest.raises(NotImplementedError, match="DC-bus model"):
            tool.write_model(tmp_path / path, mdl, ctrl, 0.1)
    assert not list(tmp_path.iterdir())  # Nothing is written


def test_unsupported_k_comp(tmp_path: Path) -> None:
    """Only the angle advance k_comp = 1.5 of the continuous-time controller designs
    is supported."""
    mdl, ctrl = _grid(gfl=False)
    ctrl.k_comp = 0.0
    for tool, path in TOOLS:
        with pytest.raises(NotImplementedError, match="k_comp"):
            tool.write_model(tmp_path / path, mdl, ctrl, 0.1)
    assert not list(tmp_path.iterdir())  # Nothing is written


def _im_drive(
    t_d: float, u_th: float, single_update: bool, delay: int | None = None
) -> tuple[Any, Any]:
    """
    Induction machine drive with carrier comparison, the dead time and the threshold
    voltage, and single or double update, and its control system compensating for
    them. The delay defaults to one sampling period.
    """
    mdl, ctrl = im_system(True, t_d, 0.0, u_th, single_update)[0]()
    if delay is None:
        delay = 2 if single_update else 1
    drive = model.Drive(
        mdl.machine,
        mdl.mechanics,
        mdl.converter,
        pwm=True,
        delay=delay,
        single_update=single_update,
    )
    ctrl.set_speed_ref(StepSignal(0.1, 50.0))
    drive.mechanics.set_external_load_torque(StepSignal(0.2, 5.0))
    return drive, ctrl


def test_threshold_voltage_single_update(tmp_path: Path) -> None:
    """The threshold voltage should be exported to PLECS as the forward voltages of
    the IGBT converter with parasitics, and single update as the carrier period T_s,
    both also as the parameters of the PWM of the control system. In Simulink, they
    are supported in the control system but not in the system model."""
    mdl, ctrl = _im_drive(2e-6, 1.0, True)
    plecs.write_model(tmp_path / "drive.plecs", mdl, ctrl, T_STOP)
    text = (tmp_path / "drive.plecs").read_text(encoding="utf-8")
    assert (
        r'"Components/Electrical/Converters/Detailed\\n2-Level\\nIGBT\\nConv."' in text
    )
    for var in ("Vf_igbt", "Vf_d"):
        assert re.search(rf'Variable\s+"{var}"\s+Value\s+"converter.u_th"', text)
    assert re.search(r'Variable\s+"fc"\s+Value\s+"1/cfg.T_s"', text)
    for line in ("converter.u_th = 1;", "pwm.u_th = 1;", "pwm.single_update = 1;"):
        assert line in text
    with pytest.raises(NotImplementedError, match="threshold voltage"):
        simulink.write_model(tmp_path / "drive.slx", mdl, ctrl, T_STOP)
    with pytest.raises(NotImplementedError, match="Single update"):
        single = _im_drive(0.0, 0.0, True)[0]
        simulink.write_model(tmp_path / "drive.slx", single, ctrl, T_STOP)
    # The control system with single update and the threshold voltage, with the
    # system model with double update
    mdl, _ = _im_drive(0.0, 0.0, False)
    simulink.write_model(tmp_path / "drive.slx", mdl, ctrl, T_STOP)
    text = (tmp_path / "init_drive.m").read_text(encoding="utf-8")
    assert "pwm.u_th = 1;" in text and "pwm.single_update = 1;" in text


@pytest.mark.parametrize("single_update", [False, True], ids=["double", "single"])
def test_unsupported_delay(tmp_path: Path, single_update: bool) -> None:
    """The computational delay should be one sampling period: one half carrier period
    with double update and two with single update."""
    mdl, ctrl = _im_drive(0.0, 0.0, single_update, delay=1 if single_update else 2)
    for tool, path in TOOLS:
        with pytest.raises(NotImplementedError, match="computational delay"):
            tool.write_model(tmp_path / path, mdl, ctrl, T_STOP)
    assert not list(tmp_path.iterdir())  # Nothing is written


@pytest.mark.parametrize("case", ["sm", "sm_sign", "grid"])
def test_threshold_voltage(tmp_path: Path, case: str) -> None:
    """The threshold voltage of the converter is supported in the drives (with
    `sign=np.sign`) of the PLECS export, and not in the grid converter systems and
    the Simulink export."""
    if case.startswith("sm"):
        mdl, ctrl = _drive(sm=True)
        mdl.converter.u_th = 1.0
        if case == "sm":
            plecs.write_model(tmp_path / "drive.plecs", mdl, ctrl, T_STOP)
            text = (tmp_path / "drive.plecs").read_text(encoding="utf-8")
            assert "converter.u_th = 1;" in text
            (tmp_path / "drive.plecs").unlink()
        else:
            mdl.converter.sign = lambda i: np.tanh(i / 0.1)
    else:
        mdl, ctrl = _grid(gfl=True)
        mdl.converter.u_th = 1.0
    for tool, path in TOOLS:
        if tool is plecs and case == "sm":
            continue
        with pytest.raises(NotImplementedError, match="threshold voltage"):
            tool.write_model(tmp_path / path, mdl, ctrl, T_STOP)
    assert not list(tmp_path.iterdir())  # Nothing is written


@pytest.mark.parametrize("system", ["sm", "grid"])
def test_gate_enable(tmp_path: Path, system: str) -> None:
    """
    The gate enable of the control system (`set_enable`) should be exported to PLECS
    as the input enable of the control system, also sampled to block the gates of the
    converter, and rejected in Simulink (whose system model does not block the
    gates). Only a constant and a StepSignal of 0 and 1 are supported.
    """
    enable = StepSignal([0.01, 0.2], [1.0, 0.0], before=0.0)
    mdl, ctrl = _drive(sm=True) if system == "sm" else _grid(gfl=True)
    ctrl.set_enable(enable)
    plecs.write_model(tmp_path / "model.plecs", mdl, ctrl, T_STOP)
    text = (tmp_path / "model.plecs").read_text(encoding="utf-8")
    assert re.search(r'Name\s+"enable"', text)  # The Step block of the gate enable
    for name in ("Goto enable", "From enable", "Sampled enable", "Gate enable"):
        assert f'"{name}"' in text
    assert "2-Level\\\\nIGBT\\\\nConv." in text  # Blocking the gates
    with pytest.raises(NotImplementedError, match="Gate blocking"):
        simulink.write_model(tmp_path / "model.slx", mdl, ctrl, T_STOP)
    assert not (tmp_path / "model.slx").exists()
    for bad, error in [
        (lambda t: t > 0.01, NotImplementedError),
        (StepSignal(0.01, 2.0, before=0.0), ValueError),
        (0.5, NotImplementedError),
    ]:
        ctrl.set_enable(bad)
        with pytest.raises(error):
            plecs.write_model(tmp_path / "other.plecs", mdl, ctrl, T_STOP)
    assert not (tmp_path / "other.plecs").exists()


def test_unsupported_single_update_grid(tmp_path: Path) -> None:
    """Single update is supported in the system model of the PLECS export only."""
    ref, ctrl = _grid(gfl=True)
    mdl = grid_model.GridConverterSystem(
        ref.converter,
        ref.ac_filter,
        ref.ac_source,
        pwm=True,
        delay=2,
        single_update=True,
    )
    plecs.write_model(tmp_path / "grid.plecs", mdl, ctrl, 0.1)
    text = (tmp_path / "grid.plecs").read_text(encoding="utf-8")
    assert re.search(r'Variable\s+"fc"\s+Value\s+"1/cfg.T_s"', text)
    with pytest.raises(NotImplementedError, match="Single update"):
        simulink.write_model(tmp_path / "grid.slx", mdl, ctrl, 0.1)
    assert not (tmp_path / "grid.slx").exists()


def test_unsupported_gate_blocking_gradnet(tmp_path: Path) -> None:
    """The gate blocking is not supported with the GradNet machine model, whose
    controlled current sources would toggle the diodes of a blocked leg in PLECS."""
    current_map = gn.CurrentMapWithHarmonics(gn.load_gradnet(CURRENT_MAP))
    par = model.SpatialSaturatedSynchronousMachinePars(
        n_p=2, R_s=0.63, magnetic_map_fcn=current_map
    )
    mdl = model.Drive(
        model.SynchronousMachine(par),
        model.MechanicalSystem(J=0.015),
        model.VoltageSourceConverter(u_dc=540),
        pwm=True,
    )
    ctrl = sm_system(True, 0.0)[0]()[1]
    ctrl.set_speed_ref(StepSignal(0.1, 50.0))
    ctrl.set_enable(StepSignal(0.01, 1.0, before=0.0))
    with pytest.raises(NotImplementedError, match="GradNet"):
        plecs.write_model(tmp_path / "drive.plecs", mdl, ctrl, T_STOP)
    assert not list(tmp_path.iterdir())  # Nothing is written
