"""
Test the control systems, in which each class of motulator is a block.

The C blocks are S-functions, compiled with gcc against the mock of the Simulink
API and executed as Simulink executes the model (`tests/netlist.py`). Their
parameters are computed from the model initialization (`init_script`) through the
masks, as Simulink and PLECS compute them (`tests/matlab.py`). The control systems
are compared with motulator in closed-loop simulations and with the monolithic
control systems (`tests/monolithic.py`), which should give the same results exactly.
The layouts of the subsystems are checked for crossing wires. MATLAB and PLECS are
not needed.

Run from the repository root:

    pytest tests/test_control.py

"""

# %%
import ctypes
import re
from collections.abc import Callable, Iterator
from typing import Any

import motulator.drive.control.im as im_control
import motulator.drive.control.sm as sm_control
import numpy as np
import pytest
from motulator.common.utils import complex2abc
from motulator.drive import model
from motulator.drive.control._base import VectorControlSystem
from motulator.grid import control as grid_control
from motulator.grid import model as grid_model
from motulator.grid.control._gfm_observer import Observer

from motulator_export.plecs import (
    _grid,
    _grid_control,
    _im,
    _im_control,
    _im_vhz_control,
    _sm,
    _sm_control,
    _sm_vhz_control,
)
from motulator_export.plecs._common import ControlBlock
from motulator_export.plecs._netlist import PLECS, SIMULINK, Subsystem, layout, problems
from tests import monolithic
from tests.c_port import arr
from tests.matlab import block_params
from tests.monolithic import MonolithicBlock, sfunction_params
from tests.netlist import Netlist
from tests.sfunctions import (
    GFL_INPUTS,
    GFM_INPUTS,
    IM_INPUTS,
    SM_INPUTS,
    VHZ_INPUTS,
    check_dc_bus_limit,
    compare,
    compile_sfunction,
    drive_sfun,
    gradnet_inputs,
    grid_sfun,
    start,
)
from tests.systems import (
    BASE,
    IM_PAR,
    IM_SPEED,
    SM_PAR,
    SM_SPEED,
    SM_VHZ_CASES,
    SM_VHZ_IDS,
    VHZ_CASES,
    VHZ_IDS,
    VHZ_RTOL,
    gfl_system,
    gfm_system,
    gradnet_control_system,
    im_system,
    sm_system,
    sm_vhz_system,
    vhz_system,
)

# Control systems: the block and its monolithic reference
SYSTEMS: dict[str, tuple[ControlBlock, MonolithicBlock]] = {
    "fvc": (_sm_control.FVC_BLOCK, monolithic.FVC),
    "cvc": (_im_control.CVC_BLOCK, monolithic.CVC),
    "vhz": (_im_vhz_control.VHZ_BLOCK, monolithic.VHZ),
    "smvhz": (_sm_vhz_control.VHZ_BLOCK, monolithic.SM_VHZ),
    "gfl": (_grid_control.GFL_BLOCK, monolithic.GFL),
    "gfm": (_grid_control.GFM_BLOCK, monolithic.GFM),
}


def subsystems(sub: Subsystem) -> Iterator[Subsystem]:
    """A subsystem and its subsystems."""
    yield sub
    for b in sub.blocks:
        if isinstance(b, Subsystem):
            yield from subsystems(b)


@pytest.mark.parametrize("geo", [PLECS, SIMULINK], ids=["plecs", "simulink"])
@pytest.mark.parametrize(
    "sub",
    [s for block, _ in SYSTEMS.values() for s in subsystems(block.netlist())],
    ids=lambda s: s.name,
)
def test_layout(sub: Subsystem, geo: Any) -> None:
    """The wires should not cross each other or go through blocks."""
    assert problems(layout(sub, geo)) == []


# %%
class Compiled:
    """The compiled control systems and their references, compiled when needed."""

    def __init__(self, tmp_path_factory: pytest.TempPathFactory) -> None:
        self.tmp = tmp_path_factory
        self.cache: dict[str, tuple[Netlist, ctypes.CDLL]] = {}

    def __call__(self, key: str) -> tuple[Netlist, ctypes.CDLL]:
        if key not in self.cache:
            block, mono = SYSTEMS[key]
            net = Netlist(block.netlist(), self.tmp.mktemp(key), compile_sfunction)
            ref = compile_sfunction(mono, self.tmp.mktemp(f"{key}_monolithic"))
            self.cache[key] = net, ref
        return self.cache[key]


@pytest.fixture(scope="module")
def compiled(tmp_path_factory: pytest.TempPathFactory) -> Compiled:
    return Compiled(tmp_path_factory)


def init_script(key: str, values: dict[str, Any], flux_map: Any = None) -> str:
    """Model initialization of the control system (without the system model)."""
    if key in ("fvc", "smvhz"):
        variables = []
        if flux_map is not None:
            variables = [(f"est_flux_map.{f}", flux_map[f]) for f in _sm.GRADNET_FIELDS]
        module = _sm_control if key == "fvc" else _sm_vhz_control
        return module.init_script(variables, values, "")
    if key == "cvc":
        return _im_control.init_script([], values, "")
    if key == "vhz":
        return _im_vhz_control.init_script([], values, "")
    return _grid_control.init_script([], values, "", gfl=key == "gfl")


def check(
    compiled: Compiled,
    key: str,
    values: dict[str, Any],
    run: Callable[[Netlist], list[list[float]]],
    dc_bus_inputs: list[float] | None,
    flux_map: Any = None,
) -> None:
    """
    Start the control system with the parameters of its masks, and run it by `run`
    (e.g., in a closed-loop simulation compared with motulator), which returns the
    inputs at each step. The monolithic control system should give the same outputs
    exactly with the same inputs, from the initial state. Finally, check the lower
    limit of the measured DC-bus voltage with the constant inputs `dc_bus_inputs`.
    The parameters are the same with the scoping of the masks in PLECS and in
    Simulink.
    """
    net, ref = compiled(key)
    block, mono = SYSTEMS[key]
    init = init_script(key, values, flux_map)
    params = block_params(net.top, init, block)
    np.testing.assert_equal(block_params(net.top, init, block, plecs=True), params)

    def restart() -> None:
        assert net.start(params, start) is None
        assert start(ref, sfunction_params(mono, values, flux_map)) is None

    restart()
    inputs = run(net)
    restart()
    n = 3 + len(block.signals)
    y1, y2 = (ctypes.c_double * n)(), (ctypes.c_double * n)()
    for u in inputs:
        net.sfun_step(arr(u), y1)
        ref.sfun_step(arr(u), y2)
        assert list(y1) == list(y2)
    if dc_bus_inputs is not None:

        def restart_net() -> None:
            assert net.start(params, start) is None

        check_dc_bus_limit(net, block, dc_bus_inputs, restart_net)
    net.sfun_terminate()
    ref.sfun_terminate()


# %%
@pytest.mark.parametrize(
    ("sensorless", "t_d", "blocked"),
    [(True, 0.0, False), (False, 0.0, False), (True, 2e-6, False), (True, 0.0, True)],
    ids=["sensorless", "sensored", "dead_time", "blocked"],
)
def test_flux_vector_control(
    compiled: Compiled, sensorless: bool, t_d: float, blocked: bool
) -> None:
    """Flux-vector control should agree with motulator, and exactly with the
    monolithic control system, also with the gates blocked."""
    build, signals = sm_system(sensorless, t_d, blocked)
    values, _ = _sm.export_values(build()[1])

    def run(net: Netlist) -> list[list[float]]:
        sfun = drive_sfun(net, _sm_control.FVC_BLOCK)
        return compare(build, model.Simulation, sfun, signals, 0.3)

    check(compiled, "fvc", values, run, SM_INPUTS)


def test_interrupted_flying_start(compiled: Compiled) -> None:
    """A new blocking of the gates right after the release should postpone the catch
    of the flying start in the netlist as in the monolithic control system."""
    build, _ = sm_system(True, 0.0, True)
    values, _ = _sm.export_values(build()[1])
    enabled = [0, 1, 0, 0, 1, 1, 1, 1]
    i_s_ab = [0j, 0j, -1j, 0j, 0j, -1j, -2j, -2j]

    def run(net: Netlist) -> list[list[float]]:
        return [
            [e, 100.0, *complex2abc(i), 540.0, 0.0]
            for e, i in zip(enabled, i_s_ab, strict=True)
        ]

    check(compiled, "fvc", values, run, None)


def test_gradnet_flux_vector_control(compiled: Compiled) -> None:
    """Flux-vector control with a GradNet flux map should agree exactly with the
    monolithic control system (which is compared with motulator in
    `test_simulink`)."""
    ctrl = gradnet_control_system()
    values, flux_map = _sm.export_values(ctrl)

    def run(net: Netlist) -> list[list[float]]:
        return [u for _, u in gradnet_inputs(ctrl, 1600)]

    check(compiled, "fvc", values, run, None, flux_map)


@pytest.mark.parametrize(
    ("sensorless", "t_d", "i_0", "u_th", "single_update", "blocked"),
    [
        (True, 0.0, 0.0, 0.0, False, False),
        (False, 0.0, 0.0, 0.0, False, False),
        (True, 2e-6, 0.5, 0.0, False, False),
        (True, 2e-6, 0.5, 1.0, True, False),
        (True, 0.0, 0.5, 1.0, False, True),
    ],
    ids=["sensorless", "sensored", "dead_time_tanh", "u_th_single", "u_th_blocked"],
)
def test_current_vector_control(
    compiled: Compiled,
    sensorless: bool,
    t_d: float,
    i_0: float,
    u_th: float,
    single_update: bool,
    blocked: bool,
) -> None:
    """Current-vector control should agree with motulator, and exactly with the
    monolithic control system, also with the threshold voltage, single update, and
    the gates blocked."""
    build, signals = im_system(sensorless, t_d, i_0, u_th, single_update, blocked)
    values = _im.export_values(build()[1])

    def run(net: Netlist) -> list[list[float]]:
        sfun = drive_sfun(net, _im_control.CVC_BLOCK)
        return compare(build, model.Simulation, sfun, signals, 0.3)

    check(compiled, "cvc", values, run, IM_INPUTS)


@pytest.mark.parametrize(
    ("open_loop", "t_d", "d_min", "blocked"), VHZ_CASES, ids=VHZ_IDS
)
def test_vhz_control(
    compiled: Compiled, open_loop: bool, t_d: float, d_min: float, blocked: bool
) -> None:
    """Observer-based or open-loop V/Hz control should agree with motulator, and
    exactly with the monolithic control system."""
    build, signals = vhz_system(open_loop, t_d, d_min, blocked)
    values = _im.export_vhz_values(build()[1])

    def run(net: Netlist) -> list[list[float]]:
        sfun = drive_sfun(net, _im_vhz_control.VHZ_BLOCK)
        return compare(build, model.Simulation, sfun, signals, 1.0, VHZ_RTOL)

    check(compiled, "vhz", values, run, VHZ_INPUTS)


@pytest.mark.parametrize(("t_d", "d_min", "blocked"), SM_VHZ_CASES, ids=SM_VHZ_IDS)
def test_sm_vhz_control(
    compiled: Compiled, t_d: float, d_min: float, blocked: bool
) -> None:
    """Observer-based V/Hz control of a synchronous machine drive."""
    build, signals = sm_vhz_system(t_d, d_min, blocked)
    values, _ = _sm.export_vhz_values(build()[1])

    def run(net: Netlist) -> list[list[float]]:
        sfun = drive_sfun(net, _sm_vhz_control.VHZ_BLOCK)
        return compare(build, model.Simulation, sfun, signals, 1.0, VHZ_RTOL)

    check(compiled, "smvhz", values, run, VHZ_INPUTS)


@pytest.mark.parametrize("blocked", [False, True], ids=["enabled", "blocked"])
def test_grid_following_control(compiled: Compiled, blocked: bool) -> None:
    """Grid-following control should agree with motulator, and exactly with the
    monolithic control system, also with the gates blocked."""
    build, signals = gfl_system(blocked)
    values = _grid.export_values(build()[1])

    def run(net: Netlist) -> list[list[float]]:
        sfun = grid_sfun(net, _grid_control.GFL_BLOCK)
        return compare(build, grid_model.Simulation, sfun, signals, 0.06)

    check(compiled, "gfl", values, run, GFL_INPUTS)


@pytest.mark.parametrize(
    ("power_limitation", "blocked"),
    [(False, False), (True, False), (True, True)],
    ids=["no_limitation", "power_limitation", "power_limitation_blocked"],
)
def test_grid_forming_control(
    compiled: Compiled, power_limitation: bool, blocked: bool
) -> None:
    """Grid-forming control should agree with motulator, and exactly with the
    monolithic control system, also with the gates blocked."""
    build, signals = gfm_system(power_limitation, blocked)
    values = _grid.export_values(build()[1])

    def run(net: Netlist) -> list[list[float]]:
        sfun = grid_sfun(net, _grid_control.GFM_BLOCK)
        return compare(build, grid_model.Simulation, sfun, signals, 0.25)

    check(compiled, "gfm", values, run, GFM_INPUTS)


def test_parameter_errors(compiled: Compiled) -> None:
    """Missing or invalid parameters should be reported by the blocks."""
    net, _ = compiled("fvc")
    build, _ = sm_system(True, 0.0)
    values, _ = _sm.export_values(build()[1])
    for name, value, message in [
        ("speed_J", None, "SpeedController: J must be a scalar."),
        ("pwm_d_min", None, "PWM/compute_output: d_min must be a scalar."),
        ("k_o", [1.0, 2.0, 3.0], "k_o must be [k0 k1]."),
        ("L_d", None, "L_d, L_q, and psi_f needed without a flux map."),
    ]:
        init = init_script("fvc", values | {name: value})
        err = net.start(block_params(net.top, init, _sm_control.FVC_BLOCK), start)
        assert err is not None and err.endswith(message), err
    net.sfun_terminate()


@pytest.mark.parametrize(
    ("changed", "message"),
    [
        ({"psi_f": 0.0}, "psi_s_min must be positive for machines without PMs."),
        (
            {"L_d": SM_PAR["L_q"], "L_q": SM_PAR["L_d"]},
            "No MTPA solution found. Check the alignment of the d axis",
        ),
    ],
    ids=["psi_s_min", "mtpa"],
)
def test_reference_generator_errors(
    compiled: Compiled, changed: dict[str, float], message: str
) -> None:
    """
    The block ReferenceGenerator should report the errors that ReferenceGenerator of
    motulator raises: a machine without PMs needs a positive psi_s_min (the default
    psi_f being zero), and the MTPA locus needs a solution (here, the inductances of
    a PM machine swapped).
    """
    net, _ = compiled("fvc")
    build, _ = sm_system(True, 0.0)
    values, _ = _sm.export_values(build()[1])
    pars: dict[str, Any] = {**SM_PAR, **changed}
    par = model.SynchronousMachinePars(**pars)
    cfg = sm_control.FluxVectorControllerCfg(i_s_max=values["i_s_max"])
    with pytest.raises(ValueError, match=re.escape(message)):
        sm_control.FluxVectorController(par, cfg)
    init = init_script("fvc", values | changed)
    err = net.start(block_params(net.top, init, _sm_control.FVC_BLOCK), start)
    assert err is not None and message in err, err
    net.sfun_terminate()


@pytest.mark.parametrize("J", [None, 0.015], ids=["without_J", "with_J"])
@pytest.mark.parametrize("sensorless", [True, False], ids=["sensorless", "sensored"])
def test_observer_gains(sensorless: bool, J: float | None) -> None:
    """The masks should give the observer gains of motulator, with the default speed
    estimation poles matched to the sampling period."""
    # Flux-vector control of a synchronous machine
    sm_par = model.SynchronousMachinePars(**SM_PAR)
    cfg = sm_control.FluxVectorControllerCfg(i_s_max=6.5, sensorless=sensorless, J=J)
    fvc = sm_control.FluxVectorController(sm_par, cfg)
    ctrl = VectorControlSystem(fvc, sm_control.SpeedController(**SM_SPEED))
    values, _ = _sm.export_values(ctrl)
    params = block_params(
        _sm_control.FVC_BLOCK.netlist(),
        init_script("fvc", values),
        _sm_control.FVC_BLOCK,
    )
    obs = fvc.observer
    path = "FluxVectorController/SpeedFluxObserver/"
    assert params[path + "FluxObserver"][0] == pytest.approx(
        obs.flux_observer.k_theta, rel=1e-14
    )
    k = params[path + "SpeedObserver"][:2]
    assert k == pytest.approx(
        [obs.speed_observer.k_w, obs.speed_observer.k_tau], rel=1e-14
    )

    # Current-vector control of an induction machine
    im_par = model.InductionMachineInvGammaPars(**IM_PAR)
    cfg = im_control.CurrentVectorControllerCfg(
        psi_s_nom=1.04, i_s_max=10.6, sensorless=sensorless, J=J
    )
    cvc = im_control.CurrentVectorController(im_par, cfg)
    ctrl = VectorControlSystem(cvc, im_control.SpeedController(**IM_SPEED))
    values = _im.export_values(ctrl)
    params = block_params(
        _im_control.CVC_BLOCK.netlist(),
        init_script("cvc", values),
        _im_control.CVC_BLOCK,
    )
    obs = cvc.observer
    k = params["CurrentVectorController/SpeedFluxObserver/SpeedObserver"][:2]
    assert k == pytest.approx(
        [obs.speed_observer.k_w, obs.speed_observer.k_tau], rel=1e-14
    )


def test_init_script() -> None:
    """The model initialization should define the parameters as in motulator."""
    build, _ = gfm_system(True)
    values = _grid.export_values(build()[1])
    init = init_script("gfm", values)
    assert "cfg.R_a = " in init
    assert "cfg.i_d_max = " in init
    assert np.isfinite(values["i_d_max"])


@pytest.mark.parametrize("k_v", [None, 0.7], ids=["default_k_v", "k_v"])
def test_gfm_configuration(k_v: float | None) -> None:
    """
    The parameters of grid-forming control should be read from its configuration,
    also after a simulation has changed the state of its observer, and the mask of
    the observer should give the observer gain of motulator.
    """
    cfg = grid_control.ObserverBasedGridFormingControllerCfg(
        i_max=1.3 * BASE.i,
        L=0.35 * BASE.L,
        R=0.05 * BASE.Z,
        alpha_o=2 * np.pi * 37.5,
        k_v=k_v,
        u_nom=BASE.u,
        w_nom=BASE.w,
    )
    gfm = grid_control.ObserverBasedGridFormingController(cfg)
    ctrl = grid_control.GridConverterControlSystem(gfm)
    ctrl.set_power_ref(0.5 * BASE.p)
    ctrl.set_ac_voltage_ref(BASE.u)
    mdl = grid_model.GridConverterSystem(
        grid_model.VoltageSourceConverter(u_dc=650),
        grid_model.LFilter(L_f=0.15 * BASE.L, L_g=0.74 * BASE.L),
        grid_model.ThreePhaseSource(w_g=BASE.w, e_g=BASE.u),
    )
    grid_model.Simulation(mdl, ctrl, show_progress=False).simulate(t_stop=0.01)
    assert gfm.observer.state.u_g != BASE.u  # The initial estimate has changed
    values = _grid.export_values(ctrl)
    assert (values["u_nom"], values["alpha_o"], values["k_v"]) == (
        BASE.u,
        cfg.alpha_o,
        k_v,
    )
    params = block_params(
        _grid_control.GFM_BLOCK.netlist(),
        init_script("gfm", values),
        _grid_control.GFM_BLOCK,
    )
    path = "ObserverBasedGridFormingController/Observer"
    u_nom, w_nom, L, alpha, T_s, R = params[path]
    assert Observer(u_nom, w_nom, L, alpha, T_s, R).k_o == gfm.observer.k_o
