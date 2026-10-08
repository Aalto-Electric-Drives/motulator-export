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
from collections.abc import Callable, Iterator
from typing import Any

import numpy as np
import pytest
from motulator.drive import model
from motulator.grid import model as grid_model

from motulator_export.plecs import (
    _grid_control,
    _im_control,
    _im_vhz_control,
    _sm_control,
    grid,
    im,
    sm,
)
from motulator_export.plecs._common import ControlBlock
from motulator_export.plecs._control import VALUES
from motulator_export.plecs._netlist import PLECS, SIMULINK, Subsystem, layout, problems
from tests import monolithic
from tests.c_port import arr
from tests.matlab import block_params
from tests.monolithic import MonolithicBlock, sfunction_params
from tests.netlist import Netlist
from tests.test_simulink import (
    GFL_INPUTS,
    GFM_INPUTS,
    IM_INPUTS,
    IM_SPEED,
    SM_INPUTS,
    SM_SPEED,
    VHZ_CASES,
    VHZ_IDS,
    VHZ_INPUTS,
    check_enable,
    compare,
    compile_sfunction,
    drive_sfun,
    gfl_system,
    gfm_system,
    gradnet_control_system,
    gradnet_inputs,
    grid_sfun,
    im_system,
    sm_system,
    start,
    vhz_system,
)

# Control systems: the block and its monolithic reference
SYSTEMS: dict[str, tuple[ControlBlock, MonolithicBlock]] = {
    "fvc": (_sm_control.FVC_BLOCK, monolithic.FVC),
    "cvc": (_im_control.CVC_BLOCK, monolithic.CVC),
    "vhz": (_im_vhz_control.VHZ_BLOCK, monolithic.VHZ),
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
    if key == "fvc":
        variables = []
        if flux_map is not None:
            variables = [(f"est_flux_map.{f}", flux_map[f]) for f in sm.GRADNET_FIELDS]
        return _sm_control.init_script(variables, values, "")
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
    enable_inputs: list[float] | None,
    flux_map: Any = None,
) -> None:
    """
    Start the control system with the parameters of its masks, and run it by `run`
    (e.g., in a closed-loop simulation compared with motulator), which returns the
    inputs at each step. The monolithic control system should give the same outputs
    exactly with the same inputs, from the initial state. Finally, check the input
    `enable` with the constant inputs `enable_inputs`. The parameters are the same
    with the scoping of the masks in PLECS and in Simulink.
    """
    net, ref = compiled(key)
    block, mono = SYSTEMS[key]
    init = init_script(key, values, flux_map)
    params = block_params(net.top, init, VALUES)
    np.testing.assert_equal(block_params(net.top, init, VALUES, plecs=True), params)

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
    if enable_inputs is not None:
        check_enable(net, block, enable_inputs)
    net.sfun_terminate()
    ref.sfun_terminate()


# %%
@pytest.mark.parametrize(
    ("sensorless", "t_d"),
    [(True, 0.0), (False, 0.0), (True, 2e-6)],
    ids=["sensorless", "sensored", "dead_time"],
)
def test_flux_vector_control(compiled: Compiled, sensorless: bool, t_d: float) -> None:
    """Flux-vector control should agree with motulator, and exactly with the
    monolithic control system, also when disabled and enabled."""
    build, signals = sm_system(sensorless, t_d)
    values, _ = sm.export_values(build()[1], SM_SPEED)

    def run(net: Netlist) -> list[list[float]]:
        sfun = drive_sfun(net, _sm_control.FVC_BLOCK)
        return compare(build, model.Simulation, sfun, signals, 0.3)

    check(compiled, "fvc", values, run, SM_INPUTS)


def test_gradnet_flux_vector_control(compiled: Compiled) -> None:
    """Flux-vector control with a GradNet flux map should agree exactly with the
    monolithic control system (which is compared with motulator in
    `test_simulink`)."""
    ctrl, speed = gradnet_control_system()
    values, flux_map = sm.export_values(ctrl, speed)

    def run(net: Netlist) -> list[list[float]]:
        return [u for _, u in gradnet_inputs(ctrl, 1600)]

    check(compiled, "fvc", values, run, None, flux_map)


@pytest.mark.parametrize(
    ("sensorless", "t_d", "i_0"),
    [(True, 0.0, 0.0), (False, 0.0, 0.0), (True, 2e-6, 0.5)],
    ids=["sensorless", "sensored", "dead_time_tanh"],
)
def test_current_vector_control(
    compiled: Compiled, sensorless: bool, t_d: float, i_0: float
) -> None:
    """Current-vector control should agree with motulator, and exactly with the
    monolithic control system."""
    build, signals = im_system(sensorless, t_d, i_0)
    values = im.export_values(build()[1], IM_SPEED)

    def run(net: Netlist) -> list[list[float]]:
        sfun = drive_sfun(net, _im_control.CVC_BLOCK)
        return compare(build, model.Simulation, sfun, signals, 0.3)

    check(compiled, "cvc", values, run, IM_INPUTS)


@pytest.mark.parametrize(("open_loop", "t_d", "d_min"), VHZ_CASES, ids=VHZ_IDS)
def test_vhz_control(
    compiled: Compiled, open_loop: bool, t_d: float, d_min: float
) -> None:
    """Observer-based or open-loop V/Hz control should agree with motulator, and
    exactly with the monolithic control system."""
    build, signals = vhz_system(open_loop, t_d, d_min)
    values = im.export_vhz_values(build()[1])

    def run(net: Netlist) -> list[list[float]]:
        sfun = drive_sfun(net, _im_vhz_control.VHZ_BLOCK)
        return compare(build, model.Simulation, sfun, signals, 1.0)

    check(compiled, "vhz", values, run, VHZ_INPUTS)


def test_grid_following_control(compiled: Compiled) -> None:
    """Grid-following control should agree with motulator, and exactly with the
    monolithic control system."""
    build, signals = gfl_system()
    values = grid.export_values(build()[1])

    def run(net: Netlist) -> list[list[float]]:
        sfun = grid_sfun(net, _grid_control.GFL_BLOCK)
        return compare(build, grid_model.Simulation, sfun, signals, 0.06)

    check(compiled, "gfl", values, run, GFL_INPUTS)


@pytest.mark.parametrize("power_limitation", [False, True])
def test_grid_forming_control(compiled: Compiled, power_limitation: bool) -> None:
    """Grid-forming control should agree with motulator, and exactly with the
    monolithic control system."""
    build, signals = gfm_system(power_limitation)
    values = grid.export_values(build()[1])

    def run(net: Netlist) -> list[list[float]]:
        sfun = grid_sfun(net, _grid_control.GFM_BLOCK)
        return compare(build, grid_model.Simulation, sfun, signals, 0.25)

    check(compiled, "gfm", values, run, GFM_INPUTS)


def test_parameter_errors(compiled: Compiled) -> None:
    """Missing or invalid parameters should be reported by the blocks."""
    net, _ = compiled("fvc")
    build, _ = sm_system(True, 0.0)
    values, _ = sm.export_values(build()[1], SM_SPEED)
    for name, value, message in [
        ("speed_J", None, "SpeedController: J must be a scalar."),
        ("pwm_d_min", None, "PWM/compute_output: d_min must be a scalar."),
        ("k_o", [1.0, 2.0, 3.0], "k_o must be [k0 k1]."),
        ("L_d", None, "L_d, L_q, and psi_f needed without a flux map."),
    ]:
        init = init_script("fvc", values | {name: value})
        err = net.start(block_params(net.top, init, VALUES), start)
        assert err is not None and err.endswith(message), err
    net.sfun_terminate()


def test_init_script() -> None:
    """The model initialization should define the parameters as in motulator."""
    build, _ = gfm_system(True)
    values = grid.export_values(build()[1])
    init = init_script("gfm", values)
    assert "cfg.R_a = " in init
    assert "cfg.i_d_max = " in init
    assert np.isfinite(values["i_d_max"])
