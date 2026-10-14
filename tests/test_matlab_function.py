"""
Test the control systems with MATLAB Function blocks in Simulink.

The blocks of the control systems run the MATLAB port of motulator in MATLAB Function
blocks (`code` "matlab" of the Simulink export). Each control system is built alone
in a Simulink model (`build_harness.m`) and run with the inputs of the closed-loop
simulations of `test_control.py`, followed by a sequence with the input `enable`
toggled and the DC-bus voltage zero (as in `test_simulink.check_enable`). The
outputs, i.e., the duty ratios and the monitored signals, should agree with those of
the S-functions of the blocks (`tests/netlist.py`), which agree with motulator, to
the rounding errors (amplified by the replay in the vector control of the drives,
see `test_matlab_function`).

Needs MATLAB with the MATLAB Engine API for Python (skipped otherwise) and gcc. Run
from the repository root:

    pytest tests/test_matlab_function.py

"""

# %%
import ctypes
from collections.abc import Callable, Iterator
from pathlib import Path
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
from motulator_export.plecs._netlist import SIMULINK, layout, leaves, problems
from motulator_export.simulink._common import write_script
from motulator_export.simulink._mfunction import block_mfunction
from motulator_export.simulink._render import wrapper
from tests.c_port import arr
from tests.matlab import block_params
from tests.netlist import Netlist
from tests.test_control import SYSTEMS, Compiled, init_script
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
    compare,
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

TESTS = Path(__file__).parent
# Tolerance of the outputs relative to their magnitudes (at least 1)
RTOL = 1e-9

Run = Callable[[Netlist], list[list[float]]]


# %%
# Tests without MATLAB
def test_wrapper_layouts() -> None:
    """The subsystems of the blocks with delayed inputs should have clean layouts."""
    for block, _ in SYSTEMS.values():
        for _, b in leaves(block.netlist()):
            f = block_mfunction(b)
            if f.delayed:
                assert problems(layout(wrapper(b, f), SIMULINK)) == []


def test_functions() -> None:
    """
    The MATLAB functions of the blocks should have the code of all the blocks, unique
    fields of their parameters, and the sample times among the parameters.
    """
    names: dict[str, str] = {}
    for block, _ in SYSTEMS.values():
        for _, b in leaves(block.netlist()):
            f = block_mfunction(b)
            assert names.setdefault(f.name, f.source) == f.source
            fields = [field for field, _ in f.params]
            assert len(set(fields)) == len(fields)
            assert f.sample_time is None or f.sample_time in fields
            assert f.source.startswith("function ")
            assert f"{f.name}(" in f.source.splitlines()[0]


# %%
# Cases: the key of the control system, its parameter values and flux map, the
# closed-loop simulation giving the inputs, and the constant inputs of the sequence
# with the input enable
Case = tuple[str, dict[str, Any], Any, Run, list[float]]


def drive_run(build: Any, signals: Any, block: ControlBlock, t_stop: float) -> Run:
    """Closed-loop simulation of a drive."""

    def run(net: Netlist) -> list[list[float]]:
        sfun = drive_sfun(net, block)
        return compare(build, model.Simulation, sfun, signals, t_stop)

    return run


def grid_run(build: Any, signals: Any, block: ControlBlock, t_stop: float) -> Run:
    """Closed-loop simulation of a grid converter."""

    def run(net: Netlist) -> list[list[float]]:
        sfun = grid_sfun(net, block)
        return compare(build, grid_model.Simulation, sfun, signals, t_stop)

    return run


def case(name: str) -> Case:
    """Case of a test, see `Case`."""
    if name.startswith("fvc_gradnet"):
        ctrl, speed = gradnet_control_system()
        values, flux_map = sm.export_values(ctrl, speed)

        def gradnet_run(_: Netlist) -> list[list[float]]:
            return [u for _, u in gradnet_inputs(ctrl, 1600)]

        return "fvc", values, flux_map, gradnet_run, SM_INPUTS
    if name.startswith("fvc"):
        sensorless, t_d = {
            "fvc_sensorless": (True, 0.0),
            "fvc_sensored": (False, 0.0),
            "fvc_dead_time": (True, 2e-6),
        }[name]
        build, signals = sm_system(sensorless, t_d)
        values, _ = sm.export_values(build()[1], SM_SPEED)
        run = drive_run(build, signals, _sm_control.FVC_BLOCK, 0.3)
        return "fvc", values, None, run, SM_INPUTS
    if name.startswith("cvc"):
        sensorless, t_d, i_0 = {
            "cvc_sensorless": (True, 0.0, 0.0),
            "cvc_sensored": (False, 0.0, 0.0),
            "cvc_dead_time_tanh": (True, 2e-6, 0.5),
        }[name]
        build, signals = im_system(sensorless, t_d, i_0)
        values = im.export_values(build()[1], IM_SPEED)
        run = drive_run(build, signals, _im_control.CVC_BLOCK, 0.3)
        return "cvc", values, None, run, IM_INPUTS
    if name.startswith("vhz"):
        open_loop, t_d, d_min = VHZ_CASES[VHZ_IDS.index(name.removeprefix("vhz_"))]
        build, signals = vhz_system(open_loop, t_d, d_min)
        values = im.export_vhz_values(build()[1])
        run = drive_run(build, signals, _im_vhz_control.VHZ_BLOCK, 1.0)
        return "vhz", values, None, run, VHZ_INPUTS
    if name == "gfl":
        build, signals = gfl_system()
        values = grid.export_values(build()[1])
        run = grid_run(build, signals, _grid_control.GFL_BLOCK, 0.06)
        return "gfl", values, None, run, GFL_INPUTS
    build, signals = gfm_system(name == "gfm_power_limitation")
    values = grid.export_values(build()[1])
    run = grid_run(build, signals, _grid_control.GFM_BLOCK, 0.25)
    return "gfm", values, None, run, GFM_INPUTS


CASES = [
    "fvc_sensorless",
    "fvc_sensored",
    "fvc_dead_time",
    "fvc_gradnet",
    "cvc_sensorless",
    "cvc_sensored",
    "cvc_dead_time_tanh",
    *(f"vhz_{v}" for v in VHZ_IDS),
    "gfl",
    "gfm",
    "gfm_power_limitation",
]


def enable_sequence(block: ControlBlock, inputs: list[float]) -> list[list[float]]:
    """
    Inputs disabling and enabling the control system with the constant inputs, also
    with the DC-bus voltage zero (as in `test_simulink.check_enable`).
    """
    k_dc = sum(block.input_widths[1 : block.inputs.index("u_dc")])
    out: list[list[float]] = []
    for enable, n, u_dc in [(0, 2, None), (1, 400, None), (0, 2, None), (1, 200, 0)]:
        u = list(inputs)
        if u_dc is not None:
            u[k_dc] = u_dc
        out += [[float(enable), *u]] * n
    return out


# %%
# Tests with MATLAB
class Harnesses:
    """The harness models of the control systems in MATLAB, built when needed."""

    def __init__(self, eng: Any, tmp: pytest.TempPathFactory) -> None:
        self.eng = eng
        self.tmp = tmp
        self.models: dict[str, Path] = {}

    def simulate(
        self, key: str, init: str, inputs: np.ndarray, T_s: float
    ) -> np.ndarray:
        """Simulate the control system with the initialization and the inputs."""
        import matlab  # noqa: PLC0415  # pyright: ignore[reportMissingImports]

        name = f"harness_{key}"
        if key not in self.models:
            folder = self.tmp.mktemp(key)
            path = folder / f"{name}.slx"
            (folder / f"init_{name}.m").write_text(init, encoding="utf-8")
            block, _ = SYSTEMS[key]
            fields = [
                ("init_script", f"init_{name}"),
                ("input_widths", block.input_widths),
            ]
            script = write_script(
                path, block, VALUES, fields, "build_harness", VALUES["T_s"], 1.0,
                code="matlab",
            )  # fmt: skip
            self.eng.run(str(script), nargout=0)
            self.models[key] = path
        path = self.models[key]
        path.with_name(f"init_{name}.m").write_text(init, encoding="utf-8")
        self.eng.eval(f"clear('init_{name}');", nargout=0)
        # The input of the sampling instant k at (k - 0.5)*T_s (k > 0), so that the
        # inputs are sampled at the instants regardless of the rounding of the time,
        # and the last input repeated after the stop time (else the input is zero
        # after its last time)
        n = len(inputs)
        t = np.maximum(np.arange(n + 1) - 0.5, 0) * T_s
        u = np.vstack((inputs, inputs[-1]))
        # Without the Engine API, pyright takes the module tests/matlab.py
        table = np.column_stack((t, u)).tolist()
        self.eng.workspace["u"] = matlab.double(table)  # pyright: ignore[reportAttributeAccessIssue]
        stop = (len(inputs) - 1) * T_s
        self.eng.eval(
            f"out = sim('{name}', 'LoadExternalInput', 'on', 'ExternalInput', 'u', "
            f"'StopTime', '{stop!r}');",
            nargout=0,
        )
        return np.asarray(self.eng.eval("out.yout"))


@pytest.fixture(scope="module")
def harnesses(tmp_path_factory: pytest.TempPathFactory) -> Iterator[Harnesses]:
    engine = pytest.importorskip("matlab.engine")
    eng = engine.start_matlab()
    eng.addpath(str(TESTS), nargout=0)
    try:
        yield Harnesses(eng, tmp_path_factory)
    finally:
        eng.quit()


@pytest.fixture(scope="module")
def compiled(tmp_path_factory: pytest.TempPathFactory) -> Compiled:
    return Compiled(tmp_path_factory)


def replay(
    net: Netlist, params: dict[str, list[Any]], inputs: np.ndarray, n_out: int
) -> np.ndarray:
    """Outputs of the netlist of S-functions with the inputs, from its start."""
    assert net.start(params, start) is None
    y = (ctypes.c_double * n_out)()
    out = np.zeros((len(inputs), n_out))
    for k, u in enumerate(inputs):
        net.sfun_step(arr(u), y)
        out[k] = y
    return out


@pytest.mark.parametrize("name", CASES)
def test_matlab_function(harnesses: Harnesses, compiled: Compiled, name: str) -> None:
    """
    The control system with MATLAB Function blocks should give the outputs of its
    S-functions with the same inputs.

    The inputs of a closed-loop simulation replayed in open loop may amplify the
    rounding errors: in the vector control of the drives, the observers see the
    voltages of the control system itself, but the recorded currents do not respond
    to them, so the differences grow exponentially. The outputs are therefore
    compared within their sensitivity, which the S-functions with the inputs
    perturbed at the level of the rounding errors (relative 1e-14) show: from the
    start of the closed loop and of the sequence with `enable` (which resets the
    state), each output is compared until its sensitivity reaches 1e-6 relative to
    its magnitude (after which the replay is meaningless), with the tolerance of
    RTOL plus 100 times the sensitivity (the rounding errors of the MATLAB code grow
    as the perturbation does).
    """
    key, values, flux_map, run, constant = case(name)
    net, _ = compiled(key)
    block, _ = SYSTEMS[key]
    init = init_script(key, values, flux_map)
    params = block_params(net.top, init, VALUES)
    assert net.start(params, start) is None
    closed_loop = run(net)
    inputs = np.array(closed_loop + enable_sequence(block, constant))
    n_out = 3 + len(block.signals)
    expected = replay(net, params, inputs, n_out)
    signs = np.random.default_rng(0).choice([-1.0, 1.0], inputs.shape)
    perturbed = replay(net, params, inputs * (1 + 1e-14 * signs), n_out)
    net.sfun_terminate()
    actual = harnesses.simulate(key, init, inputs, values["T_s"])
    assert actual.shape == expected.shape
    scale = np.maximum(np.max(np.abs(expected), axis=0), 1.0)
    sensitivity = np.abs(perturbed - expected)
    n_loop = len(closed_loop)
    compared = np.zeros(sensitivity.shape, dtype=bool)
    for a, b in [(0, n_loop), (n_loop, len(inputs))]:
        conditioned = sensitivity[a:b] <= 1e-6 * scale
        compared[a:b] = np.cumprod(conditioned, axis=0).astype(bool)
        sensitivity[a:b] = np.maximum.accumulate(sensitivity[a:b], axis=0)
    err = np.where(compared, np.abs(actual - expected), 0.0)
    tol = RTOL * scale + 100 * sensitivity
    for j, signal in enumerate(["d_a", "d_b", "d_c", *block.signals]):
        print(
            f"  {signal}: max error {np.max(err[:, j]):.3g} (scale {scale[j]:.3g}), "
            f"max error/tolerance {np.max(err[:, j] / tol[:, j]):.2g}, steps "
            f"compared {np.sum(compared[:n_loop, j])} of {n_loop} and "
            f"{np.sum(compared[n_loop:, j])} of {len(inputs) - n_loop}"
        )
    # The duty ratios over the sequence with enable, at least (sensorless
    # current-vector control is sensitive already at the start, with zero flux)
    assert np.all(np.sum(compared[n_loop:, :3], axis=0) >= 100)
    assert np.all(err <= tol)
