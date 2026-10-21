"""
Test the control systems with MATLAB Function blocks in Simulink.

The blocks of the control systems run the MATLAB port of motulator in MATLAB Function
blocks (`code` "matlab" of the Simulink export). Each control system is built alone
in a Simulink model (`build_harness.m`) and run with the inputs of the closed-loop
simulations of `test_control.py` (also with the gates blocked), and with a sequence
of constant inputs with the gates blocked and enabled and the DC-bus voltage zero (as
in `test_simulink.check_dc_bus_limit`). The outputs, i.e., the duty ratios and the
monitored signals, should agree with those of the S-functions of the blocks
(`tests/netlist.py`), which agree with motulator, to the rounding errors (amplified
by the replay in the vector control of the drives, see `test_matlab_function`).

Needs MATLAB with the MATLAB Engine API for Python (skipped otherwise) and gcc. Run
from the repository root:

    pytest tests/test_matlab_function.py

"""

# %%
import ctypes
from collections.abc import Callable, Iterator
from pathlib import Path
from typing import Any, cast

import numpy as np
import pytest
from motulator.drive import model
from motulator.grid import model as grid_model

from motulator_export.plecs import (
    _grid,
    _grid_control,
    _im,
    _im_control,
    _im_vhz_control,
    _sm,
    _sm_control,
)
from motulator_export.plecs._common import MONITOR, ControlBlock
from motulator_export.plecs._control import VALUES, mask_values
from motulator_export.plecs._netlist import SIMULINK, layout, leaves, problems
from motulator_export.simulink._common import write_script
from motulator_export.simulink._mfunction import (
    block_mfunction,
    functions,
    gradnet_machine_mfunction,
    library,
    reference_luts_mfunction,
)
from motulator_export.simulink._render import netlist_rows, with_signals, wrapper
from tests.c_port import arr
from tests.matlab import block_params
from tests.netlist import Netlist
from tests.sfunctions import (
    GFL_INPUTS,
    GFM_INPUTS,
    IM_INPUTS,
    SM_INPUTS,
    VHZ_INPUTS,
    compare,
    drive_sfun,
    gradnet_inputs,
    grid_sfun,
    start,
)
from tests.systems import (
    VHZ_CASES,
    VHZ_IDS,
    gfl_system,
    gfm_system,
    gradnet_control_system,
    im_system,
    sm_system,
    vhz_system,
)
from tests.test_control import SYSTEMS, Compiled, init_script

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


def test_port_used() -> None:
    """
    Every function of the MATLAB port should be used by a block or by the machine
    model, since the others would not be tested.
    """
    sources = [gradnet_machine_mfunction(), reference_luts_mfunction()]
    for block, _ in SYSTEMS.values():
        sources += [block_mfunction(b) for _, b in leaves(block.netlist())]
    used = {name for f in sources for name in functions(f.source)}
    assert sorted(set(library()) - used) == []


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
        ctrl = gradnet_control_system()
        values, flux_map = _sm.export_values(ctrl)

        def gradnet_run(_: Netlist) -> list[list[float]]:
            return [u for _, u in gradnet_inputs(ctrl, 1600)]

        return "fvc", values, flux_map, gradnet_run, SM_INPUTS
    if name.startswith("fvc"):
        sensorless, t_d, blocked = {
            "fvc_sensorless": (True, 0.0, False),
            "fvc_sensored": (False, 0.0, False),
            "fvc_dead_time": (True, 2e-6, False),
            "fvc_blocked": (True, 0.0, True),
        }[name]
        build, signals = sm_system(sensorless, t_d, blocked)
        values, _ = _sm.export_values(build()[1])
        run = drive_run(build, signals, _sm_control.FVC_BLOCK, 0.3)
        return "fvc", values, None, run, SM_INPUTS
    if name.startswith("cvc"):
        # With the dead time, the duty-ratio error of a clamped leg (d = 1) differs
        # from its limit as d approaches 1 by t_d/(4*T_half)*(1 + s), and similarly at
        # d = 0. With the threshold voltage pulling the compensated duty ratio inside
        # the limits, the outputs then depend on the rounding of the clamping in the
        # overmodulation, unless the current direction s is exactly +-1, as np.sign
        # gives (i_0 = 0)
        args = {
            "cvc_sensorless": (True, 0.0, 0.0, 0.0, False, False),
            "cvc_sensored": (False, 0.0, 0.0, 0.0, False, False),
            "cvc_dead_time_tanh": (True, 2e-6, 0.5, 0.0, False, False),
            "cvc_u_th_single": (True, 2e-6, 0.0, 1.0, True, False),
            "cvc_u_th_blocked": (True, 0.0, 0.5, 1.0, False, True),
        }[name]
        build, signals = im_system(*args)
        values = _im.export_values(build()[1])
        run = drive_run(build, signals, _im_control.CVC_BLOCK, 0.3)
        return "cvc", values, None, run, IM_INPUTS
    if name.startswith("vhz"):
        vhz_args = VHZ_CASES[VHZ_IDS.index(name.removeprefix("vhz_"))]
        build, signals = vhz_system(*vhz_args)
        values = _im.export_vhz_values(build()[1])
        run = drive_run(build, signals, _im_vhz_control.VHZ_BLOCK, 1.0)
        return "vhz", values, None, run, VHZ_INPUTS
    if name.startswith("gfl"):
        build, signals = gfl_system(name == "gfl_blocked")
        values = _grid.export_values(build()[1])
        run = grid_run(build, signals, _grid_control.GFL_BLOCK, 0.06)
        return "gfl", values, None, run, GFL_INPUTS
    build, signals = gfm_system(
        name.startswith("gfm_power_limitation"), name.endswith("blocked")
    )
    values = _grid.export_values(build()[1])
    run = grid_run(build, signals, _grid_control.GFM_BLOCK, 0.25)
    return "gfm", values, None, run, GFM_INPUTS


CASES = [
    "fvc_sensorless",
    "fvc_sensored",
    "fvc_dead_time",
    "fvc_blocked",
    "fvc_gradnet",
    "cvc_sensorless",
    "cvc_sensored",
    "cvc_dead_time_tanh",
    "cvc_u_th_single",
    "cvc_u_th_blocked",
    *(f"vhz_{v}" for v in VHZ_IDS),
    "gfl",
    "gfl_blocked",
    "gfm",
    "gfm_power_limitation",
    "gfm_power_limitation_blocked",
]


def constant_sequence(block: ControlBlock, inputs: list[float]) -> list[list[float]]:
    """
    The constant inputs with the gates blocked and enabled, also with the DC-bus
    voltage zero (as in `test_simulink.check_dc_bus_limit`).
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

    def build(self, key: str, init: str) -> Path:
        """The harness model of the control system, built with the initialization
        when needed."""
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
        return self.models[key]

    def lines(self, key: str) -> set[tuple[str, ...]]:
        """Connections of the lines in the control system of the built harness model
        (system, source, destination), see `model_lines.m`."""
        name = f"harness_{key}"
        self.eng.load_system(str(self.models[key]), nargout=0)
        rows = self.eng.model_lines(f"{name}/Control system")
        return {tuple(row.split("|")) for row in rows}

    def simulate(
        self, key: str, init: str, inputs: np.ndarray, T_s: float
    ) -> np.ndarray:
        """Simulate the control system with the initialization and the inputs."""
        import matlab  # noqa: PLC0415  # pyright: ignore[reportMissingImports]

        name = f"harness_{key}"
        path = self.build(key, init)
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


@pytest.mark.parametrize(
    "name", ["fvc_sensorless", "cvc_sensorless", f"vhz_{VHZ_IDS[0]}", "gfl", "gfm"]
)
def test_model_lines(harnesses: Harnesses, name: str) -> None:
    """
    The lines of the control system built in Simulink should connect the ports of the
    netlist. The lines are drawn through the points of the layout, so a block whose
    size differs from that of the layout (e.g., a MATLAB Function block resized by
    Simulink) could make a line start from another port.
    """
    key, values, flux_map, _, _ = case(name)
    harnesses.build(key, init_script(key, values, flux_map))
    block, _ = SYSTEMS[key]
    rows = netlist_rows(with_signals(block.netlist(), MONITOR), code="matlab")
    expected = {(path, src, dst) for path, src, dst, _ in rows["lines"]}
    assert len(expected) == len(rows["lines"])
    assert harnesses.lines(key) == expected


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
    start of the closed loop and of the sequence of constant inputs (each simulated
    from the initial state), the outputs are compared until the sensitivity of any
    of them reaches 1e-6 relative to its magnitude, after which the replay is
    meaningless (an output that does not respond to the perturbation, e.g., the
    torque reference at its limit, does not show that the states still agree). The
    tolerance is RTOL plus 100 times the sensitivity (the rounding errors of the
    MATLAB code grow as the perturbation does).
    """
    key, values, flux_map, run, constant = case(name)
    net, _ = compiled(key)
    block, _ = SYSTEMS[key]
    init = init_script(key, values, flux_map)
    params = block_params(net.top, init, mask_values(block))
    assert net.start(params, start) is None
    closed_loop = run(net)
    n_out = 3 + len(block.signals)
    rng = np.random.default_rng(0)
    counts = []
    for inputs in (np.array(closed_loop), np.array(constant_sequence(block, constant))):
        expected = replay(net, params, inputs, n_out)
        signs = rng.choice([-1.0, 1.0], inputs.shape)
        perturbed = replay(net, params, inputs * (1 + 1e-14 * signs), n_out)
        actual = harnesses.simulate(key, init, inputs, values["T_s"])
        assert actual.shape == expected.shape
        scale = np.maximum(np.max(np.abs(expected), axis=0), 1.0)
        sensitivity = np.abs(perturbed - expected)
        conditioned = np.all(sensitivity <= 1e-6 * scale, axis=1)
        compared = np.cumprod(conditioned).astype(bool)
        sensitivity = np.maximum.accumulate(sensitivity, axis=0)
        err = np.where(compared[:, None], np.abs(actual - expected), 0.0)
        tol = RTOL * scale + 100 * sensitivity
        print(f"  steps compared: {np.sum(compared)} of {len(inputs)}")
        for j, signal in enumerate(["d_a", "d_b", "d_c", *block.signals]):
            ratio = np.max(err[:, j] / tol[:, j])
            print(
                f"  {signal}: max error {np.max(err[:, j]):.3g} (scale "
                f"{scale[j]:.3g}), max error/tolerance {ratio:.2g}"
            )
        assert np.all(err <= tol)
        counts.append(np.sum(compared))
    net.sfun_terminate()
    # The sequence of constant inputs, at least (sensorless current-vector control is
    # sensitive already at the start of the closed loop, with zero flux)
    assert counts[1] >= 100


@pytest.mark.parametrize("gradnet", [False, True], ids=["constant", "gradnet"])
def test_reference_luts(harnesses: Harnesses, tmp_path: Path, gradnet: bool) -> None:
    """
    The lookup tables of sm_reference_luts.m (called by the mask initialization of
    the ReferenceGenerator in Simulink) should be those of the reference generator
    of motulator at their sample points, to 1e-12 relative to their magnitudes, and
    to RTOL with the GradNet flux map, whose MTPV locus differs by about 1e-11.
    """
    if gradnet:
        ctrl = gradnet_control_system()
    else:
        ctrl = sm_system(True, 0.0)[0]()[1]
    values, flux_map = _sm.export_values(ctrl)
    rg = cast(Any, ctrl.vector_ctrl).reference_gen
    eng = harnesses.eng
    eng.addpath(str(reference_luts_mfunction().write(tmp_path).parent), nargout=0)
    # The struct par of a previous initialization (e.g., with a flux map) is cleared
    eng.eval("clear par cfg", nargout=0)
    eng.eval(init_script("fvc", values, flux_map), nargout=0)
    luts = np.asarray(eng.eval("sm_reference_luts(par, cfg.i_s_max)"))
    assert luts.shape == (16, 6)
    tables = [(rg.psi_s_mtpa, "psi_s_mtpa"), (rg.tau_M_cl, "tau_M_cl")]
    tables.append((rg.tau_M_mtpv, "tau_M_mtpv"))
    for i, (f, name) in enumerate(tables):
        x, y = luts[:, 2 * i], luts[:, 2 * i + 1]
        err = np.max(np.abs(f(x) - y)) / np.max(np.abs(y))
        print(f"  {name}: max relative error {err:.3g}")
        assert err < (RTOL if gradnet else 1e-12)
