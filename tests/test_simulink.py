"""
Test the generated Simulink S-functions against motulator.

The S-functions are generated from the C-Script code of the PLECS models and
compiled with gcc against a mock of the Simulink API (`tests/simulink_mock`). Each
control system runs in a closed-loop simulation of motulator, and the results are
compared with those of the control system of motulator. MATLAB is not needed.

Run from the repository root:

    pytest tests/test_simulink.py

"""

# %%
import ctypes
import subprocess
from collections.abc import Callable, Sequence
from math import pi
from pathlib import Path
from types import SimpleNamespace
from typing import Any, cast

import motulator.drive.control.im as im_control
import motulator.drive.control.sm as sm_control
import motulator.grid.control as grid_control
import motulator.grid.model as grid_model
import numpy as np
import pytest
from motulator.common.control._base import ControlSystem, TimeSeries
from motulator.common.utils import complex2abc, dead_time_error
from motulator.common.utils._utils import wrap
from motulator.drive import model
from motulator.drive.control._base import Measurements, VectorControlSystem
from motulator.grid import utils as grid_utils

from motulator_export.plecs import grid, im, sm
from motulator_export.plecs._common import C_SOURCES, ControlBlock
from motulator_export.simulink._sfunction import (
    SFunction,
    control_sfunction,
    gradnet_machine_sfunction,
)
from tests.c_port import GCC, arr

MOCK = Path(__file__).parent / "simulink_mock"


def compile_sfunction(sfun: ControlBlock | SFunction, out_dir: Path) -> ctypes.CDLL:
    """Generate the S-function (of a block) and compile it against the mock."""
    if isinstance(sfun, ControlBlock):
        sfun = control_sfunction(sfun)
    src = sfun.write(out_dir)
    lib = out_dir / "libsfun.so"
    cmd = [*GCC, "-Wall", "-Werror", "-Wno-unused-function", "-DMATLAB_MEX_FILE"]
    cmd += [f"-I{MOCK}", f"-I{C_SOURCES}", str(src), "-lm", "-o", str(lib)]
    subprocess.run(cmd, check=True)
    dll = ctypes.CDLL(str(lib))
    dll.sfun_start.restype = ctypes.c_char_p
    dll.sfun_sample_time.restype = ctypes.c_double
    return dll


@pytest.fixture(scope="module")
def fvc(tmp_path_factory: pytest.TempPathFactory) -> ctypes.CDLL:
    return compile_sfunction(sm.FVC_BLOCK, tmp_path_factory.mktemp("fvc"))


@pytest.fixture(scope="module")
def cvc(tmp_path_factory: pytest.TempPathFactory) -> ctypes.CDLL:
    return compile_sfunction(im.CVC_BLOCK, tmp_path_factory.mktemp("cvc"))


@pytest.fixture(scope="module")
def gfl(tmp_path_factory: pytest.TempPathFactory) -> ctypes.CDLL:
    return compile_sfunction(grid.GFL_BLOCK, tmp_path_factory.mktemp("gfl"))


@pytest.fixture(scope="module")
def gfm(tmp_path_factory: pytest.TempPathFactory) -> ctypes.CDLL:
    return compile_sfunction(grid.GFM_BLOCK, tmp_path_factory.mktemp("gfm"))


def sfunction_params(
    block: ControlBlock, values: dict[str, Any], flux_map: dict[str, Any] | None = None
) -> list[Any]:
    """
    Values of the S-function parameters (the C-Script parameters) of a block.

    The values are those of the mask, and the GradNet flux map gives the parameters
    of the mask initialization (empty without a flux map).

    """
    params = block.cscript_params or [m.variable for m in block.mask_params]
    out = []
    for p in params:
        if p == "isempty(psi_s_dq_fcn)":
            out.append(float(flux_map is None))
        elif p.startswith("gn_"):
            out.append(None if flux_map is None else flux_map[p[3:]])
        else:
            out.append(values[p])
    return out


def start(dll: ctypes.CDLL, params: list[Any]) -> str | None:
    """
    Start the S-function with the parameters, returning the error message.

    The parameters are passed as column vectors, the matrices in column-major order
    as in MATLAB.

    """
    arrays = [
        np.zeros(0) if p is None else np.asarray(p, dtype=float).ravel(order="F")
        for p in params
    ]
    numel = (ctypes.c_int * len(arrays))(*[a.size for a in arrays])
    err = dll.sfun_start(len(arrays), numel, arr(np.concatenate(arrays)))
    return None if err is None else err.decode()


class SFunctionControlSystem(ControlSystem):
    """Control system running the S-function, for closed-loop simulations."""

    def __init__(
        self,
        dll: ctypes.CDLL,
        block: ControlBlock,
        T_s: float,
        inputs: Callable[[Any, float], list[float]],
    ) -> None:
        super().__init__()
        self.dll = dll
        self.T_s = T_s
        self.inputs = inputs  # Inputs of the S-function from the model and the time
        self.names = ["d_a", "d_b", "d_c", *block.signals]
        self.y = (ctypes.c_double * len(self.names))()

    # The control loop is overridden, so the steps of the protocol are not used
    def get_measurement(self, mdl: Any) -> Any:
        raise NotImplementedError

    def get_feedback(self, meas: Any) -> Any:
        raise NotImplementedError

    def compute_output(self, fbk: Any) -> Any:
        raise NotImplementedError

    def run_control_loop(self, mdl: Any) -> tuple[float, Sequence[float]]:
        self.dll.sfun_step(arr(self.inputs(mdl, self.t)), self.y)
        out = np.array(self.y)
        self.save(
            self.t, out=SimpleNamespace(**dict(zip(self.names, out, strict=True)))
        )
        self.t = (self.t + self.T_s) % 1e9  # As in motulator
        return self.T_s, list(out[:3])

    def post_process(self) -> TimeSeries:
        return ControlSystem.post_process(self)  # Only the saved S-function outputs


def compare(
    build: Callable[[], tuple[Any, Any]],
    simulation: Callable[[Any, Any], Any],
    sfun: Callable[[Any], SFunctionControlSystem],
    signals: dict[str, Callable[[Any, Any], Any]],
    t_stop: float,
) -> None:
    """
    Simulate with the control systems of motulator and of the S-function.

    The system is built twice by `build`, and `sfun` creates the control system of
    the S-function from the control system of motulator (for its references).

    """
    mdl, ctrl = build()
    res = simulation(mdl, ctrl).simulate(t_stop=t_stop)
    fbk, ref = res.ctrl.fbk, res.ctrl.ref
    py = {
        "d_a": ref.d_abc[:, 0],
        "d_b": ref.d_abc[:, 1],
        "d_c": ref.d_abc[:, 2],
        **{name: f(fbk, ref) for name, f in signals.items()},
    }
    mdl, ctrl = build()
    res_sl = simulation(mdl, sfun(ctrl)).simulate(t_stop=t_stop)
    assert np.allclose(res.ctrl.t, res_sl.ctrl.t)
    for name, value in py.items():
        err = np.max(np.abs(getattr(res_sl.ctrl.out, name) - value))
        scale = np.max(np.abs(value))
        print(f"  {name}: max error {err:.3g} (max value {scale:.3g})")
        assert err <= 1e-8 * max(scale, 1.0), name


def drive_sfun(
    dll: ctypes.CDLL, block: ControlBlock, sensor: str
) -> Callable[[VectorControlSystem], SFunctionControlSystem]:
    """S-function control system of a drive, with the measured angle or speed."""

    def sfun(ctrl: VectorControlSystem) -> SFunctionControlSystem:
        def inputs(mdl: model.Drive, t: float) -> list[float]:
            mech = mdl.mechanics
            x = wrap(mech.meas_position()) if sensor == "theta_M" else mech.meas_speed()
            i_s_abc = mdl.machine.meas_currents()
            u_dc = mdl.converter.meas_dc_voltage()
            return [cast(Any, ctrl.ext_ref.w_M)(t), *i_s_abc, u_dc, x]

        return SFunctionControlSystem(
            dll, block, cast(Any, ctrl.vector_ctrl).cfg.T_s, inputs
        )

    return sfun


# %%
SM_PAR = {"n_p": 3, "R_s": 3.6, "L_d": 0.036, "L_q": 0.051, "psi_f": 0.545}
SM_SPEED = {"J": 0.015, "alpha_s": 25.0}


def test_parameter_errors(fvc: ctypes.CDLL) -> None:
    """Missing or invalid parameters should be reported."""
    values: dict[str, Any] = dict.fromkeys(m.variable for m in sm.MASK_PARAMS)
    assert start(fvc, sfunction_params(sm.FVC_BLOCK, values)) is not None
    values |= SM_PAR | {"i_s_max": 6.5, "T_s": 125e-6}
    values |= {"speed_J": 0.015, "speed_alpha_s": 25.0, "k_o": [1.0, 2.0, 3.0]}
    assert start(fvc, sfunction_params(sm.FVC_BLOCK, values)) == (
        "k_o must be [] or [k0 k1]."
    )
    values["k_o"] = None
    assert start(fvc, sfunction_params(sm.FVC_BLOCK, values)) is None
    assert fvc.sfun_sample_time() == 125e-6
    fvc.sfun_terminate()


@pytest.mark.parametrize("sensorless", [True, False])
def test_flux_vector_control(fvc: ctypes.CDLL, sensorless: bool) -> None:
    """The S-function of flux-vector control should agree with motulator."""

    def build() -> tuple[model.Drive, VectorControlSystem]:
        par = model.SynchronousMachinePars(**SM_PAR)
        mdl = model.Drive(
            model.SynchronousMachine(par),
            model.MechanicalSystem(J=0.015),
            model.VoltageSourceConverter(u_dc=540),
        )
        mdl.mechanics.set_external_load_torque(lambda t: (t > 0.2) * 10.0)
        cfg = sm_control.FluxVectorControllerCfg(i_s_max=6.5, sensorless=sensorless)
        ctrl = VectorControlSystem(
            sm_control.FluxVectorController(par, cfg),
            sm_control.SpeedController(**SM_SPEED),
        )
        ctrl.set_speed_ref(lambda t: (t > 0.05) * 2 * pi * 20)
        return mdl, ctrl

    values, _ = sm.export_mask_values(build()[1], SM_SPEED)
    assert start(fvc, sfunction_params(sm.FVC_BLOCK, values)) is None
    signals = {
        "w_M": lambda fbk, _: fbk.w_M,
        "tau_M_ref": lambda _, ref: ref.tau_M,
        "psi_s_ref": lambda _, ref: ref.psi_s,
        "psi_s": lambda fbk, _: np.abs(fbk.psi_s),
    }
    sfun = drive_sfun(fvc, sm.FVC_BLOCK, "theta_M")
    compare(build, model.Simulation, sfun, signals, 0.3)
    fvc.sfun_terminate()


@pytest.mark.parametrize(
    ("sensorless", "t_d", "i_0"),
    [(True, 0.0, 0.0), (False, 0.0, 0.0), (True, 2e-6, 0.0), (True, 2e-6, 0.5)],
    ids=["sensorless", "sensored", "dead_time", "dead_time_tanh"],
)
def test_current_vector_control(
    cvc: ctypes.CDLL, sensorless: bool, t_d: float, i_0: float
) -> None:
    """The S-function of current-vector control should agree with motulator, also
    with the dead time of the converter and its compensation, with the
    current-direction function np.sign or tanh(i/i_0)."""
    par = model.InductionMachineInvGammaPars(
        n_p=2, R_s=3.7, R_R=2.1, L_sgm=0.021, L_M=0.224
    )
    speed = {"J": 0.015, "alpha_s": 2 * pi * 4}

    def build() -> tuple[model.Drive, VectorControlSystem]:
        mdl = model.Drive(
            model.InductionMachine(par),
            model.MechanicalSystem(J=0.015),
            model.VoltageSourceConverter(u_dc=540, t_d=t_d),
        )
        mdl.mechanics.set_external_load_torque(lambda t: (t > 0.2) * 14.6)
        cfg = im_control.CurrentVectorControllerCfg(
            psi_s_nom=1.04, i_s_max=10.6, sensorless=sensorless
        )
        sign = np.sign if i_0 == 0 else lambda i: np.tanh(i / i_0)
        d_err = (
            None if t_d == 0 else lambda i, d: dead_time_error(i, d, t_d, cfg.T_s, sign)
        )
        ctrl = VectorControlSystem(
            im_control.CurrentVectorController(par, cfg),
            im_control.SpeedController(**speed),
            im_control.PWM(d_err=d_err),
        )
        ctrl.set_speed_ref(lambda t: (t > 0.05) * 2 * pi * 20)
        return mdl, ctrl

    values = im.export_mask_values(build()[1], speed)
    assert start(cvc, sfunction_params(im.CVC_BLOCK, values)) is None
    signals = {
        "w_M": lambda fbk, _: fbk.w_M,
        "tau_M_ref": lambda _, ref: ref.tau_M,
        "psi_R": lambda fbk, _: np.abs(fbk.psi_R),
    }
    sfun = drive_sfun(cvc, im.CVC_BLOCK, "w_M")
    compare(build, model.Simulation, sfun, signals, 0.3)
    cvc.sfun_terminate()


# %%
NOM = grid_utils.NominalValues(U=400, I=18, f=50, P=12.5e3)
BASE = grid_utils.BaseValues.from_nominal(NOM)


def grid_sfun(
    dll: ctypes.CDLL, block: ControlBlock
) -> Callable[[grid_control.GridConverterControlSystem], SFunctionControlSystem]:
    """S-function control system of a grid converter."""

    def sfun(ctrl: grid_control.GridConverterControlSystem) -> SFunctionControlSystem:
        ext_ref = cast(Any, ctrl.ext_ref)

        def inputs(mdl: grid_model.GridConverterSystem, t: float) -> list[float]:
            i_c_abc = mdl.ac_filter.meas_currents()
            u_dc = mdl.converter.meas_dc_voltage()
            if block is grid.GFL_BLOCK:
                u_g_line = mdl.ac_filter.meas_pcc_voltages()
                return [ext_ref.p_g(t), ext_ref.q_g(t), *i_c_abc, *u_g_line, u_dc]
            return [ext_ref.p_g(t), ext_ref.v_c, *i_c_abc, u_dc]

        T_s = grid._control_block(ctrl)[1]["T_s"]
        return SFunctionControlSystem(dll, block, T_s, inputs)

    return sfun


def test_grid_following_control(gfl: ctypes.CDLL) -> None:
    """The S-function of grid-following control should agree with motulator."""

    def build() -> tuple[Any, grid_control.GridConverterControlSystem]:
        ac_filter = grid_model.LCLFilter(
            L_fc=0.073 * BASE.L, L_fg=0.073 * BASE.L, C_f=0.043 * BASE.C, u_f0_ab=BASE.u
        )
        mdl = grid_model.GridConverterSystem(
            grid_model.VoltageSourceConverter(u_dc=650),
            ac_filter,
            grid_model.ThreePhaseSource(w_g=BASE.w, e_g=BASE.u),
        )
        cfg = grid_control.CurrentVectorControllerCfg(
            i_max=1.5 * BASE.i, L=0.073 * BASE.L, T_s=100e-6
        )
        ctrl = grid_control.GridConverterControlSystem(
            grid_control.CurrentVectorController(cfg)
        )
        ctrl.set_power_ref(lambda t: (t > 0.02) * 5e3)
        ctrl.set_reactive_power_ref(lambda t: (t > 0.04) * 4e3)
        return mdl, ctrl

    _, values = grid._control_block(build()[1])
    assert start(gfl, sfunction_params(grid.GFL_BLOCK, values)) is None
    signals = {
        "p_g": lambda fbk, _: fbk.p_g,
        "q_g": lambda fbk, _: fbk.q_g,
        "w_g": lambda fbk, _: fbk.w_g,
        "i_c_d_ref": lambda _, ref: ref.i_c.real,
    }
    sfun = grid_sfun(gfl, grid.GFL_BLOCK)
    compare(build, grid_model.Simulation, sfun, signals, 0.06)
    gfl.sfun_terminate()


@pytest.mark.parametrize("power_limitation", [False, True])
def test_grid_forming_control(gfm: ctypes.CDLL, power_limitation: bool) -> None:
    """The S-function of grid-forming control should agree with motulator."""

    def build() -> tuple[Any, grid_control.GridConverterControlSystem]:
        ac_filter = grid_model.LFilter(
            L_f=0.15 * BASE.L, R_f=0.05 * BASE.Z, L_g=0.74 * BASE.L
        )
        mdl = grid_model.GridConverterSystem(
            grid_model.VoltageSourceConverter(u_dc=650),
            ac_filter,
            grid_model.ThreePhaseSource(w_g=BASE.w, e_g=BASE.u),
        )
        cfg = grid_control.ObserverBasedGridFormingControllerCfg(
            i_max=1.3 * BASE.i,
            L=0.35 * BASE.L,
            R=0.05 * BASE.Z,
            R_a=0.2 * BASE.Z,
            u_nom=BASE.u,
            w_nom=BASE.w,
            i_d_max=0.85 * 1.3 * BASE.i if power_limitation else None,
        )
        ctrl = grid_control.GridConverterControlSystem(
            grid_control.ObserverBasedGridFormingController(cfg)
        )
        ctrl.set_power_ref(lambda t: (t > 0.05) * NOM.P - (t > 0.15) * 2 * NOM.P)
        ctrl.set_ac_voltage_ref(BASE.u)
        return mdl, ctrl

    _, values = grid._control_block(build()[1])
    assert start(gfm, sfunction_params(grid.GFM_BLOCK, values)) is None
    signals = {
        "p_g": lambda fbk, _: fbk.p_g,
        "q_g": lambda fbk, _: fbk.q_g,
        "v_c": lambda fbk, _: np.abs(fbk.v_c),
        "theta_c": lambda fbk, _: fbk.theta_c,
    }
    sfun = grid_sfun(gfm, grid.GFM_BLOCK)
    compare(build, grid_model.Simulation, sfun, signals, 0.25)
    gfm.sfun_terminate()


# %%
MODEL_DIR = Path(__file__).parents[1] / "examples" / "trained_models"
FLUX_MAP = MODEL_DIR / "baldor_fem_flux_map_pnorm_d12_sub20.pth"
CURRENT_MAP = MODEL_DIR / "baldor_fem_curr_map_harm_softmax_d48_sub10.pth"
RTOL = 1e-4  # motulator evaluates the GradNets in single precision


@pytest.fixture(scope="module")
def gn_machine(tmp_path_factory: pytest.TempPathFactory) -> ctypes.CDLL:
    path = tmp_path_factory.mktemp("gn_machine")
    return compile_sfunction(gradnet_machine_sfunction(), path)


def test_gradnet_flux_vector_control(fvc: ctypes.CDLL) -> None:
    """The S-function with a GradNet flux map should agree with motulator."""
    import motulator.drive.gradnet as gn  # noqa: PLC0415 (loads PyTorch)

    flux_map = gn.FluxMap(gn.load_gradnet(FLUX_MAP, activation=gn.PNormGradient))
    par = sm_control.SaturatedSynchronousMachinePars(
        n_p=2, R_s=0.63, psi_s_dq_fcn=flux_map
    )
    cfg = sm_control.FluxVectorControllerCfg(
        i_s_max=17.6, alpha_i=0, alpha_o=2 * pi * 8, J=0.05, sensorless=False
    )
    speed = {"J": 0.05, "alpha_s": 2 * pi * 4}
    ctrl = VectorControlSystem(
        sm_control.FluxVectorController(par, cfg), sm_control.SpeedController(**speed)
    )
    values, g = sm.export_mask_values(ctrl, speed)
    assert start(fvc, sfunction_params(sm.FVC_BLOCK, values, g)) is None

    # Measurements: accelerating rotor, current vector in rotor coordinates
    T_s, n = cfg.T_s, 2000
    t = np.arange(n) * T_s
    w_M = 100.0 * t / t[-1]
    theta_M = np.cumsum(w_M) * T_s
    i_s_dq = (5 + 10 * np.sin(2 * pi * 5 * t)) + 1j * (8 + 12 * np.cos(2 * pi * 3 * t))
    i_s_ab = np.exp(1j * 2 * theta_M) * i_s_dq
    ctrl.set_speed_ref(lambda t_: 50.0 if t_ > 0.05 else 0.0)
    names = ["d_a", "d_b", "d_c", *sm.FVC_BLOCK.signals]
    compared = ["d_a", "d_b", "d_c", "w_M", "tau_M_ref", "psi_s_ref", "psi_s"]
    y = (ctypes.c_double * len(names))()
    res_sl, res_py = np.zeros((n, 7)), np.zeros((n, 7))
    for k in range(n):
        th = float(wrap(theta_M[k]))
        meas = Measurements(i_s_ab[k], 540.0, w_M[k], th)
        fbk = cast(Any, ctrl.get_feedback(meas))
        ref = cast(Any, ctrl.compute_output(fbk))
        ctrl.update(ref, fbk)
        res_py[k] = [*ref.d_abc, fbk.w_M, ref.tau_M, ref.psi_s, abs(fbk.psi_s)]
        fvc.sfun_step(arr([ref.w_M, *complex2abc(i_s_ab[k]), 540.0, th]), y)
        out = dict(zip(names, y, strict=True))
        res_sl[k] = [out[name] for name in compared]
    fvc.sfun_terminate()
    err = np.max(np.abs(res_sl - res_py), axis=0)
    scale = np.max(np.abs(res_py), axis=0)
    for name, e, s in zip(compared, err, scale, strict=True):
        print(f"  {name}: max error {e:.3g} (max value {s:.3g})")
    assert np.all(err <= RTOL * np.maximum(scale, 1.0))


def test_gradnet_machine(gn_machine: ctypes.CDLL) -> None:
    """The S-function of the GradNet machine should agree with motulator."""
    import motulator.drive.gradnet as gn  # noqa: PLC0415 (loads PyTorch)

    current_map = gn.CurrentMapWithHarmonics(
        gn.load_gradnet(CURRENT_MAP, activation=gn.Softmax)
    )
    par = model.SpatialSaturatedSynchronousMachinePars(
        n_p=2, R_s=0.63, magnetic_map_fcn=current_map
    )
    machine = model.SynchronousMachine(par)
    g = sm.export_gradnet(current_map)
    params = [2.0, 0.63, g["k"], *(g[f] for f in sm.GRADNET_FIELDS)]
    assert start(gn_machine, params) is None
    x = (ctypes.c_double * 2)()
    gn_machine.sfun_states(x, None)
    assert abs(x[0] - par.psi_f) < RTOL * par.psi_f  # Initial state

    rng = np.random.default_rng(1)
    y, dx = (ctypes.c_double * 8)(), (ctypes.c_double * 2)()
    err, scale = np.zeros(3), np.zeros(3)
    for _ in range(200):
        psi = rng.uniform(-0.3, 1.0) + 1j * rng.uniform(-0.8, 0.8)
        theta_M, w_M = rng.uniform(-pi, pi), rng.uniform(-200, 200)
        u_s_ab = complex(rng.uniform(-300, 300), rng.uniform(-300, 300))
        u_a, u_b, u_c = complex2abc(u_s_ab)
        gn_machine.sfun_states(None, arr([psi.real, psi.imag]))
        gn_machine.sfun_step(arr([-(u_a - u_c), -(u_b - u_c), theta_M, w_M]), y)
        gn_machine.sfun_derivatives(dx)
        # motulator
        machine.state.psi_s_dq = psi
        machine.state.exp_j_theta_m = np.exp(1j * 2 * theta_M)
        machine.inp.u_s_ab, machine.inp.w_M = u_s_ab, w_M
        machine.set_outputs(0.0)
        d_psi = machine.rhs(0.0)[0]
        sl = [np.array(y[2:5]), y[5], dx[0] + 1j * dx[1]]
        py = [complex2abc(machine.out.i_s_ab), machine.out.tau_M, d_psi]
        err = np.maximum(
            err, [np.max(np.abs(a - b)) for a, b in zip(sl, py, strict=True)]
        )
        scale = np.maximum(scale, [np.max(np.abs(b)) for b in py])
    for name, e, s in zip(["i_s_abc", "tau_M", "d_psi_s_dq"], err, scale, strict=True):
        print(f"  {name}: max error {e:.3g} (max value {s:.3g})")
    assert np.all(err <= RTOL * scale)
