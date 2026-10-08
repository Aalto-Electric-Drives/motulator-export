"""
Test the S-functions of the monolithic control systems and of the GradNet machine.

The S-functions are compiled with gcc against a mock of the Simulink API
(`tests/simulink_mock`). The monolithic control systems (`tests/monolithic.py`) run
the whole-system functions of the C port, and they are the references of the
control systems of the package in `test_control.py`. Each control system runs in a
closed-loop simulation of motulator, and the results are compared with those of
the control system of motulator. MATLAB is not needed.

Run from the repository root:

    pytest tests/test_simulink.py

"""

# %%
import ctypes
import subprocess
from collections.abc import Callable, Sequence
from math import inf, pi
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
from motulator.drive import model, utils
from motulator.drive.control._base import (
    Measurements,
    VectorControlSystem,
    VHzControlSystem,
)
from motulator.grid import utils as grid_utils

from motulator_export.plecs import grid, im, sm
from motulator_export.plecs._common import C_SOURCES, ControlBlock
from motulator_export.simulink._sfunction import SFunction, gradnet_machine_sfunction
from tests import monolithic
from tests.c_port import GCC, arr
from tests.monolithic import MonolithicBlock, control_sfunction, sfunction_params

MOCK = Path(__file__).parent / "simulink_mock"
Block = MonolithicBlock | ControlBlock


def compile_sfunction(sfun: MonolithicBlock | SFunction, out_dir: Path) -> ctypes.CDLL:
    """Generate the S-function (of a block) and compile it against the mock."""
    if isinstance(sfun, MonolithicBlock):
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
    return compile_sfunction(monolithic.FVC, tmp_path_factory.mktemp("fvc"))


@pytest.fixture(scope="module")
def cvc(tmp_path_factory: pytest.TempPathFactory) -> ctypes.CDLL:
    return compile_sfunction(monolithic.CVC, tmp_path_factory.mktemp("cvc"))


@pytest.fixture(scope="module")
def vhz(tmp_path_factory: pytest.TempPathFactory) -> ctypes.CDLL:
    return compile_sfunction(monolithic.VHZ, tmp_path_factory.mktemp("vhz"))


@pytest.fixture(scope="module")
def gfl(tmp_path_factory: pytest.TempPathFactory) -> ctypes.CDLL:
    return compile_sfunction(monolithic.GFL, tmp_path_factory.mktemp("gfl"))


@pytest.fixture(scope="module")
def gfm(tmp_path_factory: pytest.TempPathFactory) -> ctypes.CDLL:
    return compile_sfunction(monolithic.GFM, tmp_path_factory.mktemp("gfm"))


# Values of the parameters of the started S-functions (see start)
PARAM_VALUES: dict[int, Any] = {}


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
    # The mock refers to the values of the parameters, so they are kept, as Simulink
    # keeps the parameters
    values = PARAM_VALUES[id(dll)] = arr(np.concatenate(arrays))
    err = dll.sfun_start(len(arrays), numel, values)
    return None if err is None else err.decode()


class SFunctionControlSystem(ControlSystem):
    """Control system running the S-function, for closed-loop simulations."""

    def __init__(
        self,
        dll: Any,
        block: Block,
        T_s: float,
        inputs: Callable[[Any, float], list[float]],
    ) -> None:
        super().__init__()
        self.dll = dll
        self.T_s = T_s
        self.inputs = inputs  # Inputs of the S-function from the model and the time
        self.names = ["d_a", "d_b", "d_c", *block.signals]
        self.y = (ctypes.c_double * len(self.names))()
        self.recorded: list[list[float]] = []  # The inputs of each step

    # The control loop is overridden, so the steps of the protocol are not used
    def get_measurement(self, mdl: Any) -> Any:
        raise NotImplementedError

    def get_feedback(self, meas: Any) -> Any:
        raise NotImplementedError

    def compute_output(self, fbk: Any) -> Any:
        raise NotImplementedError

    def run_control_loop(self, mdl: Any) -> tuple[float, Sequence[float]]:
        u = self.inputs(mdl, self.t)
        self.recorded.append(u)
        self.dll.sfun_step(arr(u), self.y)
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
) -> list[list[float]]:
    """
    Simulate with the control systems of motulator and of the S-function.

    The system is built twice by `build`, and `sfun` creates the control system of
    the S-function from the control system of motulator (for its references).
    Returns the inputs of the S-function at each step.

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
    sl = sfun(ctrl)
    res_sl = simulation(mdl, sl).simulate(t_stop=t_stop)
    assert np.allclose(res.ctrl.t, res_sl.ctrl.t)
    for name, value in py.items():
        err = np.max(np.abs(getattr(res_sl.ctrl.out, name) - value))
        scale = np.max(np.abs(value))
        print(f"  {name}: max error {err:.3g} (max value {scale:.3g})")
        assert err <= 1e-8 * max(scale, 1.0), name
    return sl.recorded


def check_enable(dll: Any, block: Block, inputs: list[float]) -> None:
    """
    Check the input `enable` and the lower limit of the measured DC-bus voltage.

    The started S-function is run with the given constant inputs (without `enable`).
    Disabling should give zero voltage and reset the state to its initial value. The
    outputs should stay finite without the DC-bus voltage, e.g., if the control system
    is enabled before the DC bus is charged, the measured voltage being limited to
    U_DC_MIN = 1 V.

    """
    y = (ctypes.c_double * (3 + len(block.signals)))()

    def run(enable: float, n: int, u_dc: float | None = None) -> np.ndarray:
        u = list(inputs)
        if u_dc is not None:
            u[sum(block.input_widths[1 : block.inputs.index("u_dc")])] = u_dc
        out = np.zeros((n, len(y)))
        for k in range(n):
            dll.sfun_step(arr([enable, *u]), y)
            out[k] = y
        return out

    def disable() -> None:
        out = run(0.0, 2)
        assert np.all(out[:, :3] == 0.5)
        assert np.all(out[:, 3:] == 0)

    disable()
    first = run(1.0, 400)
    assert np.all(np.isfinite(first))
    assert np.any(first[-1, 3:] != 0)
    disable()
    assert np.array_equal(run(1.0, 400), first)
    disable()
    limited = run(1.0, 2000, 1.0)
    assert np.all(np.isfinite(limited))
    for u_dc in (0.0, -5.0):
        disable()
        assert np.array_equal(run(1.0, 2000, u_dc), limited)


# %%
# Systems of the tests: the builders of the system model and the control system of
# motulator, the compared signals, and the inputs of the control-system blocks
Build = Callable[[], tuple[Any, Any]]
Signals = dict[str, Callable[[Any, Any], Any]]

SM_PAR = {"n_p": 3, "R_s": 3.6, "L_d": 0.036, "L_q": 0.051, "psi_f": 0.545}
SM_SPEED = {"J": 0.015, "alpha_s": 25.0}
IM_PAR = {"n_p": 2, "R_s": 3.7, "R_R": 2.1, "L_sgm": 0.021, "L_M": 0.224}
IM_SPEED = {"J": 0.015, "alpha_s": 2 * pi * 4}


def smooth_sign(x: np.ndarray) -> np.ndarray:
    """Smooth current-direction function of the dead-time error model."""
    return np.tanh(x / 0.2)


def sm_system(sensorless: bool, t_d: float) -> tuple[Build, Signals]:
    """Synchronous machine drive with flux-vector control."""

    def build() -> tuple[model.Drive, VectorControlSystem]:
        par = model.SynchronousMachinePars(**SM_PAR)
        mdl = model.Drive(
            model.SynchronousMachine(par),
            model.MechanicalSystem(J=0.015),
            model.VoltageSourceConverter(u_dc=540, t_d=t_d),
        )
        mdl.mechanics.set_external_load_torque(lambda t: (t > 0.2) * 10.0)
        cfg = sm_control.FluxVectorControllerCfg(i_s_max=6.5, sensorless=sensorless)
        pwm = sm_control.PWM()
        if t_d > 0:
            pwm = sm_control.PWM(
                d_err=lambda i, d: dead_time_error(i, d, t_d, cfg.T_s, smooth_sign)
            )
        ctrl = VectorControlSystem(
            sm_control.FluxVectorController(par, cfg),
            sm_control.SpeedController(**SM_SPEED),
            pwm,
        )
        ctrl.set_speed_ref(lambda t: (t > 0.05) * 2 * pi * 20)
        return mdl, ctrl

    signals = {
        "w_M": lambda fbk, _: fbk.w_M,
        "tau_M_ref": lambda _, ref: ref.tau_M,
        "psi_s_ref": lambda _, ref: ref.psi_s,
        "psi_s": lambda fbk, _: np.abs(fbk.psi_s),
    }
    return build, signals


def im_system(sensorless: bool, t_d: float, i_0: float) -> tuple[Build, Signals]:
    """Induction machine drive with current-vector control."""
    par = model.InductionMachineInvGammaPars(**IM_PAR)

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
            im_control.SpeedController(**IM_SPEED),
            im_control.PWM(d_err=d_err),
        )
        ctrl.set_speed_ref(lambda t: (t > 0.05) * 2 * pi * 20)
        return mdl, ctrl

    signals = {
        "w_M": lambda fbk, _: fbk.w_M,
        "tau_M_ref": lambda _, ref: ref.tau_M,
        "psi_R": lambda fbk, _: np.abs(fbk.psi_R),
    }
    return build, signals


IM_NOM = utils.NominalValues(U=400, I=5, f=50, P=2.2e3, tau=14.6)
IM_BASE = utils.BaseValues.from_nominal(IM_NOM, n_p=2)
# Pure open-loop V/Hz control, as in plot_2kw_im_diode_vhz.py of motulator
OPEN_LOOP_PAR = {"n_p": 2, "R_s": 0, "R_R": 0, "L_sgm": 0, "L_M": inf}


def vhz_system(open_loop: bool, t_d: float, d_min: float) -> tuple[Build, Signals]:
    """Induction machine drive with observer-based or open-loop V/Hz control."""

    def build() -> tuple[model.Drive, VHzControlSystem]:
        mdl = model.Drive(
            model.InductionMachine(model.InductionMachineInvGammaPars(**IM_PAR)),
            model.MechanicalSystem(J=0.015),
            model.VoltageSourceConverter(u_dc=540, t_d=t_d),
        )
        mdl.mechanics.set_external_load_torque(lambda t: (t > 0.6) * 0.5 * IM_NOM.tau)
        if open_loop:
            par = im_control.InductionMachineInvGammaPars(**OPEN_LOOP_PAR)
            cfg = im_control.ObserverBasedVHzControllerCfg(
                psi_s_nom=IM_BASE.psi, i_s_max=inf, alpha_f=0, alpha_tau=0, alpha_psi=0
            )
        else:
            par = im_control.InductionMachineInvGammaPars(**IM_PAR)
            cfg = im_control.ObserverBasedVHzControllerCfg(
                psi_s_nom=IM_BASE.psi, i_s_max=1.5 * IM_BASE.i
            )
        d_err = (
            None
            if t_d == 0
            else lambda i, d: dead_time_error(i, d, t_d, cfg.T_s, smooth_sign)
        )
        pwm = im_control.PWM(overmodulation="MME", d_err=d_err, d_min=d_min)
        ctrl = VHzControlSystem(
            im_control.ObserverBasedVHzController(par, cfg),
            slew_rate=2 * pi * 120,
            pwm=pwm,
        )
        ctrl.set_speed_ref(lambda t: (t > 0.1) * 0.8 * IM_BASE.w_M)
        return mdl, ctrl

    signals = {
        "w_M_ref": lambda _, ref: ref.w_M,
        "w_s": lambda fbk, _: fbk.w_s,
        "tau_M_ref": lambda _, ref: ref.tau_M,
        "psi_s_ref": lambda _, ref: ref.psi_s,
        "psi_s": lambda fbk, _: np.abs(fbk.psi_s),
    }
    return build, signals


NOM = grid_utils.NominalValues(U=400, I=18, f=50, P=12.5e3)
BASE = grid_utils.BaseValues.from_nominal(NOM)


def gfl_system() -> tuple[Build, Signals]:
    """Grid converter system with grid-following control."""

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

    signals = {
        "p_g": lambda fbk, _: fbk.p_g,
        "q_g": lambda fbk, _: fbk.q_g,
        "w_g": lambda fbk, _: fbk.w_g,
        "i_c_d_ref": lambda _, ref: ref.i_c.real,
    }
    return build, signals


def gfm_system(power_limitation: bool) -> tuple[Build, Signals]:
    """Grid converter system with grid-forming control."""

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

    signals = {
        "p_g": lambda fbk, _: fbk.p_g,
        "q_g": lambda fbk, _: fbk.q_g,
        "v_c": lambda fbk, _: np.abs(fbk.v_c),
        "theta_c": lambda fbk, _: fbk.theta_c,
    }
    return build, signals


def drive_sfun(dll: Any, block: Block) -> Callable[[Any], SFunctionControlSystem]:
    """
    S-function control system of a drive, with the measured angle or speed (not in
    V/Hz control).
    """

    def sfun(ctrl: VectorControlSystem | VHzControlSystem) -> SFunctionControlSystem:
        def inputs(mdl: model.Drive, t: float) -> list[float]:
            mech = mdl.mechanics
            i_s_abc = mdl.machine.meas_currents()
            u_dc = mdl.converter.meas_dc_voltage()
            u = [1.0, cast(Any, ctrl.ext_ref.w_M)(t), *i_s_abc, u_dc]
            if "theta_M" in block.inputs:
                u.append(wrap(mech.meas_position()))
            elif "w_M" in block.inputs:
                u.append(mech.meas_speed())
            return u

        if isinstance(ctrl, VHzControlSystem):
            T_s = cast(Any, ctrl.vhz_ctrl).cfg.T_s
        else:
            T_s = cast(Any, ctrl.vector_ctrl).cfg.T_s
        return SFunctionControlSystem(dll, block, T_s, inputs)

    return sfun


def grid_sfun(
    dll: Any, block: Block
) -> Callable[[grid_control.GridConverterControlSystem], SFunctionControlSystem]:
    """S-function control system of a grid converter."""

    def sfun(ctrl: grid_control.GridConverterControlSystem) -> SFunctionControlSystem:
        ext_ref = cast(Any, ctrl.ext_ref)

        def inputs(mdl: grid_model.GridConverterSystem, t: float) -> list[float]:
            i_c_abc = mdl.ac_filter.meas_currents()
            u_dc = mdl.converter.meas_dc_voltage()
            if "q_g_ref" in block.inputs:
                u_g_line = mdl.ac_filter.meas_pcc_voltages()
                return [1.0, ext_ref.p_g(t), ext_ref.q_g(t), *i_c_abc, *u_g_line, u_dc]
            return [1.0, ext_ref.p_g(t), ext_ref.v_c, *i_c_abc, u_dc]

        T_s = grid.export_values(ctrl)["T_s"]
        return SFunctionControlSystem(dll, block, T_s, inputs)

    return sfun


# Constant inputs of `check_enable` (without enable)
SM_INPUTS = [100.0, 3.0, -1.0, -2.0, 540.0, 0.3]
IM_INPUTS = [100.0, 3.0, -1.0, -2.0, 540.0, 0.0]
VHZ_INPUTS = [100.0, 3.0, -1.0, -2.0, 540.0]
# Line-to-line PCC voltages u_ab and u_bc of the voltage vector BASE.u
GFL_INPUTS = [5e3, 1e3, 3.0, -1.0, -2.0, 1.5 * BASE.u, 0.0, 650.0]
GFM_INPUTS = [5e3, BASE.u, 3.0, -1.0, -2.0, 650.0]


# %%
def test_parameter_errors(fvc: ctypes.CDLL) -> None:
    """Missing or invalid parameters should be reported."""
    block = monolithic.FVC
    values: dict[str, Any] = dict.fromkeys(m.variable for m in block.mask_params)
    assert start(fvc, sfunction_params(block, values)) is not None
    values |= SM_PAR | {"i_s_max": 6.5, "T_s": 125e-6}
    values |= {"speed_J": 0.015, "speed_alpha_s": 25.0, "k_o": [1.0, 2.0, 3.0]}
    assert start(fvc, sfunction_params(block, values)) == "pwm_t_d must be a scalar."
    values |= {"pwm_t_d": 0.0, "pwm_i_0": 0.0, "pwm_feedforward": 1, "pwm_d_min": 0.0}
    assert start(fvc, sfunction_params(block, values)) == "k_o must be [] or [k0 k1]."
    values["k_o"] = None
    assert start(fvc, sfunction_params(block, values)) is None
    assert fvc.sfun_sample_time() == 125e-6
    fvc.sfun_terminate()


@pytest.mark.parametrize(
    ("sensorless", "t_d"),
    [(True, 0.0), (False, 0.0), (True, 2e-6)],
    ids=["sensorless", "sensored", "dead_time"],
)
def test_flux_vector_control(fvc: ctypes.CDLL, sensorless: bool, t_d: float) -> None:
    """The S-function of flux-vector control should agree with motulator, also with
    the dead time and its compensation (with the smooth current-direction function),
    and the input `enable` should reset it."""
    build, signals = sm_system(sensorless, t_d)
    values, _ = sm.export_values(build()[1], SM_SPEED)
    assert start(fvc, sfunction_params(monolithic.FVC, values)) is None
    compare(build, model.Simulation, drive_sfun(fvc, monolithic.FVC), signals, 0.3)
    check_enable(fvc, monolithic.FVC, SM_INPUTS)
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
    build, signals = im_system(sensorless, t_d, i_0)
    values = im.export_values(build()[1], IM_SPEED)
    assert start(cvc, sfunction_params(monolithic.CVC, values)) is None
    compare(build, model.Simulation, drive_sfun(cvc, monolithic.CVC), signals, 0.3)
    check_enable(cvc, monolithic.CVC, IM_INPUTS)
    cvc.sfun_terminate()


# Cases of V/Hz control: open loop, the dead time, and the minimum duty ratio
VHZ_CASES = [
    (False, 0.0, 0.0),
    (True, 0.0, 0.0),
    (False, 2e-6, 0.0),
    (True, 2e-6, 0.04),
]
VHZ_IDS = ["ovhz", "open_loop", "ovhz_dead_time", "open_loop_dead_time_min_pulse"]


@pytest.mark.parametrize(("open_loop", "t_d", "d_min"), VHZ_CASES, ids=VHZ_IDS)
def test_vhz_control(
    vhz: ctypes.CDLL, open_loop: bool, t_d: float, d_min: float
) -> None:
    """The S-function of observer-based or open-loop V/Hz control should agree with
    motulator, also with the dead-time compensation and the minimum pulses."""
    build, signals = vhz_system(open_loop, t_d, d_min)
    values = im.export_vhz_values(build()[1])
    assert start(vhz, sfunction_params(monolithic.VHZ, values)) is None
    compare(build, model.Simulation, drive_sfun(vhz, monolithic.VHZ), signals, 1.0)
    check_enable(vhz, monolithic.VHZ, VHZ_INPUTS)
    vhz.sfun_terminate()


def test_grid_following_control(gfl: ctypes.CDLL) -> None:
    """The S-function of grid-following control should agree with motulator."""
    build, signals = gfl_system()
    values = grid.export_values(build()[1])
    assert start(gfl, sfunction_params(monolithic.GFL, values)) is None
    sfun = grid_sfun(gfl, monolithic.GFL)
    compare(build, grid_model.Simulation, sfun, signals, 0.06)
    check_enable(gfl, monolithic.GFL, GFL_INPUTS)
    gfl.sfun_terminate()


@pytest.mark.parametrize("power_limitation", [False, True])
def test_grid_forming_control(gfm: ctypes.CDLL, power_limitation: bool) -> None:
    """The S-function of grid-forming control should agree with motulator."""
    build, signals = gfm_system(power_limitation)
    values = grid.export_values(build()[1])
    assert start(gfm, sfunction_params(monolithic.GFM, values)) is None
    sfun = grid_sfun(gfm, monolithic.GFM)
    compare(build, grid_model.Simulation, sfun, signals, 0.25)
    check_enable(gfm, monolithic.GFM, GFM_INPUTS)
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


def gradnet_control_system() -> tuple[VectorControlSystem, dict[str, float]]:
    """Flux-vector control with a GradNet flux map and its speed controller."""
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
    return ctrl, speed


def gradnet_inputs(ctrl: VectorControlSystem, n: int) -> list[tuple[Any, list[float]]]:
    """
    Measurements of the control system with a GradNet flux map, and the inputs of
    the S-function: an accelerating rotor, and a current vector in rotor coordinates.
    Later, the flux decreases until the torque-production factor c_tau of the
    flux-torque controller approaches zero, where the differences of the
    single-precision GradNet of motulator are amplified and the flux reduction
    (c_tau < 0) may start at a different sample.
    """
    T_s = cast(Any, ctrl.vector_ctrl).cfg.T_s
    t = np.arange(n) * T_s
    w_M = 100.0 * t / t[-1]
    theta_M = np.cumsum(w_M) * T_s
    i_s_dq = (5 + 10 * np.sin(2 * pi * 5 * t)) + 1j * (8 + 12 * np.cos(2 * pi * 3 * t))
    i_s_ab = np.exp(1j * 2 * theta_M) * i_s_dq
    w_M_ref = np.where(t > 0.05, 50.0, 0.0)
    out = []
    for k in range(n):
        th = float(wrap(theta_M[k]))
        meas = Measurements(i_s_ab[k], 540.0, w_M[k], th)
        u = [1.0, float(w_M_ref[k]), *complex2abc(i_s_ab[k]), 540.0, th]
        out.append((meas, u))
    return out


def test_gradnet_flux_vector_control(fvc: ctypes.CDLL) -> None:
    """The S-function with a GradNet flux map should agree with motulator."""
    ctrl, speed = gradnet_control_system()
    values, g = sm.export_values(ctrl, speed)
    assert start(fvc, sfunction_params(monolithic.FVC, values, g)) is None
    ctrl.set_speed_ref(lambda t_: 50.0 if t_ > 0.05 else 0.0)
    names = ["d_a", "d_b", "d_c", *monolithic.FVC.signals]
    compared = ["d_a", "d_b", "d_c", "w_M", "tau_M_ref", "psi_s_ref", "psi_s"]
    y = (ctypes.c_double * len(names))()
    steps = gradnet_inputs(ctrl, 1600)
    res_sl, res_py = np.zeros((len(steps), 7)), np.zeros((len(steps), 7))
    for k, (meas, u) in enumerate(steps):
        fbk = cast(Any, ctrl.get_feedback(meas))
        ref = cast(Any, ctrl.compute_output(fbk))
        ctrl.update(ref, fbk)
        res_py[k] = [*ref.d_abc, fbk.w_M, ref.tau_M, ref.psi_s, abs(fbk.psi_s)]
        u[1] = ref.w_M  # The speed reference sampled as in motulator
        fvc.sfun_step(arr(u), y)
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
