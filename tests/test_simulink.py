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
from math import pi
from typing import Any, cast

import motulator.drive.gradnet as gn
import motulator.grid.model as grid_model
import numpy as np
import pytest
from motulator.common.utils import complex2abc
from motulator.drive import model

from motulator_export.plecs import _grid, _im, _sm
from motulator_export.simulink._sfunction import gradnet_machine_sfunction
from tests import monolithic
from tests.c_port import arr
from tests.monolithic import sfunction_params
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
    restarter,
    start,
)
from tests.systems import (
    CURRENT_MAP,
    SM_CVC_D_MIN,
    SM_PAR,
    SM_VHZ_CASES,
    SM_VHZ_IDS,
    VHZ_CASES,
    VHZ_IDS,
    VHZ_RTOL,
    gfl_system,
    gfm_system,
    gradnet_control_system,
    im_fvc_system,
    im_system,
    sm_cvc_system,
    sm_system,
    sm_vhz_system,
    vhz_system,
)


@pytest.fixture(scope="module")
def fvc(tmp_path_factory: pytest.TempPathFactory) -> ctypes.CDLL:
    return compile_sfunction(monolithic.FVC, tmp_path_factory.mktemp("fvc"))


@pytest.fixture(scope="module")
def cvc(tmp_path_factory: pytest.TempPathFactory) -> ctypes.CDLL:
    return compile_sfunction(monolithic.CVC, tmp_path_factory.mktemp("cvc"))


@pytest.fixture(scope="module")
def im_fvc(tmp_path_factory: pytest.TempPathFactory) -> ctypes.CDLL:
    return compile_sfunction(monolithic.IM_FVC, tmp_path_factory.mktemp("im_fvc"))


@pytest.fixture(scope="module")
def vhz(tmp_path_factory: pytest.TempPathFactory) -> ctypes.CDLL:
    return compile_sfunction(monolithic.VHZ, tmp_path_factory.mktemp("vhz"))


@pytest.fixture(scope="module")
def sm_cvc(tmp_path_factory: pytest.TempPathFactory) -> ctypes.CDLL:
    return compile_sfunction(monolithic.SM_CVC, tmp_path_factory.mktemp("sm_cvc"))


@pytest.fixture(scope="module")
def sm_vhz(tmp_path_factory: pytest.TempPathFactory) -> ctypes.CDLL:
    return compile_sfunction(monolithic.SM_VHZ, tmp_path_factory.mktemp("sm_vhz"))


@pytest.fixture(scope="module")
def gfl(tmp_path_factory: pytest.TempPathFactory) -> ctypes.CDLL:
    return compile_sfunction(monolithic.GFL, tmp_path_factory.mktemp("gfl"))


@pytest.fixture(scope="module")
def gfm(tmp_path_factory: pytest.TempPathFactory) -> ctypes.CDLL:
    return compile_sfunction(monolithic.GFM, tmp_path_factory.mktemp("gfm"))


# %%
def test_parameter_errors(fvc: ctypes.CDLL) -> None:
    """Missing or invalid parameters should be reported."""
    block = monolithic.FVC
    values: dict[str, Any] = dict.fromkeys(m.variable for m in block.mask_params)
    assert start(fvc, sfunction_params(block, values)) is not None
    values |= SM_PAR | {"i_s_max": 6.5, "T_s": 125e-6}
    values |= {"speed_J": 0.015, "speed_alpha_s": 25.0, "k_o": [1.0, 2.0, 3.0]}
    assert start(fvc, sfunction_params(block, values)) == "pwm_t_d must be a scalar."
    values |= {"pwm_t_d": 0.0, "pwm_u_th": 0.0, "pwm_i_0": 0.0, "pwm_feedforward": 1}
    values |= {"pwm_d_min": 0.0, "pwm_single_update": 0}
    assert start(fvc, sfunction_params(block, values)) == "k_o must be [] or [k0 k1]."
    values["k_o"] = None
    assert start(fvc, sfunction_params(block, values)) is None
    assert fvc.sfun_sample_time() == 125e-6
    fvc.sfun_terminate()


@pytest.mark.parametrize(
    ("sensorless", "t_d", "blocked"),
    [
        (True, 0.0, False),
        (False, 0.0, False),
        (True, 2e-6, False),
        (True, 0.0, True),
        (False, 0.0, True),
    ],
    ids=["sensorless", "sensored", "dead_time", "blocked", "sensored_blocked"],
)
def test_flux_vector_control(
    fvc: ctypes.CDLL, sensorless: bool, t_d: float, blocked: bool
) -> None:
    """The S-function of flux-vector control should agree with motulator, also with
    the dead time and its compensation (with the smooth current-direction function),
    and with the gates blocked (`blocked`)."""
    build, signals = sm_system(sensorless, t_d, blocked)
    values, _ = _sm.export_values(build()[1])
    params = sfunction_params(monolithic.FVC, values)
    assert start(fvc, params) is None
    compare(build, model.Simulation, drive_sfun(fvc, monolithic.FVC), signals, 0.3)
    check_dc_bus_limit(fvc, monolithic.FVC, SM_INPUTS, restarter(fvc, params))
    fvc.sfun_terminate()


@pytest.mark.parametrize(
    ("sensorless", "t_d", "i_0", "u_th", "single_update", "blocked", "discrete"),
    [
        (True, 0.0, 0.0, 0.0, False, False, False),
        (False, 0.0, 0.0, 0.0, False, False, False),
        (True, 2e-6, 0.0, 0.0, False, False, False),
        (True, 2e-6, 0.5, 0.0, False, False, False),
        (True, 2e-6, 0.5, 1.0, True, False, False),
        (True, 0.0, 0.5, 1.0, False, True, False),
        (True, 2e-6, 0.5, 0.0, False, False, True),
    ],
    ids=[
        "sensorless",
        "sensored",
        "dead_time",
        "dead_time_tanh",
        "u_th_single",
        "u_th_blocked",
        "discrete_dead_time",
    ],
)
def test_current_vector_control(
    cvc: ctypes.CDLL,
    sensorless: bool,
    t_d: float,
    i_0: float,
    u_th: float,
    single_update: bool,
    blocked: bool,
    discrete: bool,
) -> None:
    """The S-function of current-vector control should agree with motulator, also
    with the dead time and the threshold voltage of the converter and their
    compensation, with the current-direction function np.sign or tanh(i/i_0), with
    single update, with the gates blocked, and with the direct discrete-time current
    controller."""
    args = (sensorless, t_d, i_0, u_th, single_update, blocked, discrete)
    build, signals = im_system(*args)
    values = _im.export_values(build()[1])
    params = sfunction_params(monolithic.CVC, values)
    assert start(cvc, params) is None
    compare(build, model.Simulation, drive_sfun(cvc, monolithic.CVC), signals, 0.3)
    check_dc_bus_limit(cvc, monolithic.CVC, IM_INPUTS, restarter(cvc, params))
    cvc.sfun_terminate()


@pytest.mark.parametrize(
    ("sensorless", "t_d", "blocked", "d_min"),
    [(True, 0.0, False, 0.0), (False, 0.0, False, 0.0), (True, 2e-6, False, 0.2)],
    ids=["sensorless", "sensored", "dead_time_min_pulse"],
)
def test_im_flux_vector_control(
    im_fvc: ctypes.CDLL, sensorless: bool, t_d: float, blocked: bool, d_min: float
) -> None:
    """The S-function of flux-vector control of an induction machine should agree
    with motulator, also with the dead-time compensation and the minimum pulses."""
    build, signals = im_fvc_system(sensorless, t_d, blocked, d_min)
    values = _im.export_values(build()[1])
    params = sfunction_params(monolithic.IM_FVC, values)
    assert start(im_fvc, params) is None
    sfun = drive_sfun(im_fvc, monolithic.IM_FVC)
    compare(build, model.Simulation, sfun, signals, 0.3)
    check_dc_bus_limit(im_fvc, monolithic.IM_FVC, IM_INPUTS, restarter(im_fvc, params))
    im_fvc.sfun_terminate()


@pytest.mark.parametrize(
    ("open_loop", "t_d", "d_min", "blocked"), VHZ_CASES, ids=VHZ_IDS
)
def test_vhz_control(
    vhz: ctypes.CDLL, open_loop: bool, t_d: float, d_min: float, blocked: bool
) -> None:
    """The S-function of observer-based or open-loop V/Hz control should agree with
    motulator, also with the dead-time compensation, the minimum pulses, and the
    gates blocked."""
    build, signals = vhz_system(open_loop, t_d, d_min, blocked)
    values = _im.export_vhz_values(build()[1])
    params = sfunction_params(monolithic.VHZ, values)
    assert start(vhz, params) is None
    sfun = drive_sfun(vhz, monolithic.VHZ)
    compare(build, model.Simulation, sfun, signals, 1.0, VHZ_RTOL)
    check_dc_bus_limit(vhz, monolithic.VHZ, VHZ_INPUTS, restarter(vhz, params))
    vhz.sfun_terminate()


@pytest.mark.parametrize(
    ("sensorless", "t_d", "blocked", "discrete", "d_min"),
    [
        (True, 0.0, False, False, 0.0),
        (False, 0.0, False, False, 0.0),
        (True, 2e-6, False, False, 0.0),
        (True, 0.0, True, False, 0.0),
        (True, 0.0, False, True, SM_CVC_D_MIN),
        (True, 0.0, True, True, 0.0),
    ],
    ids=[
        "sensorless",
        "sensored",
        "dead_time",
        "blocked",
        "discrete_min_pulse",
        "discrete_blocked",
    ],
)
def test_sm_current_vector_control(
    sm_cvc: ctypes.CDLL,
    sensorless: bool,
    t_d: float,
    blocked: bool,
    discrete: bool,
    d_min: float,
) -> None:
    """The S-function of current-vector control of a synchronous machine should agree
    with motulator, also with the dead-time compensation, the gates blocked (the
    flying start), and the direct discrete-time current controller with the minimum
    pulses."""
    build, signals = sm_cvc_system(sensorless, t_d, blocked, discrete, d_min)
    values, _ = _sm.export_values(build()[1])
    params = sfunction_params(monolithic.SM_CVC, values)
    assert start(sm_cvc, params) is None
    compare(
        build, model.Simulation, drive_sfun(sm_cvc, monolithic.SM_CVC), signals, 0.3
    )
    check_dc_bus_limit(sm_cvc, monolithic.SM_CVC, SM_INPUTS, restarter(sm_cvc, params))
    sm_cvc.sfun_terminate()


@pytest.mark.parametrize(("t_d", "d_min", "blocked"), SM_VHZ_CASES, ids=SM_VHZ_IDS)
def test_sm_vhz_control(
    sm_vhz: ctypes.CDLL, t_d: float, d_min: float, blocked: bool
) -> None:
    """The S-function of observer-based V/Hz control of a synchronous machine should
    agree with motulator, also with the dead-time compensation, the minimum pulses,
    and the gates blocked."""
    build, signals = sm_vhz_system(t_d, d_min, blocked)
    values, _ = _sm.export_vhz_values(build()[1])
    params = sfunction_params(monolithic.SM_VHZ, values)
    assert start(sm_vhz, params) is None
    sfun = drive_sfun(sm_vhz, monolithic.SM_VHZ)
    compare(build, model.Simulation, sfun, signals, 1.0, VHZ_RTOL)
    check_dc_bus_limit(sm_vhz, monolithic.SM_VHZ, VHZ_INPUTS, restarter(sm_vhz, params))
    sm_vhz.sfun_terminate()


@pytest.mark.parametrize("blocked", [False, True], ids=["enabled", "blocked"])
def test_grid_following_control(gfl: ctypes.CDLL, blocked: bool) -> None:
    """The S-function of grid-following control should agree with motulator, also
    with the gates blocked."""
    build, signals = gfl_system(blocked)
    values = _grid.export_values(build()[1])
    params = sfunction_params(monolithic.GFL, values)
    assert start(gfl, params) is None
    sfun = grid_sfun(gfl, monolithic.GFL)
    compare(build, grid_model.Simulation, sfun, signals, 0.06)
    check_dc_bus_limit(gfl, monolithic.GFL, GFL_INPUTS, restarter(gfl, params))
    gfl.sfun_terminate()


@pytest.mark.parametrize(
    ("power_limitation", "blocked"),
    [(False, False), (True, False), (True, True)],
    ids=["no_limitation", "power_limitation", "power_limitation_blocked"],
)
def test_grid_forming_control(
    gfm: ctypes.CDLL, power_limitation: bool, blocked: bool
) -> None:
    """The S-function of grid-forming control should agree with motulator, also
    with the gates blocked."""
    build, signals = gfm_system(power_limitation, blocked)
    values = _grid.export_values(build()[1])
    params = sfunction_params(monolithic.GFM, values)
    assert start(gfm, params) is None
    sfun = grid_sfun(gfm, monolithic.GFM)
    compare(build, grid_model.Simulation, sfun, signals, 0.25)
    check_dc_bus_limit(gfm, monolithic.GFM, GFM_INPUTS, restarter(gfm, params))
    gfm.sfun_terminate()


# %%
RTOL = 1e-9  # Rounding errors


@pytest.fixture(scope="module")
def gn_machine(tmp_path_factory: pytest.TempPathFactory) -> ctypes.CDLL:
    path = tmp_path_factory.mktemp("gn_machine")
    return compile_sfunction(gradnet_machine_sfunction(), path)


def test_gradnet_flux_vector_control(fvc: ctypes.CDLL) -> None:
    """The S-function with a GradNet flux map should agree with motulator."""
    ctrl = gradnet_control_system()
    values, g = _sm.export_values(ctrl)
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
    current_map = gn.CurrentMapWithHarmonics(gn.load_gradnet(CURRENT_MAP))
    par = model.SpatialSaturatedSynchronousMachinePars(
        n_p=2, R_s=0.63, magnetic_map_fcn=current_map
    )
    machine = model.SynchronousMachine(par)
    g = _sm.export_gradnet(current_map)
    params = [2.0, 0.63, g["k"], *(g[f] for f in _sm.GRADNET_FIELDS)]
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
        machine.state.theta_m = complex(2 * theta_M)
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
