"""
Test the native simulation against the simulation of motulator.

The drive systems and the grid converter systems are simulated in motulator and
natively, and the results are compared. With the zero-order hold (ZOH), the native
simulation takes the same steps as motulator, so the results agree to the rounding
errors, also with the gates blocked, when the transitions of the diode states are
located as events. With the carrier comparison, a change of the quantized duty ratios
at the rounding error changes the switching instants, so a fine quantization is used.

Run from the repository root (requires gcc):

    pytest tests/test_native.py

"""

# %%
from functools import partial
from math import inf, nan, pi, sin
from typing import Any, cast

import motulator.drive.control.im as im_control
import motulator.drive.control.sm as control
import motulator.drive.gradnet as gn
import motulator.grid.control as grid_control
import motulator.grid.model as grid_model
import numpy as np
import pytest
from motulator.common.control import PIController
from motulator.common.model import (
    CarrierComparison,
    CompositeModel,
    DCBus,
    DiodeBridge,
    SolverCfg,
    ThreePhaseSource,
)
from motulator.drive import model, utils

from motulator_export import StepSignal, native
from tests.systems import BASE as GRID_BASE
from tests.systems import (
    CURRENT_MAP,
    FLUX_MAP,
    SM_CVC_D_MIN,
    Build,
    blocked_drive,
    gfl_system,
    gfm_system,
    im_fvc_system,
    im_system,
    sm_cvc_system,
    sm_system,
    sm_vhz_system,
    vhz_system,
)
from tests.systems import NOM as GRID_NOM

# 2.2-kW IPMSM
NOM = utils.NominalValues(U=370, I=4.3, f=75, P=2.2e3, tau=14)
BASE = utils.BaseValues.from_nominal(NOM, n_p=3)


def ipmsm_drive(pwm: bool = False) -> tuple[model.Drive, control.VectorControlSystem]:
    """
    2.2-kW IPMSM drive with sensorless FVC. With `pwm`, the carrier comparison, the
    dead time, and the threshold voltage are modeled, and the PWM of the control
    system compensates for them with the single update.
    """
    par = model.SynchronousMachinePars(
        n_p=3, R_s=3.6, L_d=0.036, L_q=0.051, psi_f=0.545
    )
    t_d, u_th = (2e-6, 1.0) if pwm else (0.0, 0.0)
    mdl = model.Drive(
        model.SynchronousMachine(par),
        model.MechanicalSystem(J=0.015, B_L=0.01),
        model.VoltageSourceConverter(u_dc=540, t_d=t_d, u_th=u_th),
        pwm=pwm,
        delay=2 if pwm else 1,
        single_update=pwm,
    )
    d_err = None
    if pwm:
        mdl.pwm = CarrierComparison(N=2**24, t_d=t_d)
        d_err = control.InverterNonlinearity(t_d, u_th)
    cfg = control.FluxVectorControllerCfg(i_s_max=1.5 * BASE.i)
    ctrl = control.VectorControlSystem(
        control.FluxVectorController(par, cfg),
        control.SpeedController(J=0.015, alpha_s=2 * pi * 4),
        control.PWM(d_err=d_err, single_update=pwm),
    )
    ctrl.set_speed_ref(lambda t: (t > 0.05) * 2 * BASE.w_M)
    mdl.mechanics.set_external_load_torque(lambda t: (t > 0.3) * 0.7 * NOM.tau)
    return mdl, ctrl


def gradnet_drive() -> tuple[model.Drive, control.VectorControlSystem]:
    """
    Drive of the example plot_6kw_pmsyrm_gn_fvc_fem_harm.py: GradNet current map with
    spatial harmonics in the machine model and a GradNet flux map in the control.
    """
    nom = utils.NominalValues(U=460, I=8.8, f=60, P=5.6e3, tau=29.7)
    base = utils.BaseValues.from_nominal(nom, n_p=2)
    current_map = gn.CurrentMapWithHarmonics(gn.load_gradnet(CURRENT_MAP))
    par = model.SpatialSaturatedSynchronousMachinePars(
        n_p=2, R_s=0.63, magnetic_map_fcn=current_map
    )
    mdl = model.Drive(
        model.SynchronousMachine(par),
        model.MechanicalSystem(J=0.05),
        model.VoltageSourceConverter(u_dc=540),
    )
    flux_map = gn.FluxMap(gn.load_gradnet(FLUX_MAP))
    est_par = control.SaturatedSynchronousMachinePars(
        n_p=2, R_s=0.63, psi_s_dq_fcn=flux_map
    )
    cfg = control.FluxVectorControllerCfg(
        i_s_max=2 * base.i, alpha_i=0, alpha_o=2 * pi * 8, J=0.05, sensorless=False
    )
    ctrl = control.VectorControlSystem(
        control.FluxVectorController(est_par, cfg),
        control.SpeedController(J=0.05, alpha_s=2 * pi * 4),
    )
    ctrl.set_speed_ref(StepSignal(time=0.05, after=2 * base.w_M))
    mdl.mechanics.set_external_load_torque(StepSignal(time=0.2, after=0.5 * base.tau))
    return mdl, ctrl


def gradnet_cvc_drive(
    discrete: bool = False,
) -> tuple[model.Drive, control.VectorControlSystem]:
    """
    Drive of `gradnet_drive` with sensored current-vector control, whose machine
    model is the GradNet flux map, optionally with the direct discrete-time current
    controller (`discrete`).
    """
    mdl, fvc = gradnet_drive()
    flux_map = gn.FluxMap(gn.load_gradnet(FLUX_MAP))
    par = control.SaturatedSynchronousMachinePars(
        n_p=2, R_s=0.63, psi_s_dq_fcn=flux_map
    )
    cfg = control.CurrentVectorControllerCfg(
        i_s_max=17.6, alpha_o=2 * pi * 8, J=0.05, sensorless=False, discrete=discrete
    )
    ctrl = control.VectorControlSystem(
        control.CurrentVectorController(par, cfg),
        control.SpeedController(J=0.05, alpha_s=2 * pi * 4),
    )
    ctrl.set_speed_ref(cast(Any, fvc.ext_ref.w_M))
    return mdl, ctrl


def gradnet_vhz_drive() -> tuple[model.Drive, control.VHzControlSystem]:
    """
    Drive of `gradnet_drive` with observer-based V/Hz control, whose machine model is
    the GradNet flux map.
    """
    mdl, _ = gradnet_drive()
    flux_map = gn.FluxMap(gn.load_gradnet(FLUX_MAP))
    par = control.SaturatedSynchronousMachinePars(
        n_p=2, R_s=0.63, psi_s_dq_fcn=flux_map
    )
    cfg = control.ObserverBasedVHzControllerCfg(i_s_max=17.6)
    ctrl = control.VHzControlSystem(
        control.ObserverBasedVHzController(par, cfg), slew_rate=2 * pi * 60
    )
    ctrl.set_speed_ref(StepSignal(time=0.05, after=100.0))
    return mdl, ctrl


def simulate(build: Any, t_stop: float, **kwargs: Any) -> tuple[Any, Any]:
    """Simulate the system in motulator and natively."""
    res_py = model.Simulation(*build(), show_progress=False).simulate(t_stop, **kwargs)
    res_c = native.Simulation(*build(), show_progress=False).simulate(t_stop, **kwargs)
    assert res_py.success and res_c.success
    return res_py, res_c


def max_errors(groups_py: dict[str, Any], groups_c: dict[str, Any]) -> dict[str, float]:
    """
    Maximum errors of the signals relative to their maximum magnitudes, at least 1
    (e.g., the error signal of a locked PLL is at the level of the rounding errors).
    """
    errors = {}
    for group, signals in groups_py.items():
        for name, values in vars(signals).items():
            if name == "s_c_ab":
                # The current-direction vector is discontinuous at zero currents (it
                # affects the voltage only with the dead time or threshold voltage)
                continue
            x_py = np.asarray(values)
            x_c = np.asarray(getattr(groups_c[group], name))
            assert x_c.shape == x_py.shape, f"{group}.{name}"
            if x_py.dtype == object:  # None, e.g., the DC-bus voltage reference
                assert all(v is None for v in [*x_py, *x_c]), f"{group}.{name}"
                errors[f"{group}.{name}"] = 0.0
                continue
            x_py, x_c = x_py.astype(complex), x_c.astype(complex)
            scale = max(np.max(np.abs(x_py), initial=0.0), 1.0)
            errors[f"{group}.{name}"] = np.max(np.abs(x_c - x_py), initial=0.0) / scale
    return errors


def check(res_py: Any, res_c: Any, rtol: float, same_steps: bool = True) -> None:
    """
    Check that the control signals and, with the same solver steps, the time series of
    the system model agree.
    """
    ctrl_py, ctrl_c = res_py.ctrl, res_c.ctrl
    assert np.array_equal(ctrl_c.t, ctrl_py.t)
    errors = max_errors(
        {"ref": ctrl_py.ref, "fbk": ctrl_py.fbk}, {"ref": ctrl_c.ref, "fbk": ctrl_c.fbk}
    )
    if same_steps:
        assert np.max(np.abs(res_c.mdl.t - res_py.mdl.t)) < 1e-12
        # The time series of the subsystems (e.g., machine and converter)
        names = [name for name in vars(res_py.mdl) if name != "t"]
        mdl_py = {name: getattr(res_py.mdl, name) for name in names}
        errors |= max_errors(mdl_py, {name: getattr(res_c.mdl, name) for name in names})
    worst = max(errors, key=lambda name: errors[name])
    print(f"  max relative error {errors[worst]:.3g} ({worst})")
    assert errors[worst] < rtol


def test_ipmsm_zoh() -> None:
    """Sensorless FVC with the zero-order hold: the same steps as in motulator."""
    check(*simulate(ipmsm_drive, 0.5), rtol=1e-9)


def test_ipmsm_pwm() -> None:
    """Carrier comparison with the dead time and its compensation, single update."""
    res_py, res_c = simulate(lambda: ipmsm_drive(pwm=True), 0.4)
    check(res_py, res_c, rtol=1e-4, same_steps=False)


def test_gradnet() -> None:
    """GradNet current map with spatial harmonics in the machine model."""
    check(*simulate(gradnet_drive, 0.3), rtol=1e-9)


@pytest.mark.parametrize(
    "build",
    [
        sm_vhz_system(t_d=0.0, d_min=0.0)[0],
        sm_vhz_system(t_d=0.0, d_min=0.2)[0],
        gradnet_vhz_drive,
    ],
    ids=["smvhz", "smvhz_d_min", "smvhz_gradnet"],
)
def test_sm_vhz(build: Build) -> None:
    """Observer-based V/Hz control, also with a GradNet flux map."""
    check(*simulate(build, 0.3), rtol=1e-9)


@pytest.mark.parametrize(
    "build",
    [
        sm_cvc_system(sensorless=True, t_d=0.0)[0],
        sm_cvc_system(sensorless=False, t_d=0.0)[0],
        gradnet_cvc_drive,
        sm_cvc_system(sensorless=True, t_d=0.0, discrete=True)[0],
        sm_cvc_system(sensorless=True, t_d=0.0, discrete=True, d_min=SM_CVC_D_MIN)[0],
        sm_cvc_system(sensorless=True, t_d=2e-6, discrete=True, d_min=SM_CVC_D_MIN)[0],
        partial(gradnet_cvc_drive, discrete=True),
    ],
    ids=[
        "sensorless",
        "sensored",
        "gradnet",
        "discrete",
        "discrete_d_min",
        "discrete_d_err_d_min",
        "discrete_gradnet",
    ],
)
def test_sm_current_vector_control(build: Build) -> None:
    """Current-vector control, also with a GradNet flux map and with the direct
    discrete-time current controller, which takes the limited voltage reference of
    the PWM, also with the minimum pulses and the compensation of the duty-ratio
    error."""
    check(*simulate(build, 0.3), rtol=1e-9)


def torque_drive(
    method: str, sensorless: bool = True
) -> tuple[model.Drive, control.VectorControlSystem]:
    """
    Drive in the torque-control mode (`speed_ctrl=None`) with the external rotor speed
    (`ExternalRotorSpeed`): the 2.2-kW IPMSM with flux-vector or current-vector
    control (`method` "fvc" or "cvc"), the speed ramping up after the start, or the
    2.2-kW induction machine of plot_2kw_im_cvc_tq.py with current-vector or
    flux-vector control ("im_cvc" or "im_fvc"), the speed varying sinusoidally.
    """
    mechanics = model.ExternalRotorSpeed()
    if method.startswith("im"):
        nom = utils.NominalValues(U=400, I=5, f=50, P=2.2e3, tau=14.6)
        base = utils.BaseValues.from_nominal(nom, n_p=2)
        par = model.InductionMachineInvGammaPars(
            n_p=2, R_s=3.7, R_R=2.1, L_sgm=0.021, L_M=0.224
        )
        mdl = model.Drive(
            model.InductionMachine(par),
            mechanics,
            model.VoltageSourceConverter(u_dc=540),
        )
        if method == "im_fvc":
            vector_ctrl: Any = im_control.FluxVectorController(
                par,
                im_control.FluxVectorControllerCfg(
                    psi_s_nom=base.psi, i_s_max=1.5 * base.i, sensorless=sensorless
                ),
            )
        else:
            vector_ctrl = im_control.CurrentVectorController(
                par,
                im_control.CurrentVectorControllerCfg(
                    psi_s_nom=base.psi, i_s_max=1.5 * base.i, sensorless=sensorless
                ),
            )
        ctrl = control.VectorControlSystem(vector_ctrl, speed_ctrl=None)
        ctrl.set_torque_ref(lambda t: (t > 0.25) * nom.tau - (t > 1.25) * 2 * nom.tau)
        mechanics.set_external_rotor_speed(lambda t: 0.5 * base.w_M * sin(2 * pi * t))
        return mdl, ctrl
    par = model.SynchronousMachinePars(
        n_p=3, R_s=3.6, L_d=0.036, L_q=0.051, psi_f=0.545
    )
    mdl = model.Drive(
        model.SynchronousMachine(par), mechanics, model.VoltageSourceConverter(u_dc=540)
    )
    if method == "fvc":
        cfg: Any = control.FluxVectorControllerCfg(
            i_s_max=1.5 * BASE.i, sensorless=sensorless
        )
        vector_ctrl = control.FluxVectorController(par, cfg)
    else:
        cfg = control.CurrentVectorControllerCfg(
            i_s_max=1.5 * BASE.i, sensorless=sensorless
        )
        vector_ctrl = control.CurrentVectorController(par, cfg)
    ctrl = control.VectorControlSystem(vector_ctrl)
    ctrl.set_torque_ref(lambda t: (t > 0.1) * 0.5 * NOM.tau)
    mechanics.set_external_rotor_speed(lambda t: BASE.w_M * min(t / 0.2, 0.8))
    return mdl, ctrl


@pytest.mark.parametrize(
    ("method", "sensorless", "t_stop"),
    [
        ("fvc", True, 0.4),
        ("fvc", False, 0.4),
        ("cvc", True, 0.4),
        ("im_cvc", True, 2.0),
        ("im_fvc", True, 2.0),
    ],
    ids=["fvc", "fvc_sensored", "cvc", "im_cvc", "im_fvc"],
)
def test_torque_control(method: str, sensorless: bool, t_stop: float) -> None:
    """
    Torque-control mode (no speed controller) with the external rotor speed
    (`ExternalRotorSpeed`), whose rotor angle is the only state of the mechanics.
    """
    res_py, res_c = simulate(partial(torque_drive, method, sensorless), t_stop)
    assert np.all(res_c.ctrl.ref.w_M == None)  # noqa: E711, as in motulator
    check(res_py, res_c, rtol=1e-9)


def im_cvc_discrete_drive(
    discrete: bool,
) -> tuple[model.Drive, control.VectorControlSystem]:
    """
    The induction machine drive of plot_2kw_im_cvc_discrete.py in the torque-control
    mode at the constant external speed of 0.5 p.u., with the continuous-time or the
    direct discrete-time current controller (`discrete`) and the initial speed
    estimate of the observer.
    """
    nom = utils.NominalValues(U=400, I=5, f=50, P=2.2e3, tau=14.6)
    base = utils.BaseValues.from_nominal(nom, n_p=2)
    par = model.InductionMachineInvGammaPars(
        n_p=2, R_s=3.7, R_R=2.1, L_sgm=0.021, L_M=0.224
    )
    mdl = model.Drive(
        model.InductionMachine(par),
        model.ExternalRotorSpeed(),
        model.VoltageSourceConverter(u_dc=540),
        pwm=True,
    )
    mdl.pwm = CarrierComparison(N=2**24)  # Fine quantization
    cast(Any, mdl.mechanics).set_external_rotor_speed(lambda t: 0.5 * base.w_M)
    cfg = im_control.CurrentVectorControllerCfg(
        psi_s_nom=base.psi,
        i_s_max=2 * base.i,
        alpha_c=2 * pi * 700,
        T_s=200e-6,
        discrete=discrete,
    )
    vector_ctrl = im_control.CurrentVectorController(par, cfg)
    vector_ctrl.observer.w_M = 0.5 * base.w_M  # Initial speed estimate
    ctrl = control.VectorControlSystem(vector_ctrl)
    ctrl.set_torque_ref(
        lambda t: nom.tau * (0.3 * (t > 0.3) + 0.2 * (t > 0.32) - 0.8 * (t > 0.34))
    )
    return mdl, ctrl


@pytest.mark.parametrize("discrete", [False, True], ids=["continuous", "discrete"])
def test_initial_speed_estimate(discrete: bool) -> None:
    """
    The initial speed estimate of the observer (`vector_ctrl.observer.w_M`) in the drive
    of plot_2kw_im_cvc_discrete.py, with the carrier comparison (with fine quantization,
    whose switching instants may differ by a quantum, see `test_ipmsm_pwm`). The stop
    time 0.37 s of the example is a sampling instant (1850*T_s), at which the sampling
    instants accumulated from the switching intervals may differ by a rounding error,
    which decides whether the sampling instant is simulated, so the stop time is just
    before it.
    """
    res_py, res_c = simulate(partial(im_cvc_discrete_drive, discrete), 0.3699)
    check(res_py, res_c, rtol=1e-4, same_steps=False)


def test_n_eval() -> None:
    """Evenly spaced points of the solver (N_eval)."""
    check(*simulate(ipmsm_drive, 0.06, N_eval=4), rtol=1e-9)


def test_interface() -> None:
    """Continued simulation, the step signals, and the errors."""
    # A continued simulation equals a single one, also with step signals
    res = native.Simulation(*ipmsm_drive(), show_progress=False).simulate(0.1)
    mdl, ctrl = ipmsm_drive()
    sim = native.Simulation(mdl, ctrl, show_progress=False)
    sim.simulate(0.04)
    ctrl.set_speed_ref(StepSignal(time=0.05, after=2 * BASE.w_M))
    res_continued = sim.simulate(0.1)
    assert np.array_equal(res.mdl.t, res_continued.mdl.t)
    assert np.array_equal(res.mdl.machine.psi_s_dq, res_continued.mdl.machine.psi_s_dq)
    assert np.array_equal(res.ctrl.ref.d_abc, res_continued.ctrl.ref.d_abc)

    # An exception in a reference function is raised again
    def failing(t: float) -> float:
        if t > 0.01:
            raise ZeroDivisionError("test")
        return 0.0

    mdl, ctrl = ipmsm_drive()
    mdl.mechanics.set_external_load_torque(failing)
    with pytest.raises(ZeroDivisionError, match="test"):
        native.Simulation(mdl, ctrl, show_progress=False).simulate(0.02)

    # Invalid arguments
    for t_stop in (inf, nan):
        with pytest.raises(ValueError, match="t_stop"):
            native.Simulation(*ipmsm_drive(), show_progress=False).simulate(t_stop)

    # Unsupported systems
    mdl, ctrl = ipmsm_drive()
    with pytest.raises(NotImplementedError, match="RK45"):
        native.Simulation(mdl, ctrl, cfg=SolverCfg(method="DOP853"))


@pytest.mark.parametrize("case", ["sm", "im_nonlinearity", "sm_dc_bus"])
def test_final_state(case: str) -> None:
    """
    The model ends at the states and the switching state of the simulation, as in
    motulator, so its outputs (e.g., the converter voltage) are those of motulator.
    """
    if case == "sm":
        build: Build = ipmsm_drive
    elif case == "im_nonlinearity":  # The current directions affect the voltage
        build = im_system(
            sensorless=True, t_d=2e-6, i_0=0.0, u_th=1.0, single_update=True
        )[0]
    else:
        build = with_dc_bus(sm_system(sensorless=True, t_d=0.0)[0])
    mdl_py, ctrl_py = build()
    model.Simulation(mdl_py, ctrl_py, show_progress=False).simulate(0.06)
    mdl_c, ctrl_c = build()
    native.Simulation(mdl_c, ctrl_c, show_progress=False).simulate(0.06)
    assert mdl_c.t0 == mdl_py.t0
    np.testing.assert_allclose(
        mdl_c.get_initial_values(), mdl_py.get_initial_values(), rtol=1e-9
    )
    for name in ["q_c_ab", "s_c_ab"]:
        assert np.isclose(
            getattr(mdl_c.converter.inp, name), getattr(mdl_py.converter.inp, name)
        )
    assert np.isclose(mdl_c.converter.out.u_dc, mdl_py.converter.out.u_dc)
    assert np.isclose(mdl_c.converter.out.u_c_ab, mdl_py.converter.out.u_c_ab)
    assert np.isclose(mdl_c.machine.inp.u_s_ab, mdl_py.machine.inp.u_s_ab)


def test_final_state_blocked() -> None:
    """
    The model ends with the gate enable and the diode states of the converter of the
    simulation, here with the gates blocked, as in motulator.
    """
    build = sm_system(sensorless=True, t_d=0.0, blocked=True)[0]
    mdl_py, ctrl_py = build()
    model.Simulation(mdl_py, ctrl_py, show_progress=False).simulate(0.16)
    mdl_c, ctrl_c = build()
    native.Simulation(mdl_c, ctrl_c, show_progress=False).simulate(0.16)
    assert mdl_c.t0 == mdl_py.t0
    assert not mdl_py.converter.inp.enabled and not mdl_c.converter.inp.enabled
    assert np.array_equal(mdl_c.converter._diodes, mdl_py.converter._diodes)
    assert np.isclose(mdl_c.converter.out.u_c_ab, mdl_py.converter.out.u_c_ab)


@pytest.mark.parametrize("N_eval", [0, 4])
def test_failed_interval(N_eval: int) -> None:
    """
    Nothing of an interval in which the solver fails is saved, as in motulator, so
    the model remains at the start of the interval, and the continued simulation
    equals an uninterrupted one.
    """
    mdl, ctrl = ipmsm_drive()
    # Infinite load torque within a sampling period
    mdl.mechanics.set_external_load_torque(lambda t: inf if t > 0.05 + 50e-6 else 0.0)
    sim = native.Simulation(mdl, ctrl, show_progress=False)
    with pytest.warns(RuntimeWarning, match="Simulation stopped"):
        res = sim.simulate(0.1, N_eval)
    assert not res.success
    assert mdl.t0 == pytest.approx(0.05)
    assert res.mdl.t[-1] <= mdl.t0
    mdl.mechanics.set_external_load_torque(0.0)
    res = sim.simulate(0.1, N_eval)
    assert res.success
    mdl_ref, ctrl_ref = ipmsm_drive()
    mdl_ref.mechanics.set_external_load_torque(0.0)
    sim_ref = native.Simulation(mdl_ref, ctrl_ref, show_progress=False)
    res_ref = sim_ref.simulate(0.1, N_eval)
    assert np.array_equal(res.mdl.t, res_ref.mdl.t)
    assert np.array_equal(res.mdl.machine.psi_s_dq, res_ref.mdl.machine.psi_s_dq)
    assert np.array_equal(res.ctrl.t, res_ref.ctrl.t)
    assert np.array_equal(res.ctrl.ref.d_abc, res_ref.ctrl.ref.d_abc)


# %%
# Induction machine drives, DC buses, and grid converter systems, mostly with the
# systems of tests/systems.py
@pytest.mark.parametrize(
    "build",
    [
        im_system(sensorless=True, t_d=0.0, i_0=0.0)[0],
        im_system(sensorless=False, t_d=0.0, i_0=0.0)[0],
        im_system(sensorless=True, t_d=2e-6, i_0=0.0, u_th=1.0, single_update=True)[0],
        im_system(sensorless=True, t_d=2e-6, i_0=0.5, discrete=True)[0],
        im_system(sensorless=True, t_d=2e-6, i_0=0.5, d_min=0.2)[0],
        im_system(sensorless=True, t_d=2e-6, i_0=0.5, discrete=True, d_min=0.2)[0],
        im_fvc_system(sensorless=True)[0],
        im_fvc_system(sensorless=False, alpha_i=0.0)[0],
        im_fvc_system(sensorless=True, t_d=2e-6, d_min=0.2)[0],
        vhz_system(open_loop=False, t_d=0.0, d_min=0.0)[0],
        vhz_system(open_loop=True, t_d=0.0, d_min=0.0)[0],
        vhz_system(open_loop=False, t_d=0.0, d_min=0.1)[0],
    ],
    ids=[
        "cvc_sensorless",
        "cvc_sensored",
        "cvc_u_th_single",
        "cvc_discrete",
        "cvc_d_err_d_min",
        "cvc_discrete_d_err_d_min",
        "fvc",
        "fvc_sensored_no_integral",
        "fvc_d_err_d_min",
        "vhz",
        "vhz_open_loop",
        "vhz_d_min",
    ],
)
def test_induction_machine(build: Build) -> None:
    """Current-vector control, flux-vector control, and observer-based V/Hz control,
    also with the minimum pulses and the compensation of the duty-ratio error."""
    check(*simulate(build, 0.3), rtol=1e-9)


def test_quadratic_load() -> None:
    """Friction coefficient as a function of the speed (a quadratic load torque)."""
    build = vhz_system(open_loop=False, t_d=0.0, d_min=0.0)[0]

    def build_quadratic() -> tuple[model.Drive, Any]:
        mdl, ctrl = build()
        mechanics = model.MechanicalSystem(J=0.015, B_L=lambda w_M: 1e-4 * abs(w_M))
        return model.Drive(mdl.machine, mechanics, mdl.converter), ctrl

    check(*simulate(build_quadratic, 0.3), rtol=1e-9)


def test_pi_speed_controller() -> None:
    """
    Any PI controller is accepted as the speed controller of an induction machine
    drive, as of a synchronous machine drive.
    """

    def build() -> tuple[model.Drive, Any]:
        mdl, ctrl = im_system(sensorless=True, t_d=0.0, i_0=0.0)[0]()
        ctrl.speed_ctrl = PIController(k_p=1.0, k_i=16.0, k_t=1.0, u_max=20.0)
        return mdl, ctrl

    check(*simulate(build, 0.3), rtol=1e-9)


def with_dc_bus(build: Build) -> Build:
    """The drive of `build` fed by a DC-bus model (DCBus)."""

    def build_dc() -> tuple[model.Drive, Any]:
        mdl, ctrl = build()
        c = mdl.converter
        converter = model.VoltageSourceConverter(t_d=c.t_d, u_th=c.u_th)
        dc_bus = DCBus(C_dc=1e-3, u_dc=540)
        return model.Drive(mdl.machine, mdl.mechanics, converter, dc_bus=dc_bus), ctrl

    return build_dc


@pytest.mark.parametrize("machine", ["sm", "im"])
def test_dc_bus(machine: str) -> None:
    """DC-bus model feeding the converter of the drive."""
    if machine == "sm":
        build = sm_system(sensorless=True, t_d=0.0)[0]
    else:
        build = im_system(sensorless=True, t_d=0.0, i_0=0.0)[0]
    res_py, res_c = simulate(with_dc_bus(build), 0.3)
    check(res_py, res_c, rtol=1e-9)


def with_diode_bridge(build: Build) -> Build:
    """
    The drive of `build` fed from a diode bridge, with the threshold voltage of its
    diodes, through an L filter with its resistance, as a composite model.
    """

    def build_bridge() -> tuple[CompositeModel, dict[str, Any]]:
        mdl, ctrl = with_dc_bus(build)()
        grid = grid_model.GridConverterSystem(
            DiodeBridge(u_th=1.0),
            grid_model.LFilter(L_f=0.5e-3, R_f=0.05),
            ThreePhaseSource(w_g=2 * np.pi * 50, e_g=np.sqrt(2 / 3) * 400),
            dc_bus=mdl.dc_bus,
        )
        return CompositeModel({"grid": grid, "drive": mdl}), {"drive": ctrl}

    return build_bridge


@pytest.mark.parametrize("machine", ["sm", "im"])
def test_diode_bridge(machine: str) -> None:
    """
    Drive fed from a diode bridge: the drive and the diode bridge with its L filter
    and source agree, also the transitions of the diode states.
    """
    if machine == "sm":
        build = sm_system(sensorless=True, t_d=0.0)[0]
    else:
        build = im_system(sensorless=True, t_d=0.0, i_0=0.0)[0]
    res_py, res_c = simulate(with_diode_bridge(build), 0.3)
    assert res_py.ctrl.grid is None and res_c.ctrl.grid is None
    drive_py, drive_c = (
        model.SimulationResults(res.mdl.drive, res.ctrl.drive)
        for res in (res_py, res_c)
    )
    check(drive_py, drive_c, rtol=1e-9)
    names = [name for name in vars(res_py.mdl.grid) if name != "t"]
    errors = max_errors(
        {name: getattr(res_py.mdl.grid, name) for name in names},
        {name: getattr(res_c.mdl.grid, name) for name in names},
    )
    assert max(errors.values()) < 1e-9


def gfl_variant(lcl: bool, pwm: bool = False, dc_bus: bool = False) -> Build:
    """
    Grid-following control with the resistances and the grid impedance of the filter,
    a voltage dip, a phase shift, and a negative-sequence component of the source, and
    optionally the carrier comparison and a DC-bus model.
    """
    B = GRID_BASE

    def build() -> tuple[grid_model.GridConverterSystem, Any]:
        if lcl:
            ac_filter: Any = grid_model.LCLFilter(
                L_fc=0.073 * B.L,
                L_fg=0.073 * B.L,
                C_f=0.043 * B.C,
                R_fc=0.01 * B.Z,
                R_fg=0.01 * B.Z,
                L_g=0.2 * B.L,
                R_g=0.01 * B.Z,
                u_f0_ab=B.u,
            )
        else:
            ac_filter = grid_model.LFilter(
                L_f=0.15 * B.L, R_f=0.02 * B.Z, L_g=0.1 * B.L, R_g=0.01 * B.Z
            )
        ac_source = grid_model.ThreePhaseSource(
            w_g=B.w,
            e_g=StepSignal(time=0.1, after=0.8 * B.u, before=B.u),
            phi=0.1,
            e_g_neg=0.05 * B.u,
            phi_neg=0.3,
        )
        converter = grid_model.VoltageSourceConverter(u_dc=None if dc_bus else 650)
        dc = DCBus(C_dc=1e-3, u_dc=650) if dc_bus else None
        mdl = grid_model.GridConverterSystem(
            converter, ac_filter, ac_source, dc_bus=dc, pwm=pwm
        )
        if pwm:
            mdl.pwm = CarrierComparison(N=2**24)
        cfg = grid_control.CurrentVectorControllerCfg(
            i_max=1.5 * B.i, L=0.073 * B.L, T_s=100e-6
        )
        ctrl = grid_control.GridConverterControlSystem(
            grid_control.CurrentVectorController(cfg)
        )
        ctrl.set_power_ref(StepSignal(time=0.02, after=5e3))
        ctrl.set_reactive_power_ref(lambda t: (t > 0.04) * 4e3)
        return mdl, ctrl

    return build


def gfm_variant() -> Build:
    """Grid-forming control with an LCL filter and a step in the grid frequency."""
    B = GRID_BASE

    def build() -> tuple[grid_model.GridConverterSystem, Any]:
        ac_filter = grid_model.LCLFilter(
            L_fc=0.15 * B.L,
            L_fg=0.1 * B.L,
            C_f=0.043 * B.C,
            R_fc=0.05 * B.Z,
            L_g=0.5 * B.L,
        )
        ac_source = grid_model.ThreePhaseSource(
            w_g=lambda t: B.w * (1 + 0.01 * (t > 0.1)), e_g=B.u
        )
        mdl = grid_model.GridConverterSystem(
            grid_model.VoltageSourceConverter(u_dc=650), ac_filter, ac_source
        )
        cfg = grid_control.ObserverBasedGridFormingControllerCfg(
            i_max=1.3 * B.i,
            L=0.35 * B.L,
            R=0.05 * B.Z,
            R_a=0.2 * B.Z,
            u_nom=B.u,
            w_nom=B.w,
        )
        ctrl = grid_control.GridConverterControlSystem(
            grid_control.ObserverBasedGridFormingController(cfg)
        )
        ctrl.set_power_ref(lambda t: (t > 0.05) * GRID_NOM.P)
        ctrl.set_ac_voltage_ref(B.u)
        return mdl, ctrl

    return build


@pytest.mark.parametrize(
    "build",
    [
        gfl_system()[0],
        gfl_variant(lcl=False),
        gfl_variant(lcl=True),
        gfl_variant(lcl=True, dc_bus=True),
        gfm_system(power_limitation=False)[0],
        gfm_system(power_limitation=True)[0],
        gfm_variant(),
    ],
    ids=[
        "gfl",
        "gfl_l_filter",
        "gfl_lcl_filter",
        "gfl_dc_bus",
        "gfm",
        "gfm_power_limitation",
        "gfm_lcl_filter",
    ],
)
def test_grid(build: Build) -> None:
    """Grid-following and grid-forming control."""
    check(*simulate(build, 0.2), rtol=1e-9)


def test_grid_after_simulation() -> None:
    """
    The control system runs in C from its initial state, also if it has been
    simulated in motulator, and its parameters are read from its configuration
    (not from the state of the observer of grid-forming control).
    """

    def build() -> tuple[grid_model.GridConverterSystem, Any]:
        mdl, ctrl = gfm_system(power_limitation=False)[0]()
        ctrl.set_power_ref(0.5 * GRID_NOM.P)  # Constant, independent of ctrl.t
        return mdl, ctrl

    mdl, ctrl = build()
    model.Simulation(mdl, ctrl, show_progress=False).simulate(0.02)
    res_c = native.Simulation(build()[0], ctrl, show_progress=False).simulate(0.1)
    res_ref = native.Simulation(*build(), show_progress=False).simulate(0.1)
    n = len(res_ref.ctrl.t)
    assert np.array_equal(res_c.ctrl.ref.d_abc[-n:], res_ref.ctrl.ref.d_abc)


def test_grid_pwm() -> None:
    """Grid-following control with the carrier comparison."""
    res_py, res_c = simulate(gfl_variant(lcl=True, pwm=True), 0.2)
    check(res_py, res_c, rtol=1e-6, same_steps=False)


# %%
# Gate blocking
@pytest.mark.parametrize(
    ("build", "t_stop"),
    [
        (sm_system(sensorless=True, t_d=0.0, blocked=True)[0], 0.3),
        (sm_system(sensorless=False, t_d=0.0, blocked=True)[0], 0.3),
        (sm_cvc_system(sensorless=True, t_d=0.0, blocked=True)[0], 0.3),
        (sm_cvc_system(True, 0.0, blocked=True, discrete=True)[0], 0.3),
        (im_system(sensorless=True, t_d=0.0, i_0=0.0, blocked=True)[0], 0.3),
        (im_system(True, 0.0, 0.0, blocked=True, discrete=True)[0], 0.3),
        (vhz_system(open_loop=False, t_d=0.0, d_min=0.0, blocked=True)[0], 0.3),
        (vhz_system(open_loop=True, t_d=0.0, d_min=0.0, blocked=True)[0], 0.3),
        (sm_vhz_system(t_d=0.0, d_min=0.0, blocked=True)[0], 0.8),
        (gfl_system(blocked=True)[0], 0.06),
        (gfm_system(power_limitation=False, blocked=True)[0], 0.15),
    ],
    ids=[
        "fvc",
        "fvc_sensored",
        "smcvc",
        "smcvc_discrete",
        "cvc",
        "cvc_discrete",
        "vhz",
        "vhz_open_loop",
        "smvhz",
        "gfl",
        "gfm",
    ],
)
def test_gate_blocking(build: Build, t_stop: float) -> None:
    """
    With the gates blocked at the start and during the operation (see tests/systems.py),
    the diodes of the converter conduct according to the currents, and the native
    simulation takes the same steps as motulator, also at the events of the diode
    transitions.
    """
    res_py, res_c = simulate(build, t_stop)
    assert not np.all(res_c.mdl.converter.enabled)
    check(res_py, res_c, rtol=1e-9)


def test_gate_blocking_im_fvc() -> None:
    """
    Gate blocking in flux-vector control of an induction machine, as in
    `test_gate_blocking`, to the tolerance of the torque reference limited by the
    current while the rotor flux builds up after the release (see IM_FVC_RTOL).
    """
    res_py, res_c = simulate(im_fvc_system(sensorless=True, blocked=True)[0], 0.3)
    assert not np.all(res_c.mdl.converter.enabled)
    check(res_py, res_c, rtol=1e-6)


def test_gate_blocking_gradnet() -> None:
    """
    The gates blocked with the GradNet current map with spatial harmonics, whose AC
    port includes the current derivative due to the harmonics.
    """

    def build() -> tuple[model.Drive, control.VectorControlSystem]:
        mdl, ctrl = gradnet_drive()
        ctrl.set_enable(blocked_drive)
        return mdl, ctrl

    check(*simulate(build, 0.2), rtol=1e-9)


def test_gate_blocking_n_eval() -> None:
    """The events of the diode transitions with the evenly spaced points (N_eval)."""
    build = sm_system(sensorless=True, t_d=0.0, blocked=True)[0]
    check(*simulate(build, 0.2, N_eval=4), rtol=1e-9)


def test_unsupported() -> None:
    """Unsupported systems raise errors."""
    # Gate enable other than 0 and 1
    mdl, ctrl = gfl_system()[0]()
    ctrl.set_enable(StepSignal(time=0.01, after=0.5))
    with pytest.raises(ValueError, match="gate enable"):
        native.Simulation(mdl, ctrl)
    # DC-bus voltage control
    mdl, ctrl = gfl_system()[0]()
    ctrl.dc_bus_voltage_ctrl = grid_control.DCBusVoltageController(
        C_dc=1e-3, alpha_dc=2 * pi * 30
    )
    with pytest.raises(NotImplementedError, match="DC-bus voltage control"):
        native.Simulation(mdl, ctrl)
    # External DC current
    mdl, ctrl = with_dc_bus(sm_system(sensorless=True, t_d=0.0)[0])()
    cast_dc_bus: Any = mdl.dc_bus
    cast_dc_bus.set_external_dc_current(1.0)
    with pytest.raises(NotImplementedError, match="External DC current"):
        native.Simulation(mdl, ctrl)
