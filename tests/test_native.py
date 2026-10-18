"""
Test the native simulation against the simulation of motulator.

The drive systems are simulated in motulator and natively, and the results are
compared. With the zero-order hold (ZOH), the native simulation takes the same steps
as motulator, so the results agree to the rounding errors. With the carrier
comparison, a change of the quantized duty ratios at the rounding error changes the
switching instants, so a fine quantization is used.

Run from the repository root (requires gcc):

    pytest tests/test_native.py

"""

# %%
from math import pi
from pathlib import Path
from typing import Any

import motulator.drive.control.sm as control
import motulator.drive.gradnet as gn
import numpy as np
import pytest
from motulator.common.model import CarrierComparison, SolverCfg
from motulator.drive import model, utils

from motulator_export import StepSignal, native

MODEL_DIR = Path(__file__).parents[1] / "examples" / "trained_models"

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
    path = MODEL_DIR / "baldor_fem_curr_map_harm_softmax_d48_sub10.npz"
    current_map = gn.CurrentMapWithHarmonics(gn.load_gradnet(path))
    par = model.SpatialSaturatedSynchronousMachinePars(
        n_p=2, R_s=0.63, magnetic_map_fcn=current_map
    )
    mdl = model.Drive(
        model.SynchronousMachine(par),
        model.MechanicalSystem(J=0.05),
        model.VoltageSourceConverter(u_dc=540),
    )
    path = MODEL_DIR / "baldor_fem_flux_map_pnorm_d12_sub20.npz"
    flux_map = gn.FluxMap(gn.load_gradnet(path))
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


def simulate(build: Any, t_stop: float, **kwargs: Any) -> tuple[Any, Any]:
    """Simulate the system in motulator and natively."""
    res_py = model.Simulation(*build(), show_progress=False).simulate(t_stop, **kwargs)
    res_c = native.Simulation(*build(), show_progress=False).simulate(t_stop, **kwargs)
    assert res_py.success and res_c.success
    return res_py, res_c


def max_errors(groups_py: dict[str, Any], groups_c: dict[str, Any]) -> dict[str, float]:
    """Maximum errors of the signals relative to their maximum magnitudes."""
    errors = {}
    for group, signals in groups_py.items():
        for name, values in vars(signals).items():
            if name == "s_c_ab":
                # The current-direction vector is discontinuous at zero currents (it
                # affects the voltage only with the dead time or threshold voltage)
                continue
            x_py = np.asarray(values, dtype=complex)
            x_c = np.asarray(getattr(groups_c[group], name), dtype=complex)
            assert x_c.shape == x_py.shape, f"{group}.{name}"
            scale = max(np.max(np.abs(x_py), initial=0.0), 1e-9)
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
        names = ["machine", "mechanics", "converter"]
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

    # Unsupported systems
    mdl, ctrl = ipmsm_drive()
    with pytest.raises(NotImplementedError, match="RK45"):
        native.Simulation(mdl, ctrl, cfg=SolverCfg(method="DOP853"))
    ctrl.set_enable(False)
    with pytest.raises(NotImplementedError, match="Gate blocking"):
        native.Simulation(mdl, ctrl)
