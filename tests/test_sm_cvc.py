"""
Test the C port of current-vector control of synchronous machines against motulator.

The 2.2-kW IPMSM drive of plot_2kw_ipmsm_blocked_cvc.py of motulator is simulated in
closed loop with the current-vector control system of motulator and with the C
port, and the controller signals are compared at the sampling instants: sensorless
and sensored, with the dead time and its compensation, with single update, with the
gates blocked at the start and during the operation (the flying start after the
release), and the 6.7-kW synchronous reluctance machine drive of plot_7kw_syrm_cvc.py
(without PMs, the current reference starting from the minimum flux). PLECS is not
needed.

Run from the repository root (requires gcc):

    pytest tests/test_sm_cvc.py

"""

# %%
import ctypes
from collections.abc import Sequence
from math import pi
from types import SimpleNamespace
from typing import Any, cast

import motulator.drive.control.sm as control
import numpy as np
import pytest
from motulator.common.control._base import ControlSystem, TimeSeries
from motulator.common.utils import complex2abc
from motulator.drive import model, utils

from tests.c_port import arr, compile_library, d

CAPI = r"""
#include "common.c"
#include "gradnet.c"
#include "sm_parameters.c"
#include "sm_control_loci.c"
#include "sm_flux_vector.c"
#include "sm_current_vector.c"

static SMCVCControlSystem ctrl;

void init(const double *p, const double *c, const double *s, const double *pwm)
{
    SynchronousMachinePars par = synchronous_machine_pars(p[0], p[1], p[2], p[3], p[4]);
    SMCurrentVectorControllerCfg cfg = sm_current_vector_controller_cfg(c[0]);
    cfg.alpha_c = c[1];
    cfg.alpha_i = c[2];
    cfg.alpha_o = c[3];
    cfg.alpha_ref = c[4];
    cfg.k_o[0] = c[5];
    cfg.k_o[1] = c[6];
    cfg.psi_s_min = c[7];
    cfg.psi_s_max = c[8];
    cfg.k_u = c[9];
    cfg.k_mtpv = c[10];
    cfg.J = c[11];
    cfg.T_s = c[12];
    cfg.sensorless = (int)c[13];
    PIController speed_ctrl = speed_controller(s[0], s[1], s[2], s[3]);
    sm_cvc_control_system_init(&ctrl, par, &cfg, speed_ctrl);
    pwm_set_nonlinearity(&ctrl.pwm, pwm[0], pwm[1], pwm[2], (int)pwm[3]);
    pwm_set_min_pulse(&ctrl.pwm, pwm[4]);
    pwm_set_single_update(&ctrl.pwm, (int)pwm[5]);
}

void step(const double *i_s_abc, double u_dc, double theta_M, double w_M_ref,
          double enabled, double *out)
{
    Measurements meas = {abc2complex(i_s_abc), u_dc, theta_M};
    sm_cvc_control_system_compute_output(&ctrl, &meas, w_M_ref, (int)enabled);
    double o[11] = {ctrl.ref.d_abc[0], ctrl.ref.d_abc[1], ctrl.ref.d_abc[2],
                    ctrl.fbk.w_M,      ctrl.fbk.tau_M,    ctrl.ref.tau_M,
                    ctrl.ref.psi_s,    creal(ctrl.ref.i_s), cimag(ctrl.ref.i_s),
                    cabs(ctrl.fbk.psi_s), ctrl.fbk.theta_m};
    for (int k = 0; k < 11; k++) {
        out[k] = o[k];
    }
    sm_cvc_control_system_update(&ctrl);
}
"""

NAMES = ["d_a", "d_b", "d_c", "w_M", "tau_M", "tau_M_ref", "psi_s_ref", "i_sd_ref"]
NAMES += ["i_sq_ref", "psi_s", "theta_m"]
# 2.2-kW IPMSM of plot_2kw_ipmsm_blocked_cvc.py of motulator
NOM = utils.NominalValues(U=370, I=4.3, f=75, P=2.2e3, tau=14)
BASE = utils.BaseValues.from_nominal(NOM, n_p=3)
PAR = model.SynchronousMachinePars(n_p=3, R_s=3.6, L_d=0.036, L_q=0.051, psi_f=0.545)
# 6.7-kW SyRM of plot_7kw_syrm_cvc.py of motulator
SYRM_NOM = utils.NominalValues(U=370, I=15.5, f=105.8, P=6.7e3, tau=20.1)
SYRM_BASE = utils.BaseValues.from_nominal(SYRM_NOM, n_p=2)
SYRM_PAR = model.SynchronousMachinePars(
    n_p=2, R_s=0.54, L_d=41.5e-3, L_q=6.2e-3, psi_f=0
)
W_M_REF = 0.8 * BASE.w_M  # Speed reference of the 2.2-kW drive
T_D = 2e-6  # Dead time (s)
I_0 = 0.5  # Current scale (A) of sign = tanh(i/i_0) in the dead-time compensation


@pytest.fixture(scope="module")
def dll(tmp_path_factory: pytest.TempPathFactory) -> ctypes.CDLL:
    return compile_library(CAPI, tmp_path_factory.mktemp("sm_cvc"))


def build_system(
    sensorless: bool = True,
    t_d: float = 0.0,
    single_update: bool = False,
    blocked: bool = False,
    syrm: bool = False,
) -> tuple[model.Drive, control.VectorControlSystem]:
    """
    Build the 2.2-kW drive with current-vector control, or the 6.7-kW SyRM drive of
    its example (`syrm`).
    """
    par = SYRM_PAR if syrm else PAR
    mdl = model.Drive(
        model.SynchronousMachine(par),
        model.MechanicalSystem(J=0.015),
        model.VoltageSourceConverter(u_dc=540, t_d=t_d),
        delay=2 if single_update else 1,  # One sampling period
        single_update=single_update,
    )
    if syrm:
        mdl.mechanics.set_external_load_torque(lambda t: (t > 0.6) * SYRM_NOM.tau)
        cfg = control.CurrentVectorControllerCfg(
            i_s_max=1.5 * SYRM_BASE.i,
            psi_s_min=0.5 * SYRM_BASE.psi,
            psi_s_max=1.5 * SYRM_BASE.psi,
            alpha_o=2 * pi * 100,
            sensorless=sensorless,
        )
    else:
        mdl.mechanics.set_external_load_torque(lambda t: (t > 0.3) * 0.5 * NOM.tau)
        cfg = control.CurrentVectorControllerCfg(
            i_s_max=1.5 * BASE.i, sensorless=sensorless
        )
    d_err = None
    if t_d > 0:
        d_err = control.InverterNonlinearity(t_d, sign=lambda x: np.tanh(x / I_0))
    ctrl = control.VectorControlSystem(
        control.CurrentVectorController(par, cfg),
        control.SpeedController(J=0.015, alpha_s=2 * pi * 4),
        control.PWM(d_err=d_err, single_update=single_update),
    )
    if syrm:
        ctrl.set_speed_ref(lambda t: (t > 0.2) * SYRM_BASE.w_M)
    else:
        ctrl.set_speed_ref(lambda t: (t > 0.05) * W_M_REF)
    if blocked:
        ctrl.set_enable(lambda t: 0.05 <= t < 0.6 or t >= 0.8)
    return mdl, ctrl


def none(value: float | None) -> float:
    """None (the default of motulator) as NaN, resolved in C."""
    return float("nan") if value is None else value


class CControlSystem(control.VectorControlSystem):
    """Control system running the C port, for closed-loop simulations."""

    def __init__(self, ctrl: control.VectorControlSystem, dll: ctypes.CDLL) -> None:
        super().__init__(ctrl.vector_ctrl, ctrl.speed_ctrl)  # For the measurements
        self.ext_ref = ctrl.ext_ref
        self.set_enable(ctrl._enable)
        cvc = cast(control.CurrentVectorController, ctrl.vector_ctrl)
        cfg = cvc.cfg
        self.T_s = cfg.T_s
        self.dll = dll
        par = cast(model.SynchronousMachinePars, cvc.par)
        p = [par.n_p, par.R_s, par.L_d, par.L_q, par.psi_f]
        c = [cfg.i_s_max, cfg.alpha_c, none(cfg.alpha_i), none(cfg.alpha_o)]
        c += [cfg.alpha_ref, none(None), none(None), none(cfg.psi_s_min)]
        c += [cfg.psi_s_max, cfg.k_u, cfg.k_mtpv, none(cfg.J), cfg.T_s]
        c += [float(cfg.sensorless)]
        speed_ctrl = cast(control.SpeedController, ctrl.speed_ctrl)
        s = [speed_ctrl.J, speed_ctrl.alpha_s, none(None), speed_ctrl.u_max]
        pwm = ctrl.pwm
        d_err = cast(control.InverterNonlinearity | None, pwm.d_err)
        t_d = 0.0 if d_err is None else d_err.t_d
        i_0 = 0.0 if d_err is None else I_0
        pwm_values = [t_d, 0.0, i_0, pwm.feedforward, pwm.d_min, pwm.single_update]
        self.dll.init(arr(p), arr(c), arr(s), arr(pwm_values))
        self.out = (ctypes.c_double * len(NAMES))()

    def run_control_loop(self, mdl: model.Drive) -> tuple[float, Sequence[float]]:
        self.enabled = self.get_enable()
        meas = self.get_measurement(mdl)
        w_M_ref = cast(Any, self.ext_ref.w_M)(self.t)
        i_abc = complex2abc(meas.i_c_ab)
        theta_M = 0.0 if meas.theta_M is None else meas.theta_M
        self.dll.step(
            arr(i_abc), d(meas.u_dc), d(theta_M), d(w_M_ref), d(self.enabled), self.out
        )
        out = np.array(self.out)
        self.save(self.t, out=SimpleNamespace(**dict(zip(NAMES, out, strict=True))))
        self.t = (self.t + self.T_s) % 1e9
        return self.T_s, list(out[:3])

    def post_process(self) -> TimeSeries:
        return ControlSystem.post_process(self)  # Only the saved C outputs


@pytest.mark.parametrize(
    ("sensorless", "t_d", "single_update", "blocked", "syrm"),
    [
        (True, 0.0, False, False, False),
        (False, 0.0, False, False, False),
        (True, T_D, False, False, False),
        (True, 0.0, True, False, False),
        (True, 0.0, False, True, False),
        (True, 0.0, False, False, True),
    ],
    ids=["sensorless", "sensored", "dead_time", "single_update", "blocked", "syrm"],
)
def test_control_system(
    dll: ctypes.CDLL,
    sensorless: bool,
    t_d: float,
    single_update: bool,
    blocked: bool,
    syrm: bool,
) -> None:
    """Closed-loop simulation of the drive with motulator and with the C port."""
    args = (sensorless, t_d, single_update, blocked, syrm)
    mdl, ctrl = build_system(*args)
    res_py = model.Simulation(mdl, ctrl).simulate(t_stop=1.0)
    fbk, ref = res_py.ctrl.fbk, res_py.ctrl.ref
    py = np.column_stack(
        [
            *cast(Any, ref.d_abc).T,
            fbk.w_M,
            fbk.tau_M,
            ref.tau_M,
            ref.psi_s,
            np.real(ref.i_s),
            np.imag(ref.i_s),
            np.abs(fbk.psi_s),
            fbk.theta_m,
        ]
    )
    mdl, ctrl = build_system(*args)
    res_c = model.Simulation(mdl, CControlSystem(ctrl, dll)).simulate(t_stop=1.0)
    c = np.column_stack([getattr(res_c.ctrl.out, name) for name in NAMES])
    assert np.allclose(res_py.ctrl.t, res_c.ctrl.t)
    err = np.max(np.abs(c - py), axis=0)
    scale = np.max(np.abs(py), axis=0)
    for name, e, s in zip(NAMES, err, scale, strict=True):
        print(f"  {name}: max error {e:.3g} (max value {s:.3g})")
    assert np.all(err <= 1e-8 * np.maximum(scale, 1.0))
    # The drive runs at the speed reference, also after the flying start
    t = res_c.mdl.t
    w_M_ref = SYRM_BASE.w_M if syrm else W_M_REF
    w_M = res_c.mdl.mechanics.w_M[t > 0.95]
    assert np.all(np.abs(w_M - w_M_ref) < 0.1 * w_M_ref)
