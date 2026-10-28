"""
Test the C port of synchronous machine V/Hz control against motulator.

The 2.2-kW IPMSM drive is simulated in closed loop with the observer-based V/Hz
control system of motulator and with the C port, and the controller signals are
compared at the sampling instants, also with the dead time and the threshold voltage
of the converter and their compensation, with single update, with the minimum
pulses, and with the gates blocked at the start and during the operation. PLECS is
not needed. With the dead time or the threshold voltage, the system model is
sensitive to the rounding errors at the zero crossings of the currents (the voltage
error depends on their signs), so they are not tested with single update, with the
minimum pulses, or with the gates blocked, in which motulator itself differs as much
after a perturbation of the states at the rounding errors. The minimum duty ratio is
above the smallest duty ratios at the speed reference.

Run from the repository root (requires gcc):

    pytest tests/test_sm_vhz.py

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
from tests.systems import VHZ_RTOL

CAPI = r"""
#include "common.c"
#include "gradnet.c"
#include "sm_parameters.c"
#include "sm_control_loci.c"
#include "sm_flux_vector.c"

static SMVHzControlSystem ctrl;

void init(const double *p, const double *c, double slew_rate, const double *pwm)
{
    SynchronousMachinePars par = synchronous_machine_pars(p[0], p[1], p[2], p[3], p[4]);
    SMVHzControllerCfg cfg = sm_vhz_controller_cfg(c[0]);
    cfg.alpha_psi = c[1];
    cfg.alpha_tau = c[2];
    cfg.alpha_f = c[3];
    cfg.alpha_o = c[4];
    cfg.k_u = c[5];
    cfg.k_mtpv = c[6];
    cfg.psi_s_min = c[7];
    cfg.psi_s_max = c[8];
    cfg.T_s = c[9];
    sm_vhz_control_system_init(&ctrl, par, &cfg, slew_rate);
    pwm_set_nonlinearity(&ctrl.pwm, pwm[0], pwm[1], pwm[2], (int)pwm[3]);
    pwm_set_min_pulse(&ctrl.pwm, pwm[4]);
    pwm_set_single_update(&ctrl.pwm, (int)pwm[5]);
}

void step(const double *i_s_abc, double u_dc, double w_M_ref, double enabled,
          double *out)
{
    Measurements meas = {abc2complex(i_s_abc), u_dc, 0.0};
    sm_vhz_control_system_compute_output(&ctrl, &meas, w_M_ref, (int)enabled);
    double o[10] = {ctrl.ref.d_abc[0], ctrl.ref.d_abc[1],    ctrl.ref.d_abc[2],
                    ctrl.ref.w_M,      ctrl.fbk.w_m,         ctrl.fbk.tau_M,
                    ctrl.ref.tau_M,    ctrl.ref.psi_s,       cabs(ctrl.fbk.psi_s),
                    ctrl.fbk.theta_m};
    for (int k = 0; k < 10; k++) {
        out[k] = o[k];
    }
    sm_vhz_control_system_update(&ctrl);
}
"""

NAMES = ["d_a", "d_b", "d_c", "w_M_ref", "w_m", "tau_M", "tau_M_ref", "psi_s_ref"]
NAMES += ["psi_s", "theta_m"]
# 2.2-kW IPMSM of plot_2kw_ipmsm_ovhz.py of motulator
NOM = utils.NominalValues(U=370, I=4.3, f=75, P=2.2e3, tau=14)
BASE = utils.BaseValues.from_nominal(NOM, n_p=3)
PAR = model.SynchronousMachinePars(n_p=3, R_s=3.6, L_d=0.036, L_q=0.051, psi_f=0.545)
SLEW_RATE = 2 * pi * 120  # Slew rate of the speed reference (mechanical rad/s^2)
T_D = 2e-6  # Dead time (s)
U_TH = 1.0  # Threshold voltage (V)
I_0 = 0.5  # Current scale (A) of sign = tanh(i/i_0) in the dead-time compensation


@pytest.fixture(scope="module")
def dll(tmp_path_factory: pytest.TempPathFactory) -> ctypes.CDLL:
    return compile_library(CAPI, tmp_path_factory.mktemp("sm_vhz"))


def build_system(
    t_d: float = 0.0,
    d_min: float = 0.0,
    u_th: float = 0.0,
    single_update: bool = False,
    blocked: bool = False,
) -> tuple[model.Drive, control.VHzControlSystem]:
    """Build the 2.2-kW drive with observer-based V/Hz control."""
    mdl = model.Drive(
        model.SynchronousMachine(PAR),
        model.MechanicalSystem(J=0.015),
        model.VoltageSourceConverter(
            u_dc=540,
            modulator=model.ZOH(t_d=t_d, single_update=single_update),
            u_th=u_th,
        ),
    )
    mdl.mechanics.set_external_load_torque(lambda t: (t > 0.6) * 0.5 * NOM.tau)
    cfg = control.ObserverBasedVHzControllerCfg(i_s_max=1.5 * BASE.i)
    vhz_ctrl = control.ObserverBasedVHzController(PAR, cfg)
    d_err = None
    if t_d > 0 or u_th > 0:
        d_err = control.InverterNonlinearity(t_d, u_th, lambda x: np.tanh(x / I_0))
    pwm = control.PWM(
        overmodulation="MME", d_err=d_err, d_min=d_min, single_update=single_update
    )
    ctrl = control.VHzControlSystem(vhz_ctrl, slew_rate=SLEW_RATE, pwm=pwm)
    ctrl.set_speed_ref(lambda t: (t > 0.1) * 0.8 * BASE.w_M)
    if blocked:
        ctrl.set_enable(lambda t: 0.02 <= t < 0.7 or t >= 0.72)
    return mdl, ctrl


def nan_if_none(value: float | None) -> float:
    """None (the default of motulator) as NaN, resolved in C."""
    return float("nan") if value is None else value


class CControlSystem(control.VHzControlSystem):
    """Control system running the C port, for closed-loop simulations."""

    def __init__(self, ctrl: control.VHzControlSystem, dll: ctypes.CDLL) -> None:
        super().__init__(ctrl.vhz_ctrl)  # For the measurements
        self.ext_ref = ctrl.ext_ref
        self.set_enable(ctrl._enable)
        cfg = cast(control.ObserverBasedVHzController, ctrl.vhz_ctrl).cfg
        self.T_s = cfg.T_s
        self.dll = dll
        p = [PAR.n_p, PAR.R_s, PAR.L_d, PAR.L_q, PAR.psi_f]
        c = [cfg.i_s_max, cfg.alpha_psi, cfg.alpha_tau, cfg.alpha_f, cfg.alpha_o]
        c += [cfg.k_u, cfg.k_mtpv, nan_if_none(cfg.psi_s_min), cfg.psi_s_max, cfg.T_s]
        pwm = ctrl.pwm
        d_err = cast(control.InverterNonlinearity | None, pwm.d_err)
        t_d = 0.0 if d_err is None else d_err.t_d
        u_th = 0.0 if d_err is None else d_err.u_th
        i_0 = 0.0 if d_err is None else I_0
        pwm_values = [t_d, u_th, i_0, pwm.feedforward, pwm.d_min, pwm.single_update]
        self.dll.init(arr(p), arr(c), d(ctrl.rate_limiter.rate_limit), arr(pwm_values))
        self.out = (ctypes.c_double * len(NAMES))()

    def run_control_loop(self, mdl: model.Drive) -> tuple[float, Sequence[float]]:
        self.enabled = self.get_enable()
        meas = self.get_measurement(mdl)
        w_M_ref = cast(Any, self.ext_ref.w_M)(self.t)
        i_abc = complex2abc(meas.i_c_ab)
        self.dll.step(arr(i_abc), d(meas.u_dc), d(w_M_ref), d(self.enabled), self.out)
        out = np.array(self.out)
        self.save(self.t, out=SimpleNamespace(**dict(zip(NAMES, out, strict=True))))
        self.t = (self.t + self.T_s) % 1e9
        return self.T_s, list(out[:3])

    def post_process(self) -> TimeSeries:
        return ControlSystem.post_process(self)  # Only the saved C outputs


@pytest.mark.parametrize(
    ("t_d", "d_min", "u_th", "single_update", "blocked"),
    [
        (0.0, 0.0, 0.0, False, False),
        (T_D, 0.0, 0.0, False, False),
        (0.0, 0.15, 0.0, False, False),
        (0.0, 0.0, 0.0, True, False),
        (0.0, 0.0, U_TH, False, False),
        # Without the threshold voltage, since the current of the PM machine vanishes
        # with the gates blocked, so that its direction (the sign of the threshold
        # voltage of the system model) at the release is that of the rounding errors
        (0.0, 0.0, 0.0, False, True),
    ],
    ids=["ovhz", "dead_time", "min_pulse", "single_update", "u_th", "blocked"],
)
def test_control_system(
    dll: ctypes.CDLL,
    t_d: float,
    d_min: float,
    u_th: float,
    single_update: bool,
    blocked: bool,
) -> None:
    """Closed-loop simulation of the 2.2-kW drive with motulator and with the C port."""
    args = (t_d, d_min, u_th, single_update, blocked)
    mdl, ctrl = build_system(*args)
    res_py = model.Simulation(mdl, ctrl).simulate(t_stop=1.0)
    fbk, ref = res_py.ctrl.fbk, res_py.ctrl.ref
    py = np.column_stack(
        [
            *cast(Any, ref.d_abc).T,
            ref.w_M,
            fbk.w_m,
            fbk.tau_M,
            ref.tau_M,
            ref.psi_s,
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
    rtol = np.array([VHZ_RTOL.get(name, 1e-8) for name in NAMES])
    assert np.all(err <= rtol * np.maximum(scale, 1.0))
    # The drive runs at the speed reference, before the gates are blocked again
    t = res_c.mdl.t
    w_M = res_c.mdl.mechanics.w_M[(t > 0.5) & (t < 0.7)]
    assert np.all(np.abs(w_M - 0.8 * BASE.w_M) < 0.1 * BASE.w_M)
