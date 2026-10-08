"""
Test the C port of induction machine V/Hz control against motulator.

The 2.2-kW induction machine drive is simulated in closed loop with the observer-based
V/Hz control system of motulator and with the C port, and the controller signals are
compared at the sampling instants. Both observer-based V/Hz control and pure open-loop
V/Hz control (`L_M = inf`) are tested, also with the dead time of the converter and its
compensation and with the minimum pulses. PLECS is not needed.

Run from the repository root (requires gcc):

    pytest tests/test_im_vhz.py

"""

# %%
import ctypes
from collections.abc import Sequence
from math import inf, pi
from types import SimpleNamespace
from typing import Any, cast

import motulator.drive.control.im as control
import numpy as np
import pytest
from motulator.common.control._base import ControlSystem, TimeSeries
from motulator.common.utils import complex2abc, dead_time_error
from motulator.drive import model, utils

from tests.c_port import arr, compile_library, d

CAPI = r"""
#include "common.c"
#include "im_current_vector.c"
#include "im_flux_vector.c"

static IMVHzControlSystem ctrl;

void init(const double *p, const double *c, double slew_rate, const double *pwm)
{
    InductionMachineInvGammaPars par = {p[0], p[1], p[2], p[3], p[4]};
    IMVHzControllerCfg cfg = im_vhz_controller_cfg(c[0], c[1]);
    cfg.alpha_psi = c[2];
    cfg.alpha_tau = c[3];
    cfg.alpha_f = c[4];
    cfg.k_u = c[5];
    cfg.k_b = c[6];
    cfg.T_s = c[7];
    im_vhz_control_system_init(&ctrl, par, &cfg, slew_rate);
    pwm_set_dead_time(&ctrl.pwm, pwm[0], cfg.T_s, pwm[1], (int)pwm[2]);
    pwm_set_min_pulse(&ctrl.pwm, pwm[3]);
}

void step(const double *i_s_abc, double u_dc, double w_M_ref, double *out)
{
    IMVHzMeasurements meas = {abc2complex(i_s_abc), u_dc};
    im_vhz_control_system_compute_output(&ctrl, &meas, w_M_ref);
    double o[9] = {ctrl.ref.d_abc[0],     ctrl.ref.d_abc[1], ctrl.ref.d_abc[2],
                   ctrl.ref.w_M,          ctrl.fbk.w_s,      ctrl.fbk.tau_M,
                   ctrl.ref.tau_M,        ctrl.ref.psi_s,    cabs(ctrl.fbk.psi_s)};
    for (int k = 0; k < 9; k++) {
        out[k] = o[k];
    }
    im_vhz_control_system_update(&ctrl);
}
"""

NAMES = ["d_a", "d_b", "d_c", "w_M_ref", "w_s", "tau_M", "tau_M_ref", "psi_s_ref"]
NAMES += ["psi_s"]
NOM = utils.NominalValues(U=400, I=5, f=50, P=2.2e3, tau=14.6)
BASE = utils.BaseValues.from_nominal(NOM, n_p=2)
PAR = control.InductionMachineInvGammaPars(
    n_p=2, R_s=3.7, R_R=2.1, L_sgm=0.021, L_M=0.224
)
# Pure open-loop V/Hz control, as in plot_2kw_im_diode_vhz.py of motulator
OPEN_LOOP_PAR = control.InductionMachineInvGammaPars(
    n_p=2, R_s=0, R_R=0, L_sgm=0, L_M=inf
)
SLEW_RATE = 2 * pi * 120  # Slew rate of the speed reference (mechanical rad/s^2)
T_D = 2e-6  # Dead time (s)
I_0 = 0.5  # Current scale (A) of sign = tanh(i/i_0) in the dead-time compensation


@pytest.fixture(scope="module")
def dll(tmp_path_factory: pytest.TempPathFactory) -> ctypes.CDLL:
    return compile_library(CAPI, tmp_path_factory.mktemp("im_vhz"))


def build_system(
    open_loop: bool, t_d: float = 0.0, d_min: float = 0.0
) -> tuple[model.Drive, control.VHzControlSystem]:
    """Build the 2.2-kW drive with observer-based or pure open-loop V/Hz control."""
    mdl = model.Drive(
        model.InductionMachine(PAR),
        model.MechanicalSystem(J=0.015),
        model.VoltageSourceConverter(u_dc=540, t_d=t_d),
    )
    mdl.mechanics.set_external_load_torque(lambda t: (t > 0.6) * 0.5 * NOM.tau)
    if open_loop:
        cfg = control.ObserverBasedVHzControllerCfg(
            psi_s_nom=BASE.psi, i_s_max=inf, alpha_f=0, alpha_tau=0, alpha_psi=0
        )
        vhz_ctrl = control.ObserverBasedVHzController(OPEN_LOOP_PAR, cfg)
    else:
        cfg = control.ObserverBasedVHzControllerCfg(
            psi_s_nom=BASE.psi, i_s_max=1.5 * BASE.i
        )
        vhz_ctrl = control.ObserverBasedVHzController(PAR, cfg)

    def dead_time(i: np.ndarray, d: np.ndarray) -> np.ndarray:
        return dead_time_error(i, d, t_d, cfg.T_s, lambda x: np.tanh(x / I_0))

    d_err = dead_time if t_d > 0 else None
    pwm = control.PWM(overmodulation="MME", d_err=d_err, d_min=d_min)
    ctrl = control.VHzControlSystem(vhz_ctrl, slew_rate=SLEW_RATE, pwm=pwm)
    ctrl.set_speed_ref(lambda t: (t > 0.1) * 0.8 * BASE.w_M)
    return mdl, ctrl


class CControlSystem(control.VHzControlSystem):
    """Control system running the C port, for closed-loop simulations."""

    def __init__(
        self, ctrl: control.VHzControlSystem, dll: ctypes.CDLL, open_loop: bool
    ) -> None:
        super().__init__(ctrl.vhz_ctrl)  # For the measurements
        self.ext_ref = ctrl.ext_ref
        cfg = cast(control.ObserverBasedVHzController, ctrl.vhz_ctrl).cfg
        self.T_s = cfg.T_s
        self.dll = dll
        par = OPEN_LOOP_PAR if open_loop else PAR
        p = [par.n_p, par.R_s, par.R_R, par.L_sgm, par.L_M]
        c = [cfg.psi_s_nom, cfg.i_s_max, cfg.alpha_psi, cfg.alpha_tau, cfg.alpha_f]
        c += [cfg.k_u, cfg.k_b, cfg.T_s]
        pwm = ctrl.pwm
        t_d = 0.0 if pwm.d_err is None else T_D
        i_0 = 0.0 if pwm.d_err is None else I_0
        pwm_values = [t_d, i_0, pwm.feedforward, pwm.d_min]
        self.dll.init(arr(p), arr(c), d(ctrl.rate_limiter.rate_limit), arr(pwm_values))
        self.out = (ctypes.c_double * len(NAMES))()

    def run_control_loop(self, mdl: model.Drive) -> tuple[float, Sequence[float]]:
        meas = self.get_measurement(mdl)
        w_M_ref = cast(Any, self.ext_ref.w_M)(self.t)
        i_abc = complex2abc(meas.i_c_ab)
        self.dll.step(arr(i_abc), d(meas.u_dc), d(w_M_ref), self.out)
        out = np.array(self.out)
        self.save(self.t, out=SimpleNamespace(**dict(zip(NAMES, out, strict=True))))
        self.t = (self.t + self.T_s) % 1e9
        return self.T_s, list(out[:3])

    def post_process(self) -> TimeSeries:
        return ControlSystem.post_process(self)  # Only the saved C outputs


@pytest.mark.parametrize(
    ("open_loop", "t_d", "d_min"),
    [
        (False, 0.0, 0.0),
        (True, 0.0, 0.0),
        (False, T_D, 0.0),
        (True, T_D, 0.0),
        (False, T_D, 0.06),
    ],
    ids=["ovhz", "open_loop", "ovhz_dead_time", "open_loop_dead_time", "min_pulse"],
)
def test_control_system(
    dll: ctypes.CDLL, open_loop: bool, t_d: float, d_min: float
) -> None:
    """Closed-loop simulation of the 2.2-kW drive with motulator and with the C port."""
    mdl, ctrl = build_system(open_loop, t_d, d_min)
    res_py = model.Simulation(mdl, ctrl).simulate(t_stop=1.0)
    fbk, ref = res_py.ctrl.fbk, res_py.ctrl.ref
    py = np.column_stack(
        [
            *cast(Any, ref.d_abc).T,
            ref.w_M,
            fbk.w_s,
            fbk.tau_M,
            ref.tau_M,
            ref.psi_s,
            np.abs(fbk.psi_s),
        ]
    )
    mdl, ctrl = build_system(open_loop, t_d, d_min)
    res_c = model.Simulation(mdl, CControlSystem(ctrl, dll, open_loop)).simulate(
        t_stop=1.0
    )
    c = np.column_stack([getattr(res_c.ctrl.out, name) for name in NAMES])
    assert np.allclose(res_py.ctrl.t, res_c.ctrl.t)
    err = np.max(np.abs(c - py), axis=0)
    scale = np.max(np.abs(py), axis=0)
    for name, e, s in zip(NAMES, err, scale, strict=True):
        print(f"  {name}: max error {e:.3g} (max value {s:.3g})")
    assert np.all(err <= 1e-8 * np.maximum(scale, 1.0))
    # The drive runs at the speed reference
    w_M = res_c.mdl.mechanics.w_M[res_c.mdl.t > 0.5]
    assert np.all(np.abs(w_M - 0.8 * BASE.w_M) < 0.1 * BASE.w_M)
