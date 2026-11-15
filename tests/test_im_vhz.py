"""
Test the C port of induction machine V/Hz control against motulator.

The 2.2-kW induction machine drive is simulated in closed loop with the V/Hz control
system of motulator and with the C port, and the controller signals are compared at
the sampling instants. Both observer-based V/Hz control and open-loop V/Hz control
(with an affine V/Hz curve) are tested, also with the dead time
and the threshold voltage of the converter and their compensation, with single
update, with the minimum pulses, and with the gates blocked at the start and during
the operation. PLECS is not needed.

Run from the repository root (requires gcc):

    pytest tests/test_im_vhz.py

"""

# %%
import ctypes
from collections.abc import Sequence
from math import pi
from types import SimpleNamespace
from typing import Any, cast

import motulator.drive.control.im as control
import numpy as np
import pytest
from motulator.common.control._base import ControlSystem, TimeSeries
from motulator.common.utils import complex2abc
from motulator.drive import model, utils

from motulator_export.plecs._im import vhz_curve
from tests.c_port import arr, compile_library, d
from tests.systems import VHZ_RTOL

CAPI = r"""
#include "common.c"
#include "im_current_vector.c"
#include "im_flux_vector.c"
#include "im_open_loop_vhz.c"

static IMVHzControlSystem ctrl;
static IMOpenLoopVHzControlSystem ol_ctrl;
static int open_loop;

static void set_pwm(PWM *pwm, const double *p)
{
    pwm_set_nonlinearity(pwm, p[0], p[1], p[2], (int)p[3]);
    pwm_set_min_pulse(pwm, p[4]);
    pwm_set_single_update(pwm, (int)p[5]);
}

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
    set_pwm(&ctrl.pwm, pwm);
    open_loop = 0;
}

void init_open_loop(double n_p, const double *u_0, const double *k, double T_s,
                    double slew_rate, const double *pwm)
{
    IMOpenLoopVHzController vhz_ctrl;
    im_open_loop_vhz_ctrl_init(&vhz_ctrl, n_p, complex_from(u_0[0], u_0[1]),
                               complex_from(k[0], k[1]), T_s);
    im_open_loop_vhz_control_system_init(&ol_ctrl, &vhz_ctrl, slew_rate);
    set_pwm(&ol_ctrl.pwm, pwm);
    open_loop = 1;
}

void step(const double *i_s_abc, double u_dc, double w_M_ref, double enabled,
          double *out)
{
    if (open_loop) {
        IMOpenLoopVHzMeasurements meas = {abc2complex(i_s_abc), u_dc};
        im_open_loop_vhz_control_system_compute_output(&ol_ctrl, &meas, w_M_ref,
                                                       (int)enabled);
        double o[9] = {ol_ctrl.ref.d_abc[0],  ol_ctrl.ref.d_abc[1],
                       ol_ctrl.ref.d_abc[2],  ol_ctrl.ref.w_M,
                       ol_ctrl.fbk.w_s,       creal(ol_ctrl.ref.u_s),
                       cimag(ol_ctrl.ref.u_s), creal(ol_ctrl.fbk.i_s),
                       cimag(ol_ctrl.fbk.i_s)};
        for (int k = 0; k < 9; k++) {
            out[k] = o[k];
        }
        im_open_loop_vhz_control_system_update(&ol_ctrl);
        return;
    }
    IMVHzMeasurements meas = {abc2complex(i_s_abc), u_dc};
    im_vhz_control_system_compute_output(&ctrl, &meas, w_M_ref, (int)enabled);
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
# The signals of open-loop V/Hz control
OPEN_LOOP_NAMES = ["d_a", "d_b", "d_c", "w_M_ref", "w_s", "u_sd_ref", "u_sq_ref"]
OPEN_LOOP_NAMES += ["i_sd", "i_sq"]
NOM = utils.NominalValues(U=400, I=5, f=50, P=2.2e3, tau=14.6)
BASE = utils.BaseValues.from_nominal(NOM, n_p=2)
PAR = control.InductionMachineInvGammaPars(
    n_p=2, R_s=3.7, R_R=2.1, L_sgm=0.021, L_M=0.224
)
SLEW_RATE = 2 * pi * 120  # Slew rate of the speed reference (mechanical rad/s^2)
T_D = 2e-6  # Dead time (s)
U_TH = 1.0  # Threshold voltage (V)
I_0 = 0.5  # Current scale (A) of sign = tanh(i/i_0) in the dead-time compensation


@pytest.fixture(scope="module")
def dll(tmp_path_factory: pytest.TempPathFactory) -> ctypes.CDLL:
    return compile_library(CAPI, tmp_path_factory.mktemp("im_vhz"))


def build_system(
    open_loop: bool,
    t_d: float = 0.0,
    d_min: float = 0.0,
    u_th: float = 0.0,
    single_update: bool = False,
    blocked: bool = False,
) -> tuple[model.Drive, control.VHzControlSystem]:
    """
    Build the 2.2-kW drive with observer-based or open-loop V/Hz control, the V/Hz
    curve of the latter including the resistive voltage drop at no load.
    """
    mdl = model.Drive(
        model.InductionMachine(PAR),
        model.MechanicalSystem(J=0.015),
        model.VoltageSourceConverter(
            u_dc=540,
            modulator=model.ZOH(t_d=t_d, single_update=single_update),
            u_th=u_th,
        ),
    )
    mdl.mechanics.set_external_load_torque(lambda t: (t > 0.6) * 0.5 * NOM.tau)
    if open_loop:
        L_s = PAR.L_M + PAR.L_sgm

        def u_s_ref(w_s: float) -> complex:
            return (PAR.R_s / L_s + 1j * w_s) * BASE.psi

        vhz_ctrl: Any = control.OpenLoopVHzController(PAR.n_p, u_s_ref)
    else:
        cfg = control.ObserverBasedVHzControllerCfg(
            psi_s_nom=BASE.psi, i_s_max=1.5 * BASE.i
        )
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


class CControlSystem(control.VHzControlSystem):
    """Control system running the C port, for closed-loop simulations."""

    def __init__(self, ctrl: control.VHzControlSystem, dll: ctypes.CDLL) -> None:
        super().__init__(ctrl.vhz_ctrl)  # For the measurements
        self.ext_ref = ctrl.ext_ref
        self.set_enable(ctrl._enable)
        self.T_s = ctrl.vhz_ctrl.T_s
        self.dll = dll
        pwm = ctrl.pwm
        d_err = cast(control.InverterNonlinearity | None, pwm.d_err)
        t_d = 0.0 if d_err is None else d_err.t_d
        u_th = 0.0 if d_err is None else d_err.u_th
        i_0 = 0.0 if d_err is None else I_0
        pwm_values = [t_d, u_th, i_0, pwm.feedforward, pwm.d_min, pwm.single_update]
        slew_rate = d(ctrl.rate_limiter.rate_limit)
        if isinstance(ctrl.vhz_ctrl, control.OpenLoopVHzController):
            self.names = OPEN_LOOP_NAMES
            u_0, k = vhz_curve(ctrl.vhz_ctrl)
            curve = [arr([u_0.real, u_0.imag]), arr([k.real, k.imag])]
            n_p, T_s = d(ctrl.vhz_ctrl.n_p), d(self.T_s)
            self.dll.init_open_loop(n_p, *curve, T_s, slew_rate, arr(pwm_values))
        else:
            self.names = NAMES
            par = PAR
            cfg = cast(control.ObserverBasedVHzController, ctrl.vhz_ctrl).cfg
            p = [par.n_p, par.R_s, par.R_R, par.L_sgm, par.L_M]
            c = [cfg.psi_s_nom, cfg.i_s_max, cfg.alpha_psi, cfg.alpha_tau]
            c += [cfg.alpha_f, cfg.k_u, cfg.k_b, cfg.T_s]
            self.dll.init(arr(p), arr(c), slew_rate, arr(pwm_values))
        self.out = (ctypes.c_double * len(self.names))()

    def run_control_loop(self, mdl: model.Drive) -> tuple[float, Sequence[float]]:
        self.enabled = self.get_enable()
        meas = self.get_measurement(mdl)
        w_M_ref = cast(Any, self.ext_ref.w_M)(self.t)
        i_abc = complex2abc(meas.i_c_ab)
        self.dll.step(arr(i_abc), d(meas.u_dc), d(w_M_ref), d(self.enabled), self.out)
        out = np.array(self.out)
        out_ = SimpleNamespace(**dict(zip(self.names, out, strict=True)))
        self.save(self.t, out=out_)
        self.t = (self.t + self.T_s) % 1e9
        return self.T_s, list(out[:3])

    def post_process(self) -> TimeSeries:
        return ControlSystem.post_process(self)  # Only the saved C outputs


@pytest.mark.parametrize(
    ("open_loop", "t_d", "d_min", "u_th", "single_update", "blocked"),
    [
        (False, 0.0, 0.0, 0.0, False, False),
        (True, 0.0, 0.0, 0.0, False, False),
        (False, T_D, 0.0, 0.0, False, False),
        (True, T_D, 0.0, 0.0, False, False),
        (False, T_D, 0.06, 0.0, False, False),
        (False, T_D, 0.06, U_TH, True, False),
        # Without the dead time, see test_im.py
        (False, 0.0, 0.0, U_TH, False, True),
        (True, 0.0, 0.0, 0.0, False, True),
    ],
    ids=[
        "ovhz",
        "open_loop",
        "ovhz_dead_time",
        "open_loop_dead_time",
        "min_pulse",
        "u_th_single",
        "u_th_blocked",
        "open_loop_blocked",
    ],
)
def test_control_system(
    dll: ctypes.CDLL,
    open_loop: bool,
    t_d: float,
    d_min: float,
    u_th: float,
    single_update: bool,
    blocked: bool,
) -> None:
    """Closed-loop simulation of the 2.2-kW drive with motulator and with the C port."""
    args = (open_loop, t_d, d_min, u_th, single_update, blocked)
    mdl, ctrl = build_system(*args)
    res_py = model.Simulation(mdl, ctrl).simulate(t_stop=1.0)
    fbk, ref = res_py.ctrl.fbk, res_py.ctrl.ref
    if open_loop:
        u_s, i_s = cast(Any, ref.u_s), cast(Any, fbk.i_s)
        signals = [u_s.real, u_s.imag, i_s.real, i_s.imag]
        names = OPEN_LOOP_NAMES
    else:
        signals = [fbk.tau_M, ref.tau_M, ref.psi_s, np.abs(fbk.psi_s)]
        names = NAMES
    py = np.column_stack([*cast(Any, ref.d_abc).T, ref.w_M, fbk.w_s, *signals])
    mdl, ctrl = build_system(*args)
    res_c = model.Simulation(mdl, CControlSystem(ctrl, dll)).simulate(t_stop=1.0)
    c = np.column_stack([getattr(res_c.ctrl.out, name) for name in names])
    assert np.allclose(res_py.ctrl.t, res_c.ctrl.t)
    err = np.max(np.abs(c - py), axis=0)
    scale = np.max(np.abs(py), axis=0)
    for name, e, s in zip(names, err, scale, strict=True):
        print(f"  {name}: max error {e:.3g} (max value {s:.3g})")
    rtol = np.array([VHZ_RTOL.get(name, 1e-8) for name in names])
    assert np.all(err <= rtol * np.maximum(scale, 1.0))
    # The drive runs at the speed reference
    w_M = res_c.mdl.mechanics.w_M[res_c.mdl.t > 0.5]
    assert np.all(np.abs(w_M - 0.8 * BASE.w_M) < 0.1 * BASE.w_M)
