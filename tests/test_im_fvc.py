"""
Test the C port of induction machine flux-vector control against motulator.

The 2.2-kW induction machine drive of plot_2kw_im_sat_fvc.py of motulator (with the
constant-parameter machine model) is simulated in closed loop with the control
system of motulator and with the C port, and the controller signals are compared at
the sampling instants: sensorless (with field weakening to twice the nominal speed)
and sensored, with the integral action, with the dead time and its compensation, also
with the minimum pulses, with single update, and with the gates blocked at the start
and during the operation. PLECS is not needed.

Run from the repository root (requires gcc):

    pytest tests/test_im_fvc.py

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

from tests.c_port import arr, compile_library, d

CAPI = r"""
#include "common.c"
#include "im_current_vector.c"
#include "im_flux_vector.c"

static IMFVCControlSystem ctrl;

void init(const double *p, const double *c, const double *s, const double *pwm)
{
    InductionMachineInvGammaPars par = {p[0], p[1], p[2], p[3], p[4]};
    IMFluxVectorControllerCfg cfg = im_flux_vector_controller_cfg(c[0], c[1]);
    cfg.alpha_tau = c[2];
    cfg.alpha_psi = c[3];
    cfg.alpha_i = c[4];
    cfg.alpha_o = c[5];
    cfg.tau_M_max = c[6];
    cfg.k_u = c[7];
    cfg.k_b = c[8];
    cfg.J = c[9];
    cfg.sensorless = (int)c[10];
    cfg.T_s = c[11];
    PIController speed_ctrl = speed_controller(s[0], s[1], s[2], s[3]);
    im_fvc_control_system_init(&ctrl, par, &cfg, speed_ctrl);
    pwm_set_nonlinearity(&ctrl.pwm, pwm[0], pwm[1], pwm[2], (int)pwm[3]);
    pwm_set_min_pulse(&ctrl.pwm, pwm[4]);
    pwm_set_single_update(&ctrl.pwm, (int)pwm[5]);
}

void step(const double *i_s_abc, double u_dc, double w_M, double w_M_ref,
          double enabled, double *out)
{
    IMMeasurements meas = {abc2complex(i_s_abc), u_dc, w_M};
    im_fvc_control_system_compute_output(&ctrl, &meas, w_M_ref, (int)enabled);
    /* The angle of the estimated rotor flux coordinates, as in the post-processed
     * results of motulator */
    double theta_c = wrap(ctrl.fbk.theta_c + carg(ctrl.fbk.psi_R));
    double o[10] = {ctrl.ref.d_abc[0], ctrl.ref.d_abc[1],    ctrl.ref.d_abc[2],
                    ctrl.fbk.w_M,      ctrl.fbk.tau_M,       ctrl.ref.tau_M,
                    ctrl.ref.psi_s,    cabs(ctrl.fbk.psi_s), cabs(ctrl.fbk.psi_R),
                    theta_c};
    for (int k = 0; k < 10; k++) {
        out[k] = o[k];
    }
    im_fvc_control_system_update(&ctrl);
}
"""

NAMES = ["d_a", "d_b", "d_c", "w_M", "tau_M", "tau_M_ref", "psi_s_ref", "psi_s"]
NAMES += ["psi_R", "theta_c"]
NOM = utils.NominalValues(U=400, I=5, f=50, P=2.2e3, tau=14.6)
BASE = utils.BaseValues.from_nominal(NOM, n_p=2)
PAR = control.InductionMachineInvGammaPars(
    n_p=2, R_s=3.7, R_R=2.1, L_sgm=0.021, L_M=0.224
)
SPEED_CTRL = {"J": 0.015, "alpha_s": 2 * pi * 4}
T_D = 2e-6  # Dead time (s)
I_0 = 0.5  # Current scale (A) of sign = tanh(i/i_0) in the dead-time compensation
D_MIN = 0.2  # Minimum duty ratio, which limits the pulses at the speed reference


@pytest.fixture(scope="module")
def dll(tmp_path_factory: pytest.TempPathFactory) -> ctypes.CDLL:
    return compile_library(CAPI, tmp_path_factory.mktemp("im_fvc"))


def build_system(
    sensorless: bool = True,
    alpha_i: float | None = 0.0,
    t_d: float = 0.0,
    d_min: float = 0.0,
    single_update: bool = False,
    blocked: bool = False,
    speed: float = 2.0,
) -> tuple[model.Drive, control.VectorControlSystem]:
    """
    Build the drive system of the example with flux-vector control, optionally with
    the integral action (`alpha_i`, None for its default), the dead time and its
    compensation, the minimum duty ratio `d_min`, single update, the gates blocked at
    the start and during the operation (`blocked`), and the speed reference `speed`
    (p.u.).
    """
    mdl = model.Drive(
        model.InductionMachine(PAR),
        model.MechanicalSystem(J=0.015),
        model.VoltageSourceConverter(u_dc=540, t_d=t_d),
        delay=2 if single_update else 1,  # One sampling period
        single_update=single_update,
    )
    mdl.mechanics.set_external_load_torque(lambda t: (t > 0.8) * 0.5 * NOM.tau)
    cfg = control.FluxVectorControllerCfg(
        psi_s_nom=0.95 * BASE.psi,
        i_s_max=1.5 * BASE.i,
        J=0.015,
        alpha_i=alpha_i,
        sensorless=sensorless,
    )
    d_err = None
    if t_d > 0:
        d_err = control.InverterNonlinearity(t_d, sign=lambda i: np.tanh(i / I_0))
    ctrl = control.VectorControlSystem(
        control.FluxVectorController(PAR, cfg),
        control.SpeedController(**SPEED_CTRL),
        control.PWM(d_err=d_err, d_min=d_min, single_update=single_update),
    )
    ctrl.set_speed_ref(lambda t: (t > 0.2) * speed * BASE.w_M)
    if blocked:
        ctrl.set_enable(lambda t: 0.02 <= t < 0.5 or t >= 0.55)
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
        cfg = cast(control.FluxVectorController, ctrl.vector_ctrl).cfg
        self.T_s = cfg.T_s
        self.dll = dll
        p = [PAR.n_p, PAR.R_s, PAR.R_R, PAR.L_sgm, PAR.L_M]
        c = [cfg.psi_s_nom, cfg.i_s_max, cfg.alpha_tau, none(cfg.alpha_psi)]
        c += [none(cfg.alpha_i), none(cfg.alpha_o), cfg.tau_M_max, cfg.k_u, cfg.k_b]
        c += [none(cfg.J), float(cfg.sensorless), cfg.T_s]
        s = [*SPEED_CTRL.values(), none(None), cast(Any, ctrl.speed_ctrl).u_max]
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
        w_M = meas.w_M or 0.0  # Measured only in the sensored mode
        self.dll.step(
            arr(i_abc), d(meas.u_dc), d(w_M), d(w_M_ref), d(self.enabled), self.out
        )
        out = np.array(self.out)
        self.save(self.t, out=SimpleNamespace(**dict(zip(NAMES, out, strict=True))))
        self.t = (self.t + self.T_s) % 1e9
        return self.T_s, list(out[:3])

    def post_process(self) -> TimeSeries:
        return ControlSystem.post_process(self)  # Only the saved C outputs


# Arguments of build_system. With the dead time alone, the closed loop in the field
# weakening at twice the nominal speed amplifies the rounding errors (as motulator
# shows with its speed reference perturbed by 1e-15), so the nominal speed is used.
CASES: dict[str, dict[str, Any]] = {
    "sensorless": {},
    "sensored": {"sensorless": False},
    "integral": {"alpha_i": None},
    "dead_time": {"t_d": T_D, "speed": 1.0},
    "dead_time_min_pulse": {"t_d": T_D, "d_min": D_MIN},
    "single_update": {"single_update": True},
    "blocked": {"blocked": True},
}


@pytest.mark.parametrize("case", CASES)
def test_control_system(dll: ctypes.CDLL, case: str) -> None:
    """Closed-loop simulation of the drive with motulator and with the C port."""
    args = CASES[case]
    mdl, ctrl = build_system(**args)
    res_py = model.Simulation(mdl, ctrl).simulate(t_stop=1.2)
    fbk, ref = res_py.ctrl.fbk, res_py.ctrl.ref
    py = np.column_stack(
        [
            *cast(Any, ref.d_abc).T,
            fbk.w_M,
            fbk.tau_M,
            ref.tau_M,
            ref.psi_s,
            np.abs(fbk.psi_s),
            np.abs(fbk.psi_R),
            fbk.theta_c,
        ]
    )
    mdl, ctrl = build_system(**args)
    res_c = model.Simulation(mdl, CControlSystem(ctrl, dll)).simulate(t_stop=1.2)
    c = np.column_stack([getattr(res_c.ctrl.out, name) for name in NAMES])
    assert np.allclose(res_py.ctrl.t, res_c.ctrl.t)
    err = np.max(np.abs(c - py), axis=0)
    scale = np.max(np.abs(py), axis=0)
    for name, e, s in zip(NAMES, err, scale, strict=True):
        print(f"  {name}: max error {e:.3g} (max value {s:.3g})")
    assert np.all(err <= 1e-8 * np.maximum(scale, 1.0))
    # The drive runs at the speed reference
    w_M = res_c.mdl.mechanics.w_M[res_c.mdl.t > 1.15]
    w_M_ref = args.get("speed", 2.0) * BASE.w_M
    assert np.all(np.abs(w_M - w_M_ref) < 0.1 * BASE.w_M)
