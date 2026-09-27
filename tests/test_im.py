"""
Test the C port of induction machine current-vector control against motulator.

The 2.2-kW induction machine drive (see im_2kw_cvc.py) is simulated in closed loop
with the control system of motulator and with the C port (in the sensorless and
sensored modes), and the controller signals are compared at the sampling instants.
The closed-loop test is used, since replaying recorded measurements to a sensorless
flux observer is an open loop, which amplifies rounding differences. PLECS is not
needed.

Run from the repository root (requires gcc):

    pytest tests/test_im.py

"""

# %%
import ctypes
import subprocess
import tempfile
from collections.abc import Sequence
from math import inf, pi
from pathlib import Path
from types import SimpleNamespace
from typing import Any, cast

import motulator.drive.control.im as control
import numpy as np
import pytest
from motulator.common.control._base import ControlSystem, TimeSeries
from motulator.common.utils import complex2abc
from motulator.drive import model, utils

from motulator_plecs._common import C_SOURCES

SRC_DIR = C_SOURCES

CAPI = r"""
#include "common.c"
#include "im_current_vector.c"

static IMVectorControlSystem ctrl;

void init(const double *p, const double *c, const double *s)
{
    InductionMachineInvGammaPars par = {p[0], p[1], p[2], p[3], p[4]};
    IMCurrentVectorControllerCfg cfg = im_current_vector_controller_cfg(c[0], c[1]);
    cfg.alpha_c = c[2];
    cfg.alpha_i = c[3];
    cfg.alpha_o = c[4];
    cfg.w_s_nom = c[5];
    cfg.k_u = c[6];
    cfg.k_fw = c[7];
    cfg.J = c[8];
    cfg.sensorless = (int)c[9];
    cfg.T_s = c[10];
    PIController speed_ctrl = speed_controller(s[0], s[1], s[2], s[3]);
    im_vector_control_system_init(&ctrl, par, &cfg, speed_ctrl);
}

void step(const double *i_s_abc, double u_dc, double w_M, double w_M_ref, double *out)
{
    IMMeasurements meas = {abc2complex(i_s_abc), u_dc, w_M};
    im_vector_control_system_compute_output(&ctrl, &meas, w_M_ref);
    /* Current reference in estimated rotor flux coordinates, as in the
     * post-processed results of motulator */
    double complex i_s_ref = cexp(-I * carg(ctrl.fbk.psi_R)) * ctrl.ref.i_s;
    double o[10] = {ctrl.ref.d_abc[0], ctrl.ref.d_abc[1], ctrl.ref.d_abc[2],
                    ctrl.fbk.w_M,      ctrl.fbk.tau_M,    ctrl.ref.tau_M,
                    cabs(ctrl.fbk.psi_R), ctrl.fbk.theta_c, creal(i_s_ref),
                    cimag(i_s_ref)};
    for (int k = 0; k < 10; k++) {
        out[k] = o[k];
    }
    im_vector_control_system_update(&ctrl);
}
"""

NAMES = ["d_a", "d_b", "d_c", "w_M", "tau_M", "tau_M_ref", "psi_R", "theta_c"]
NAMES += ["i_sd_ref", "i_sq_ref"]
NOM = utils.NominalValues(U=400, I=5, f=50, P=2.2e3, tau=14.6)
BASE = utils.BaseValues.from_nominal(NOM, n_p=2)
PAR = control.InductionMachineInvGammaPars(
    n_p=2, R_s=3.7, R_R=2.1, L_sgm=0.021, L_M=0.224
)
SPEED_CTRL = {"J": 0.015, "alpha_s": 2 * pi * 4}


def compile_library() -> ctypes.CDLL:
    """Compile the C sources into a shared library."""
    tmp = Path(tempfile.mkdtemp())
    (tmp / "capi.c").write_text(CAPI)
    lib = tmp / "libim.so"
    cmd = ["gcc", "-std=c99", "-O2", "-shared", "-fPIC", "-Wall"]
    cmd += ["-Wno-unused-function", f"-I{SRC_DIR}", str(tmp / "capi.c")]
    subprocess.run([*cmd, "-lm", "-o", str(lib)], check=True)
    return ctypes.CDLL(str(lib))


def _arr(values: Any) -> Any:
    values = np.asarray(values, dtype=float).ravel()
    return (ctypes.c_double * len(values))(*values)


def _d(x: float) -> ctypes.c_double:
    return ctypes.c_double(x)


def build_system(sensorless: bool) -> tuple[model.Drive, control.VectorControlSystem]:
    """Build the drive system of the 2.2-kW example."""
    mdl = model.Drive(
        model.InductionMachine(PAR),
        model.MechanicalSystem(J=0.015),
        model.VoltageSourceConverter(u_dc=540),
    )
    mdl.mechanics.set_external_load_torque(lambda t: (t > 0.3) * NOM.tau)
    cfg = control.CurrentVectorControllerCfg(
        psi_s_nom=BASE.psi, i_s_max=1.5 * BASE.i, J=0.015, sensorless=sensorless
    )
    ctrl = control.VectorControlSystem(
        control.CurrentVectorController(PAR, cfg), control.SpeedController(**SPEED_CTRL)
    )
    ctrl.set_speed_ref(lambda t: (t > 0.1) * 0.5 * BASE.w_M)
    return mdl, ctrl


class CControlSystem(control.VectorControlSystem):
    """Control system running the C port, for closed-loop simulations."""

    def __init__(self, ctrl: control.VectorControlSystem) -> None:
        super().__init__(ctrl.vector_ctrl, ctrl.speed_ctrl)  # For the measurements
        self.ext_ref = ctrl.ext_ref
        cfg = cast(control.CurrentVectorController, ctrl.vector_ctrl).cfg
        self.T_s = cfg.T_s
        self.dll = compile_library()
        c = [cfg.psi_s_nom, cfg.i_s_max, cfg.alpha_c, np.nan, cast(float, cfg.alpha_o)]
        c += [cfg.w_s_nom, cfg.k_u, cfg.k_fw, cast(float, cfg.J), cfg.sensorless]
        c += [cfg.T_s]
        p = [PAR.n_p, PAR.R_s, PAR.R_R, PAR.L_sgm, PAR.L_M]
        self.dll.init(_arr(p), _arr(c), _arr([*SPEED_CTRL.values(), np.nan, inf]))
        self.out = (ctypes.c_double * len(NAMES))()

    def run_control_loop(self, mdl: model.Drive) -> tuple[float, Sequence[float]]:
        meas = self.get_measurement(mdl)
        w_M_ref = cast(Any, self.ext_ref.w_M)(self.t)
        i_abc = complex2abc(meas.i_c_ab)
        w_M = meas.w_M or 0.0  # Measured only in the sensored mode
        self.dll.step(_arr(i_abc), _d(meas.u_dc), _d(w_M), _d(w_M_ref), self.out)
        out = np.array(self.out)
        self.save(self.t, out=SimpleNamespace(**dict(zip(NAMES, out, strict=True))))
        self.t = (self.t + self.T_s) % 1e9
        return self.T_s, list(out[:3])

    def post_process(self) -> TimeSeries:
        return ControlSystem.post_process(self)  # Only the saved C outputs


@pytest.mark.parametrize("sensorless", [True, False])
def test_control_system(sensorless: bool) -> None:
    """Closed-loop simulation of the 2.2-kW drive with motulator and with the C port."""
    mdl, ctrl = build_system(sensorless)
    res_py = model.Simulation(mdl, ctrl).simulate(t_stop=0.5)
    fbk, ref = res_py.ctrl.fbk, res_py.ctrl.ref
    py = np.column_stack(
        [
            *cast(Any, ref.d_abc).T,
            fbk.w_M,
            fbk.tau_M,
            ref.tau_M,
            np.abs(fbk.psi_R),
            fbk.theta_c,
            ref.i_s.real,
            ref.i_s.imag,
        ]
    )
    mdl, ctrl = build_system(sensorless)
    res_c = model.Simulation(mdl, CControlSystem(ctrl)).simulate(t_stop=0.5)
    c = np.column_stack([getattr(res_c.ctrl.out, name) for name in NAMES])
    assert np.allclose(res_py.ctrl.t, res_c.ctrl.t)
    err = np.max(np.abs(c - py), axis=0)
    scale = np.max(np.abs(py), axis=0)
    for name, e, s in zip(NAMES, err, scale, strict=True):
        print(f"  {name}: max error {e:.3g} (max value {s:.3g})")
    assert np.all(err <= 1e-8 * np.maximum(scale, 1.0))


# %%
if __name__ == "__main__":
    for sensorless in (True, False):
        print(f"test_control_system (sensorless={sensorless}):")
        test_control_system(sensorless)
        print("  passed")
