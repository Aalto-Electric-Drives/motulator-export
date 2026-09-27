"""
Test the C port of disturbance-observer-based grid-forming control against motulator.

The control system of the 12.5-kVA converter (see gfm_13kva_do.py, based on the
example examples/grid/grid_forming/plot_13kva_do_gfm.py) is driven with the same
sequence of measurements in motulator and in the C port, with and without the
active-power reference limitation, and the outputs are compared step by step. PLECS
is not needed.

Run from the repository root (requires gcc):

    pytest tests/test_gfm.py

"""

# %%
import ctypes
import subprocess
import tempfile
from math import pi
from pathlib import Path
from typing import Any, cast

import numpy as np
import pytest
from motulator.common.utils import abc2complex, complex2abc
from motulator.grid import control, utils
from motulator.grid.control._base import Measurements

from motulator_plecs._common import C_SOURCES

SRC_DIR = C_SOURCES

CAPI = r"""
#include "common.c"
#include "gfm_observer.c"

static GFMControlSystem ctrl;

void init(const double *c)
{
    GFMControllerCfg cfg = gfm_controller_cfg(c[0], c[1]);
    cfg.R = c[2];
    cfg.R_a = c[3];
    cfg.k_v = c[4];
    cfg.alpha_o = c[5];
    cfg.alpha_c = c[6];
    cfg.u_nom = c[7];
    cfg.w_nom = c[8];
    cfg.T_s = c[9];
    cfg.i_d_max = c[10];
    cfg.alpha_l = c[11];
    gfm_control_system_init(&ctrl, &cfg);
}

void step(const double *i_c_abc, double u_dc, double p_g_ref, double v_c_ref,
          double *out)
{
    GFMMeasurements meas = {abc2complex(i_c_abc), u_dc};
    gfm_control_system_compute_output(&ctrl, &meas, p_g_ref, v_c_ref);
    double o[10] = {ctrl.ref.d_abc[0], ctrl.ref.d_abc[1], ctrl.ref.d_abc[2],
                    ctrl.fbk.theta_c,  ctrl.fbk.p_g,      ctrl.fbk.q_g,
                    cabs(ctrl.fbk.v_c), ctrl.ref.p_g,     creal(ctrl.ref.i_c),
                    cimag(ctrl.ref.i_c)};
    for (int k = 0; k < 10; k++) {
        out[k] = o[k];
    }
    gfm_control_system_update(&ctrl);
}
"""


def compile_library() -> ctypes.CDLL:
    """Compile the C sources into a shared library."""
    tmp = Path(tempfile.mkdtemp())
    (tmp / "capi.c").write_text(CAPI)
    lib = tmp / "libgfm.so"
    cmd = ["gcc", "-std=c99", "-O2", "-shared", "-fPIC", "-Wall"]
    cmd += ["-Wno-unused-function", f"-I{SRC_DIR}", str(tmp / "capi.c")]
    subprocess.run([*cmd, "-lm", "-o", str(lib)], check=True)
    return ctypes.CDLL(str(lib))


def _arr(values: Any) -> Any:
    values = np.asarray(values, dtype=float).ravel()
    return (ctypes.c_double * len(values))(*values)


def _d(x: float) -> ctypes.c_double:
    return ctypes.c_double(x)


@pytest.mark.parametrize("power_limitation", [False, True])
def test_control_system(power_limitation: bool) -> None:
    """Grid-forming control of the example for a sequence of measurements."""
    nom = utils.NominalValues(U=400, I=18, f=50, P=12.5e3)
    base = utils.BaseValues.from_nominal(nom)
    cfg = control.ObserverBasedGridFormingControllerCfg(
        i_max=1.3 * base.i,
        L=0.35 * base.L,
        R=0.05 * base.Z,
        R_a=0.2 * base.Z,
        u_nom=base.u,
        w_nom=base.w,
        i_d_max=0.85 * 1.3 * base.i if power_limitation else None,
    )
    ctrl = control.GridConverterControlSystem(
        control.ObserverBasedGridFormingController(cfg)
    )
    ctrl.set_ac_voltage_ref(base.u)

    def p_g_ref(t: float) -> float:
        return (t > 0.02) * nom.P - (t > 0.06) * 2 * nom.P

    ctrl.set_power_ref(p_g_ref)
    dll = compile_library()
    c = [cfg.i_max, cfg.L, cfg.R, cast(float, cfg.R_a), cast(float, cfg.k_v)]
    c += [cfg.alpha_o, cfg.alpha_c, cfg.u_nom, cfg.w_nom, cfg.T_s]
    c += [np.nan if cfg.i_d_max is None else cfg.i_d_max, cfg.alpha_l]
    dll.init(_arr(c))

    # Measurements: a current with a frequency offset and harmonics, and a ripple in
    # the DC-bus voltage. The PCC voltage is not used by the controller.
    n = 1000
    t = np.arange(n) * cfg.T_s
    theta = 2 * pi * 51 * t
    i_c_ab = (10 + 5j + 3 * np.exp(1j * 2 * pi * 250 * t)) * np.exp(1j * theta)
    u_dc = 650 + 20 * np.sin(2 * pi * 100 * t)

    out = (ctypes.c_double * 10)()
    res_c, res_py = np.zeros((n, 10)), np.zeros((n, 10))
    for k in range(n):
        i_abc = complex2abc(i_c_ab[k])
        p_ref = p_g_ref(ctrl.t)  # Unlimited reference at the current time
        meas = Measurements(abc2complex(i_abc), base.u, u_dc[k])
        fbk = cast(Any, ctrl.get_feedback(meas))
        ref = cast(Any, ctrl.compute_output(fbk))
        ctrl.update(ref, fbk)
        res_py[k] = [
            *ref.d_abc,
            fbk.theta_c,
            fbk.p_g,
            fbk.q_g,
            abs(fbk.v_c),
            ref.p_g,
            ref.i_c.real,
            ref.i_c.imag,
        ]
        dll.step(_arr(i_abc), _d(u_dc[k]), _d(p_ref), _d(ref.v_c), out)
        res_c[k] = np.array(out)
    names = ["d_a", "d_b", "d_c", "theta_c", "p_g", "q_g", "v_c", "p_g_ref"]
    names += ["i_c_d_ref", "i_c_q_ref"]
    err = np.max(np.abs(res_c - res_py), axis=0)
    scale = np.max(np.abs(res_py), axis=0)
    for name, e, s in zip(names, err, scale, strict=True):
        print(f"  {name}: max error {e:.3g} (max value {s:.3g})")
    assert np.all(err <= 1e-9 * np.maximum(scale, 1.0))


# %%
if __name__ == "__main__":
    for power_limitation in (False, True):
        print(f"test_control_system (power_limitation={power_limitation}):")
        test_control_system(power_limitation)
        print("  passed")
