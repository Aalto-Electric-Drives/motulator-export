"""
Test the C port of grid-following control against motulator.

The control system of the example examples/grid/grid_following/plot_10kva_lcl_gfl.py
is driven with the same sequence of measurements in motulator and in the C port, and
the outputs are compared step by step. PLECS is not needed.

Run from the repository root (requires gcc):

    pytest tests/test_gfl.py

"""

# %%
import ctypes
from math import pi
from typing import Any, cast

import numpy as np
import pytest
from motulator.common.utils import abc2complex, complex2abc
from motulator.grid import control, utils
from motulator.grid.control._base import Measurements

from tests.c_port import arr, compile_library, d

CAPI = r"""
#include "common.c"
#include "gfl_current_vector.c"

static GFLControlSystem ctrl;

void init(const double *c)
{
    GFLControllerCfg cfg = gfl_controller_cfg(c[0], c[1]);
    cfg.alpha_c = c[2];
    cfg.alpha_i = c[3];
    cfg.u_nom = c[4];
    cfg.w_nom = c[5];
    cfg.alpha_pll = c[6];
    cfg.T_s = c[7];
    gfl_control_system_init(&ctrl, &cfg);
}

void step(const double *i_c_abc, const double *u_g_line, double u_dc, double p_g_ref,
          double q_g_ref, double *out)
{
    GridMeasurements meas = {abc2complex(i_c_abc), line2complex(u_g_line), u_dc};
    gfl_control_system_compute_output(&ctrl, &meas, p_g_ref, q_g_ref);
    double o[8] = {ctrl.ref.d_abc[0], ctrl.ref.d_abc[1], ctrl.ref.d_abc[2],
                   ctrl.fbk.theta_c, ctrl.fbk.w_g, ctrl.fbk.u_g, creal(ctrl.ref.u_c),
                   cimag(ctrl.ref.u_c)};
    for (int k = 0; k < 8; k++) {
        out[k] = o[k];
    }
    gfl_control_system_update(&ctrl);
}
"""


@pytest.fixture(scope="module")
def dll(tmp_path_factory: pytest.TempPathFactory) -> ctypes.CDLL:
    return compile_library(CAPI, tmp_path_factory.mktemp("gfl"))


def test_control_system(dll: ctypes.CDLL) -> None:
    """Grid-following control of the example for a sequence of measurements."""
    nom = utils.NominalValues(U=400, I=14.5, f=50, P=10e3)
    base = utils.BaseValues.from_nominal(nom)
    cfg = control.CurrentVectorControllerCfg(
        i_max=1.5 * base.i, L=0.073 * base.L, T_s=100e-6
    )
    ctrl = control.GridConverterControlSystem(control.CurrentVectorController(cfg))
    ctrl.set_power_ref(lambda t: (t > 0.02) * 5e3)
    ctrl.set_reactive_power_ref(lambda t: (t > 0.04) * 4e3)
    c = [cfg.i_max, cfg.L, cfg.alpha_c, cast(float, cfg.alpha_i), cfg.u_nom]
    c += [cfg.w_nom, cfg.alpha_pll, cfg.T_s]
    dll.init(arr(c))

    # Measurements: grid voltage with a frequency offset and a phase jump, and a
    # current with harmonics
    n = 1000
    t = np.arange(n) * cfg.T_s
    theta_g = 2 * pi * 51 * t + 0.3 * (t > 0.05)
    u_g_ab = base.u * np.exp(1j * theta_g)
    i_c_ab = (5 + 3j + 2 * np.exp(1j * 2 * pi * 250 * t)) * np.exp(1j * theta_g)
    u_dc = 650 + 20 * np.sin(2 * pi * 100 * t)

    out = (ctypes.c_double * 8)()
    res_c, res_py = np.zeros((n, 8)), np.zeros((n, 8))
    for k in range(n):
        i_abc = complex2abc(i_c_ab[k])
        u_abc = complex2abc(u_g_ab[k])
        u_line = [u_abc[0] - u_abc[1], u_abc[1] - u_abc[2]]  # Line-to-line voltages
        meas = Measurements(abc2complex(i_abc), abc2complex(u_abc), u_dc[k])
        fbk = cast(Any, ctrl.get_feedback(meas))
        ref = cast(Any, ctrl.compute_output(fbk))
        ctrl.update(ref, fbk)
        res_py[k] = [
            *ref.d_abc,
            fbk.theta_c,
            fbk.w_g,
            fbk.u_g,
            ref.u_c.real,
            ref.u_c.imag,
        ]
        dll.step(arr(i_abc), arr(u_line), d(u_dc[k]), d(ref.p_g), d(ref.q_g), out)
        res_c[k] = np.array(out)
    names = ["d_a", "d_b", "d_c", "theta_c", "w_g", "u_g", "u_c_d", "u_c_q"]
    err = np.max(np.abs(res_c - res_py), axis=0)
    scale = np.max(np.abs(res_py), axis=0)
    for name, e, s in zip(names, err, scale, strict=True):
        print(f"  {name}: max error {e:.3g} (max value {s:.3g})")
    assert np.all(err <= 1e-9 * np.maximum(scale, 1.0))
