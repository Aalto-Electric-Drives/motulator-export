"""
Test the C port of the PWM with the dead-time error model against motulator.

The PWM of motulator with `d_err = dead_time_error(...)` and the C port are fed with
the same sequence of voltage references and measured currents, and the duty ratios
and the realized voltages are compared. The identification of the dead time from
`d_err` in the writers is also tested.

Run from the repository root (requires gcc):

    pytest tests/test_pwm.py

"""

# %%
import ctypes
from math import pi

import numpy as np
import pytest
from motulator.common.control import PWM
from motulator.common.utils import dead_time_error

from motulator_export.plecs._drive import pwm_values
from tests.c_port import arr, compile_library, d

CAPI = r"""
#include "common.c"

static PWM pwm;

void init(double t_d, double T_s, double feedforward)
{
    pwm_init(&pwm, 1.5);
    pwm_set_dead_time(&pwm, t_d, T_s, (int)feedforward);
}

void error(const double *i_abc, const double *d_abc, double t_d, double T_s,
           double *out)
{
    dead_time_error(i_abc, d_abc, t_d, T_s, out);
}

void step(double T_s, const double *u_ref, const double *i, double u_dc, double w,
          double *out)
{
    double complex i_c_ab = i[0] + I * i[1];
    double complex u = pwm_realized_voltage(&pwm, i_c_ab, u_dc);
    double d_abc[3];
    double complex u_c_ab = pwm_compute_output(&pwm, T_s, u_ref[0] + I * u_ref[1],
                                               u_dc, w, i_c_ab, d_abc);
    pwm_update(&pwm, u_c_ab, d_abc);
    double o[5] = {creal(u), cimag(u), d_abc[0], d_abc[1], d_abc[2]};
    for (int k = 0; k < 5; k++) {
        out[k] = o[k];
    }
}
"""

T_S, T_D, U_DC = 125e-6, 2e-6, 540.0


@pytest.fixture(scope="module")
def dll(tmp_path_factory: pytest.TempPathFactory) -> ctypes.CDLL:
    return compile_library(CAPI, tmp_path_factory.mktemp("pwm"))


def test_dead_time_error(dll: ctypes.CDLL) -> None:
    """The duty-ratio error equals that of motulator, also near the limits."""
    rng = np.random.default_rng(1)
    d_limits = [[0, 1, 0.5], [1e-3, 1 - 1e-3, 0.01]]
    d_test = np.vstack((d_limits, rng.uniform(0, 1, (50, 3))))
    i_test = np.vstack(([[1, -1, 0], [-2, 2, 0]], rng.uniform(-5, 5, (50, 3))))
    out = (ctypes.c_double * 3)()
    for i_abc, d_abc in zip(i_test, d_test, strict=True):
        dll.error(arr(i_abc), arr(d_abc), d(T_D), d(T_S), out)
        assert np.array(out) == pytest.approx(
            dead_time_error(i_abc, d_abc, T_D, T_S), abs=1e-15
        )


@pytest.mark.parametrize("t_d", [0.0, T_D], ids=["no_d_err", "d_err"])
@pytest.mark.parametrize("feedforward", [True, False], ids=["ff", "no_ff"])
def test_pwm(dll: ctypes.CDLL, t_d: float, feedforward: bool) -> None:
    """The duty ratios and the realized voltages equal those of motulator, also in
    overmodulation, where the error depends on the duty ratios."""
    d_err = None if t_d == 0 else lambda i, d: dead_time_error(i, d, t_d, T_S)
    pwm = PWM(d_err=d_err, feedforward=feedforward)
    dll.init(d(t_d), d(T_S), d(feedforward))
    out = (ctypes.c_double * 5)()
    rng = np.random.default_rng(2)
    for k in range(200):
        # Rotating voltage reference, reaching the overmodulation range
        theta = 2 * pi * 50 * k * T_S
        u_ref = (100 + 2 * k) * np.exp(1j * theta)
        i_c_ab = 5 * np.exp(1j * (theta - 0.5)) + rng.normal(0, 0.1)
        w = 2 * pi * 50
        u_py = pwm.get_realized_voltage(i_c_ab, U_DC)
        d_py = pwm(T_S, u_ref, U_DC, w)
        dll.step(
            d(T_S), arr([u_ref.real, u_ref.imag]), arr([i_c_ab.real, i_c_ab.imag]),
            d(U_DC), d(w), out,
        )
        assert out[0] + 1j * out[1] == pytest.approx(u_py, abs=1e-9)
        assert np.array(out[2:]) == pytest.approx(d_py, abs=1e-12)


def test_pwm_values() -> None:
    """The dead time is identified from d_err, and other functions are rejected."""
    pwm = PWM(d_err=lambda i, d: dead_time_error(i, d, T_D, T_S), feedforward=False)
    assert pwm_values(pwm, T_S) == {"pwm_t_d": T_D, "pwm_feedforward": 0}
    assert pwm_values(PWM(), T_S) == {"pwm_t_d": 0.0, "pwm_feedforward": 1}
    smooth = PWM(
        d_err=lambda i, d: dead_time_error(
            i, d, T_D, T_S, sign=lambda x: 2 / pi * np.arctan(x / 0.1)
        )
    )
    with pytest.raises(NotImplementedError):
        pwm_values(smooth, T_S)
