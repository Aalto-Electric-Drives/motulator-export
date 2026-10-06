"""
Test the C port of the PWM with the dead-time error model against motulator.

The PWM of motulator with `d_err = dead_time_error(...)` and the C port are fed with
the same sequence of voltage references and measured currents, and the duty ratios
and the realized voltages are compared, with the current-direction functions
`np.sign` and `tanh(i/i_0)`. The identification of the dead time and the current
scale from `d_err` in the writers is also tested.

Run from the repository root (requires gcc):

    pytest tests/test_pwm.py

"""

# %%
import ctypes
from math import pi
from typing import Callable

import numpy as np
import pytest
from motulator.common.control import PWM
from motulator.common.utils import dead_time_error

from motulator_export.plecs._drive import pwm_values
from tests.c_port import arr, compile_library, d

CAPI = r"""
#include "common.c"

static PWM pwm;

void init(double t_d, double T_s, double i_0, double feedforward, double d_min)
{
    pwm_init(&pwm, 1.5);
    pwm_set_dead_time(&pwm, t_d, T_s, i_0, (int)feedforward);
    pwm_set_min_pulse(&pwm, d_min);
}

void error(const double *i_abc, const double *d_abc, double t_d, double T_s,
           double i_0, double *out)
{
    dead_time_error(i_abc, d_abc, t_d, T_s, i_0, out);
}

void step(double T_s, const double *u_ref, const double *i, double u_dc, double w,
          double *out)
{
    double complex i_c_ab = i[0] + I * i[1];
    double complex u = pwm_realized_voltage(&pwm, i_c_ab, u_dc, 1);
    double complex u_zoh = pwm_realized_voltage(&pwm, i_c_ab, u_dc, 0);
    double d_abc[3];
    pwm_compute_output(&pwm, T_s, u_ref[0] + I * u_ref[1], u_dc, w, i_c_ab, d_abc);
    pwm_update(&pwm, d_abc);
    double o[7] = {creal(u),  cimag(u),  creal(u_zoh), cimag(u_zoh),
                   d_abc[0], d_abc[1], d_abc[2]};
    for (int k = 0; k < 7; k++) {
        out[k] = o[k];
    }
}
"""

T_S, T_D, U_DC, I_0 = 125e-6, 2e-6, 540.0, 0.5
D_MIN = 0.04  # Minimum duty ratio of a switching leg


def tanh_sign(i_0: float) -> Callable[[np.ndarray], np.ndarray]:
    """Current-direction function `tanh(i/i_0)`, or `np.sign` if `i_0` is zero."""
    return np.sign if i_0 == 0 else lambda i: np.tanh(i / i_0)


@pytest.fixture(scope="module")
def dll(tmp_path_factory: pytest.TempPathFactory) -> ctypes.CDLL:
    return compile_library(CAPI, tmp_path_factory.mktemp("pwm"))


@pytest.mark.parametrize("i_0", [0.0, I_0], ids=["sign", "tanh"])
def test_dead_time_error(dll: ctypes.CDLL, i_0: float) -> None:
    """The duty-ratio error equals that of motulator, also near the limits and near
    zero current."""
    rng = np.random.default_rng(1)
    d_limits = [[0, 1, 0.5], [1e-3, 1 - 1e-3, 0.01], [0.5, 0.5, 0.5]]
    d_test = np.vstack((d_limits, rng.uniform(0, 1, (50, 3))))
    i_near = [[1, -1, 0], [-2, 2, 0], [0.01, -0.2, 0.6]]
    i_test = np.vstack((i_near, rng.uniform(-5, 5, (50, 3))))
    out = (ctypes.c_double * 3)()
    for i_abc, d_abc in zip(i_test, d_test, strict=True):
        dll.error(arr(i_abc), arr(d_abc), d(T_D), d(T_S), d(i_0), out)
        assert np.array(out) == pytest.approx(
            dead_time_error(i_abc, d_abc, T_D, T_S, tanh_sign(i_0)), abs=1e-15
        )


@pytest.mark.parametrize(
    ("t_d", "i_0", "d_min"),
    [
        (0.0, 0.0, 0.0),
        (T_D, 0.0, 0.0),
        (T_D, I_0, 0.0),
        (0.0, 0.0, D_MIN),
        (T_D, I_0, D_MIN),
    ],
    ids=["no_d_err", "d_err", "d_err_tanh", "min_pulse", "d_err_tanh_min_pulse"],
)
@pytest.mark.parametrize("feedforward", [True, False], ids=["ff", "no_ff"])
def test_pwm(
    dll: ctypes.CDLL, t_d: float, i_0: float, d_min: float, feedforward: bool
) -> None:
    """The duty ratios and the realized voltages (the average of two sampling periods
    and that of the ongoing period) equal those of motulator, also in overmodulation,
    where the error depends on the duty ratios, and with a varying DC-bus voltage."""
    sign = tanh_sign(i_0)
    d_err = None if t_d == 0 else lambda i, d: dead_time_error(i, d, t_d, T_S, sign)
    pwm = PWM(d_err=d_err, feedforward=feedforward, d_min=d_min)
    dll.init(d(t_d), d(T_S), d(i_0), d(feedforward), d(d_min))
    out = (ctypes.c_double * 7)()
    rng = np.random.default_rng(2)
    for k in range(200):
        # Rotating voltage reference, reaching the overmodulation range
        theta = 2 * pi * 50 * k * T_S
        u_ref = (100 + 2 * k) * np.exp(1j * theta)
        i_c_ab = 5 * np.exp(1j * (theta - 0.5)) + rng.normal(0, 0.1)
        w = 2 * pi * 50
        u_dc = U_DC * (1 + 0.05 * np.sin(2 * pi * 300 * k * T_S))
        u_py = pwm.get_realized_voltage(i_c_ab, u_dc)
        u_zoh_py = pwm.get_realized_voltage(i_c_ab, u_dc, average=False)
        d_py = pwm(T_S, u_ref, u_dc, w)
        dll.step(
            d(T_S),
            arr([u_ref.real, u_ref.imag]),
            arr([i_c_ab.real, i_c_ab.imag]),
            d(u_dc),
            d(w),
            out,
        )
        assert out[0] + 1j * out[1] == pytest.approx(u_py, abs=1e-9)
        assert out[2] + 1j * out[3] == pytest.approx(u_zoh_py, abs=1e-9)
        assert np.array(out[4:]) == pytest.approx(d_py, abs=1e-12)


def test_pwm_values() -> None:
    """The dead time and the current scale of tanh are identified from d_err, and
    other functions are rejected, also if they differ from np.sign only near zero
    current."""
    pwm = PWM(d_err=lambda i, d: dead_time_error(i, d, T_D, T_S), feedforward=False)
    assert pwm_values(pwm, T_S) == {
        "pwm_t_d": T_D,
        "pwm_i_0": 0.0,
        "pwm_feedforward": 0,
        "pwm_d_min": 0.0,
    }
    assert pwm_values(PWM(), T_S) == {
        "pwm_t_d": 0.0,
        "pwm_i_0": 0.0,
        "pwm_feedforward": 1,
        "pwm_d_min": 0.0,
    }
    for i_0 in [1e-6, 0.01, I_0, 20.0]:
        sign = tanh_sign(i_0)
        pwm = PWM(d_err=lambda i, d, sign=sign: dead_time_error(i, d, T_D, T_S, sign))
        assert pwm_values(pwm, T_S) == {
            "pwm_t_d": T_D,
            "pwm_i_0": i_0,
            "pwm_feedforward": 1,
            "pwm_d_min": 0.0,
        }
    others: list[Callable[[np.ndarray], np.ndarray]] = [
        lambda x: 2 / pi * np.arctan(x / 0.1),
        lambda x: 2 / pi * np.arctan(x / 1e-4),  # Close to np.sign beyond 0.1 A
        lambda x: np.clip(x / 0.01, -1, 1),
        lambda x: np.tanh(x / 0.01) ** 3,
    ]
    for sign in others:
        pwm = PWM(d_err=lambda i, d, sign=sign: dead_time_error(i, d, T_D, T_S, sign))
        with pytest.raises(NotImplementedError):
            pwm_values(pwm, T_S)


def test_pwm_values_min_pulse() -> None:
    """The minimum duty ratio is a parameter of the PWM."""
    pwm = PWM(d_min=D_MIN)
    assert pwm_values(pwm, T_S)["pwm_d_min"] == D_MIN
