"""
Test the C port of the PWM with the duty-ratio error model against motulator.

The PWM of motulator with `d_err = InverterNonlinearity(...)` and the C port are fed
with the same sequence of voltage references and measured currents, and the duty
ratios, the limited voltages, and the realized voltages are compared, with the dead
time and the threshold voltage, the current-direction functions `np.sign` and
`tanh(i/i_0)`, the minimum pulses, and the double and single update. The
identification of the current scale from `d_err` in the writers is also tested.

Run from the repository root (requires gcc):

    pytest tests/test_pwm.py

"""

# %%
import ctypes
from math import inf, pi
from typing import Callable

import numpy as np
import pytest
from motulator.common.control import PWM, InverterNonlinearity

from motulator_export.plecs._control import pwm_values
from tests.c_port import arr, compile_library, d

CAPI = r"""
#include "common.c"

static PWM pwm;

void init(double t_d, double u_th, double i_0, double feedforward, double d_min,
          double single_update)
{
    pwm_init(&pwm, 1.5);
    pwm_set_nonlinearity(&pwm, t_d, u_th, i_0, (int)feedforward);
    pwm_set_min_pulse(&pwm, d_min);
    pwm_set_single_update(&pwm, (int)single_update);
}

void error(const double *i_abc, const double *d_abc, double t_d, double u_th,
           double i_0, double u_dc, double T_half, double *out)
{
    inverter_nonlinearity(t_d, u_th, i_0, i_abc, d_abc, u_dc, T_half, out);
}

void step(double T_s, const double *u_ref, const double *i, double u_dc, double w,
          double *out)
{
    double complex i_c_ab = i[0] + I * i[1];
    double complex u = pwm_realized_voltage(&pwm, i_c_ab, u_dc, 1);
    double complex u_zoh = pwm_realized_voltage(&pwm, i_c_ab, u_dc, 0);
    double d_abc[3];
    double complex u_lim_ab;
    double complex u_lim = pwm_compute_output(&pwm, T_s, u_ref[0] + I * u_ref[1], u_dc,
                                              w, i_c_ab, d_abc, &u_lim_ab);
    pwm_update(&pwm, T_s, d_abc);
    double o[11] = {creal(u),     cimag(u),     creal(u_zoh), cimag(u_zoh),
                    d_abc[0],     d_abc[1],     d_abc[2],     creal(u_lim),
                    cimag(u_lim), creal(u_lim_ab), cimag(u_lim_ab)};
    for (int k = 0; k < 11; k++) {
        out[k] = o[k];
    }
}
"""

T_S, T_D, U_TH, U_DC, I_0 = 125e-6, 2e-6, 1.0, 540.0, 0.5
D_MIN = 0.04  # Minimum duty ratio of a switching leg


def tanh_sign(i_0: float) -> Callable[[np.ndarray], np.ndarray]:
    """Current-direction function `tanh(i/i_0)`, or `np.sign` if `i_0` is zero."""
    return np.sign if i_0 == 0 else lambda i: np.tanh(i / i_0)


@pytest.fixture(scope="module")
def dll(tmp_path_factory: pytest.TempPathFactory) -> ctypes.CDLL:
    return compile_library(CAPI, tmp_path_factory.mktemp("pwm"))


@pytest.mark.parametrize("i_0", [0.0, I_0], ids=["sign", "tanh"])
@pytest.mark.parametrize(
    ("t_d", "u_th"), [(T_D, 0.0), (0.0, U_TH), (T_D, U_TH)], ids=["t_d", "u_th", "both"]
)
@pytest.mark.parametrize("T_half", [T_S, T_S / 2, inf], ids=["T_s", "T_s/2", "inf"])
def test_inverter_nonlinearity(
    dll: ctypes.CDLL, i_0: float, t_d: float, u_th: float, T_half: float
) -> None:
    """The duty-ratio error equals that of motulator, also near the limits, near zero
    current, and with the infinite half carrier period before the first duty
    ratios."""
    rng = np.random.default_rng(1)
    d_limits = [[0, 1, 0.5], [1e-3, 1 - 1e-3, 0.01], [0.5, 0.5, 0.5]]
    d_test = np.vstack((d_limits, rng.uniform(0, 1, (50, 3))))
    i_near = [[1, -1, 0], [-2, 2, 0], [0.01, -0.2, 0.6]]
    i_test = np.vstack((i_near, rng.uniform(-5, 5, (50, 3))))
    u_dc_test = U_DC * rng.uniform(0.5, 1.5, len(i_test))
    d_err = InverterNonlinearity(t_d, u_th, tanh_sign(i_0))
    out = (ctypes.c_double * 3)()
    for i_abc, d_abc, u_dc in zip(i_test, d_test, u_dc_test, strict=True):
        dll.error(
            arr(i_abc), arr(d_abc), d(t_d), d(u_th), d(i_0), d(u_dc), d(T_half), out
        )
        assert np.array(out) == pytest.approx(
            d_err(i_abc, d_abc, u_dc, T_half), abs=1e-15
        )


@pytest.mark.parametrize(
    ("t_d", "u_th", "i_0", "d_min"),
    [
        (0.0, 0.0, 0.0, 0.0),
        (T_D, 0.0, 0.0, 0.0),
        (T_D, 0.0, I_0, 0.0),
        (0.0, U_TH, 0.0, 0.0),
        (T_D, U_TH, I_0, 0.0),
        (0.0, 0.0, 0.0, D_MIN),
        (T_D, U_TH, I_0, D_MIN),
    ],
    ids=[
        "no_d_err",
        "d_err",
        "d_err_tanh",
        "u_th",
        "d_err_u_th_tanh",
        "min_pulse",
        "d_err_u_th_tanh_min_pulse",
    ],
)
@pytest.mark.parametrize("feedforward", [True, False], ids=["ff", "no_ff"])
@pytest.mark.parametrize("single_update", [False, True], ids=["double", "single"])
def test_pwm(
    dll: ctypes.CDLL,
    t_d: float,
    u_th: float,
    i_0: float,
    d_min: float,
    feedforward: bool,
    single_update: bool,
) -> None:
    """The duty ratios, the limited voltages (including and excluding the
    compensation), and the realized voltages (the average of two sampling periods and
    that of the ongoing period) equal those of motulator, also in overmodulation,
    where the error depends on the duty ratios, and with a varying DC-bus voltage."""
    d_err = None
    if t_d > 0 or u_th > 0:
        d_err = InverterNonlinearity(t_d, u_th, tanh_sign(i_0))
    pwm = PWM(
        d_err=d_err, feedforward=feedforward, d_min=d_min, single_update=single_update
    )
    dll.init(d(t_d), d(u_th), d(i_0), d(feedforward), d(d_min), d(single_update))
    out = (ctypes.c_double * 11)()
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
        d_py, u_lim_py = pwm.compute_output(T_S, u_ref, u_dc, w)
        pwm.update(T_S, d_py)
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
        assert np.array(out[4:7]) == pytest.approx(d_py, abs=1e-12)
        assert out[7] + 1j * out[8] == pytest.approx(u_lim_py, abs=1e-9)
        assert out[9] + 1j * out[10] == pytest.approx(pwm.limited_voltage, abs=1e-9)


@pytest.mark.parametrize("u_dc", [0.0, -1.0], ids=["zero", "negative"])
def test_pwm_discharged(dll: ctypes.CDLL, u_dc: float) -> None:
    """With a discharged DC bus, the duty ratios are 0.5 and the limited and the
    realized voltages zero, as in motulator, also with the duty-ratio error model."""
    pwm = PWM(d_err=InverterNonlinearity(T_D, U_TH))
    dll.init(d(T_D), d(U_TH), d(0.0), d(1.0), d(0.0), d(0.0))
    out = (ctypes.c_double * 11)()
    u_py = pwm.get_realized_voltage(5.0 + 0j, u_dc)
    u_zoh_py = pwm.get_realized_voltage(5.0 + 0j, u_dc, average=False)
    d_py, u_lim_py = pwm.compute_output(T_S, 100.0 + 0j, u_dc, 2 * pi * 50)
    dll.step(d(T_S), arr([100.0, 0.0]), arr([5.0, 0.0]), d(u_dc), d(2 * pi * 50), out)
    assert d_py == [0.5, 0.5, 0.5]
    py = [u_py.real, u_py.imag, u_zoh_py.real, u_zoh_py.imag, *d_py]
    assert list(out) == [*py, u_lim_py.real, u_lim_py.imag, 0.0, 0.0]
    assert np.all(np.array(out)[[0, 1, 2, 3, 7, 8]] == 0)
    assert pwm.limited_voltage == 0


def test_pwm_values() -> None:
    """The dead time and the threshold voltage are read from InverterNonlinearity
    and the current scale of tanh is identified from its current-direction function,
    and other functions are rejected, also if they differ from np.sign only near
    zero current."""
    pwm = PWM(d_err=InverterNonlinearity(T_D), feedforward=False)
    assert pwm_values(pwm) == {
        "pwm_t_d": T_D,
        "pwm_u_th": 0.0,
        "pwm_i_0": 0.0,
        "pwm_feedforward": 0,
        "pwm_d_min": 0.0,
        "pwm_single_update": 0,
        "pwm_overmodulation": "MPE",
    }
    assert pwm_values(PWM()) == {
        "pwm_t_d": 0.0,
        "pwm_u_th": 0.0,
        "pwm_i_0": 0.0,
        "pwm_feedforward": 1,
        "pwm_d_min": 0.0,
        "pwm_single_update": 0,
        "pwm_overmodulation": "MPE",
    }
    for i_0 in [1e-6, 0.01, I_0, 20.0]:
        pwm = PWM(d_err=InverterNonlinearity(T_D, U_TH, tanh_sign(i_0)))
        assert pwm_values(pwm) == {
            "pwm_t_d": T_D,
            "pwm_u_th": U_TH,
            "pwm_i_0": i_0,
            "pwm_feedforward": 1,
            "pwm_d_min": 0.0,
            "pwm_single_update": 0,
            "pwm_overmodulation": "MPE",
        }
    pwm = PWM(d_err=InverterNonlinearity(0.0, U_TH, _signum))
    assert pwm_values(pwm)["pwm_u_th"] == U_TH
    assert pwm_values(pwm)["pwm_i_0"] == 0.0
    others: list[Callable[[np.ndarray], np.ndarray]] = [
        lambda x: 2 / pi * np.arctan(x / 0.1),
        lambda x: 2 / pi * np.arctan(x / 1e-4),  # Close to np.sign beyond 0.1 A
        lambda x: np.clip(x / 0.01, -1, 1),
        lambda x: np.tanh(x / 0.01) ** 3,
    ]
    for sign in others:
        with pytest.raises(NotImplementedError, match="sign=np.sign"):
            pwm_values(PWM(d_err=InverterNonlinearity(T_D, sign=sign)))


def _signum(i: np.ndarray) -> np.ndarray:
    """Signum function, other than np.sign itself."""
    return np.sign(i)


class _Nonlinearity(InverterNonlinearity):
    """Error model with an additional error."""

    def __call__(
        self, i_abc: np.ndarray, d_abc: np.ndarray, u_dc: float, T_half: float
    ) -> np.ndarray:
        return super().__call__(i_abc, d_abc, u_dc, T_half) + 1e-3


@pytest.mark.parametrize(
    "d_err",
    [
        InverterNonlinearity(T_D).__call__,  # A function, not InverterNonlinearity
        _Nonlinearity(T_D),
        InverterNonlinearity(-T_D),
        InverterNonlinearity(T_D, -U_TH),
    ],
    ids=["function", "subclass", "negative_t_d", "negative_u_th"],
)
def test_pwm_values_unsupported(d_err: Callable) -> None:
    """Only InverterNonlinearity with nonnegative parameters is supported."""
    with pytest.raises(NotImplementedError):
        pwm_values(PWM(d_err=d_err))


def test_pwm_values_min_pulse_single_update() -> None:
    """The minimum duty ratio and the single update are parameters of the PWM."""
    values = pwm_values(PWM(d_min=D_MIN, single_update=True))
    assert values["pwm_d_min"] == D_MIN
    assert values["pwm_single_update"] == 1
