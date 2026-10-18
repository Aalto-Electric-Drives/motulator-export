"""
Test the simulation core of the native simulation (c/sim.c) against scipy and
motulator: the RK45 solver with the integration intervals of motulator, and the
computational delay and the PWM models.

Run from the repository root (requires gcc):

    pytest tests/test_sim.py

"""

# %%
import ctypes
from typing import Any

import numpy as np
import pytest
from motulator.common.model import CarrierComparison
from motulator.drive import model
from scipy.integrate import solve_ivp

from tests.c_port import arr, compile_library, d

CAPI = r"""
#include "common.c"
#include "sim.c"

/* Model of a Van der Pol oscillator driving a complex first-order system, whose
 * output is the voltage of the converter */
static void vdp_rhs(const Model *self, double t, const double complex *x,
                    double complex *dx)
{
    (void)t;
    dx[0] = x[1];
    dx[1] = 5.0 * (1.0 - x[0] * x[0]) * x[1] - x[0];
    dx[2] = (-1.0 + 10.0 * I) * x[2] + x[0] + converter_voltage(&self->converter);
}

static double complex zero_current(const Model *self, const double complex *x)
{
    (void)self;
    (void)x;
    return 0.0;
}

static void keep_states(const Model *self, double complex *x)
{
    (void)self;
    (void)x;
}

/* Control system giving the sampling period T_s and the duty ratios 0.5 */
typedef struct {
    ControlSystem base;
    double T_s;
} Control;

static int control_run(ControlSystem *base, const Model *mdl, const double complex *x,
                       double *T_s, double d_abc[3])
{
    (void)mdl;
    (void)x;
    *T_s = ((Control *)base)->T_s;
    for (int k = 0; k < 3; k++) {
        d_abc[k] = 0.5;
    }
    return SIM_OK;
}

static Model mdl;
static Control ctrl;
static Simulation sim;

/* Integrate over the sampling period T_s, writing the solution points (the time and
 * the states as real and imaginary parts) and the final states into out. Returns
 * the number of points. */
int integrate(double T_s, const double *x0, double rtol, double atol, double max_step,
              int N_eval, double *out, int max_rows)
{
    mdl.n = 3;
    mdl.rhs = vdp_rhs;
    mdl.converter_current = zero_current;
    mdl.set_states = keep_states;
    mdl.converter.u_dc = 1.0;
    modulator_init(&mdl.modulator, 0, 0.0, 0.0, 0, 0);
    ctrl.base.run = control_run;
    ctrl.T_s = T_s;
    log_init(&ctrl.base.log, 1);
    double complex x[3];
    for (int i = 0; i < 3; i++) {
        x[i] = complex_from(x0[2 * i], x0[2 * i + 1]);
    }
    SolverCfg cfg = {max_step, rtol, atol};
    simulation_init(&sim, &mdl, &ctrl.base, &cfg, 0.0, x, 0.0);
    if (simulation_run(&sim, 0.0, N_eval) != SIM_OK) {
        return -1;
    }
    int rows = (sim.log.rows < max_rows) ? sim.log.rows : max_rows;
    for (int r = 0; r < rows; r++) {
        for (int k = 0; k < 7; k++) {
            out[7 * r + k] = sim.log.data[sim.log.width * r + k];
        }
    }
    for (int i = 0; i < 3; i++) {
        out[7 * rows + 1 + 2 * i] = creal(sim.x[i]);
        out[7 * rows + 2 + 2 * i] = cimag(sim.x[i]);
    }
    simulation_free(&sim);
    return rows;
}

static Modulator modulator;

void modulator_new(int carrier, double N, double t_d, int delay, int single_update)
{
    modulator_init(&modulator, carrier, N, t_d, delay, single_update);
}

/* Switching intervals of a sampling period: the durations, the gate signals, and
 * the blanking signals as rows of seven values. Returns the number of intervals. */
int modulate(double T_s, const double *d_abc, double *out)
{
    SwitchingSequence seq;
    if (modulator_modulate(&modulator, T_s, d_abc, &seq) != SIM_OK) {
        return -1;
    }
    for (int i = 0; i < seq.n; i++) {
        out[7 * i] = seq.t_step[i];
        for (int k = 0; k < 3; k++) {
            out[7 * i + 1 + k] = seq.q_abc[i][k];
            out[7 * i + 4 + k] = seq.b_abc[i][k];
        }
    }
    return seq.n;
}
"""


@pytest.fixture(scope="module")
def dll(tmp_path_factory: pytest.TempPathFactory) -> ctypes.CDLL:
    lib = compile_library(CAPI, tmp_path_factory.mktemp("sim"))
    lib.integrate.restype = ctypes.c_int
    lib.modulate.restype = ctypes.c_int
    return lib


def vdp(t: float, x: Any) -> Any:
    """State derivatives of the model of the C test API, with the voltage 0."""
    return [x[1], 5.0 * (1.0 - x[0] * x[0]) * x[1] - x[0], (-1.0 + 10.0j) * x[2] + x[0]]


@pytest.mark.parametrize(("N_eval", "max_step"), [(0, np.inf), (0, 0.05), (7, np.inf)])
def test_solver(dll: ctypes.CDLL, N_eval: int, max_step: float) -> None:
    """RK45 over an integration interval, as solve_ivp in motulator."""
    T_s, rtol, atol = 3.0, 1e-6, 1e-8
    x0 = np.array([2.0, 0.0, 1.0 + 0.5j])
    out = (ctypes.c_double * (7 * 2000 + 7))()
    xy0 = np.column_stack((np.real(x0), np.imag(x0)))
    rows = dll.integrate(
        d(T_s), arr(xy0), d(rtol), d(atol), d(max_step), N_eval, out, 2000
    )
    assert rows > 0
    data = np.array(out)[: 7 * rows + 7].reshape(-1, 7)
    t_c, x_c = data[:rows, 0], data[:, 1::2] + 1j * data[:, 2::2]

    # The voltage of the converter is zero with the duty ratios 0.5
    t_eval = np.linspace(0, T_s, N_eval + 1) if N_eval else None
    sol = solve_ivp(
        vdp,
        (0, T_s),
        x0,
        method="RK45",
        t_eval=t_eval,
        first_step=min(T_s, max_step),
        max_step=max_step,
        rtol=rtol,
        atol=atol,
    )
    x_end = sol.y[:, -1]  # Final states, from the dense output with N_eval
    t_py, x_py = (sol.t[:-1], sol.y[:, :-1]) if N_eval else (sol.t, sol.y)
    # The same steps, up to the rounding errors (e.g., numpy and BLAS sum in a
    # different order)
    err_t, err_x = np.max(np.abs(t_c - t_py)), np.max(np.abs(x_c[:rows].T - x_py))
    print(f"  {len(t_py)} points, max errors: t {err_t:.3g}, x {err_x:.3g}")
    assert len(t_c) == len(t_py)
    assert err_t < 1e-12
    assert err_x < 1e-9
    assert np.max(np.abs(x_c[rows] - x_end)) < 1e-9


@pytest.mark.parametrize("carrier", [False, True])
@pytest.mark.parametrize(
    ("delay", "single_update"), [(0, False), (1, False), (2, True)]
)
def test_modulator(
    dll: ctypes.CDLL, carrier: bool, delay: int, single_update: bool
) -> None:
    """Computational delay and PWM models, with the dead time."""
    N, t_d = 2**6, 3e-6
    drive = model.Drive(
        model.SynchronousMachine(model.SynchronousMachinePars(3, 3.6, 0.04, 0.05, 0.5)),
        model.MechanicalSystem(J=0.015),
        model.VoltageSourceConverter(u_dc=540, t_d=t_d),
        pwm=carrier,
        delay=delay,
        single_update=single_update,
    )
    if carrier:
        drive.pwm = CarrierComparison(N=N, t_d=t_d)
    dll.modulator_new(int(carrier), d(N), d(t_d), delay, int(single_update))

    # Duty ratios, including 0, 1, short pulses, and quantization boundaries, and
    # varying sampling periods
    rng = np.random.default_rng(2)
    d_abc = rng.uniform(0, 1, (200, 3))
    d_abc[::7, 0], d_abc[::11, 1], d_abc[::13, 2] = 0.0, 1.0, 0.5 / N
    d_abc[::17] = (np.round(N * d_abc[::17]) + 0.5) / N
    d_abc[5::19, 0] = 1e-3
    T_s = 1e-4 * rng.choice([1.0, 1.25, 0.8], 200)
    out = (ctypes.c_double * (7 * 64))()
    for k in range(200):
        t_py, q_py, b_py = drive.modulate(T_s[k], list(d_abc[k]))
        n = dll.modulate(d(T_s[k]), arr(d_abc[k]), out)
        c = np.array(out)[: 7 * n].reshape(n, 7)
        assert n == len(t_py)
        assert np.array_equal(c[:, 0], t_py)
        assert np.array_equal(c[:, 1:4], q_py)
        assert np.array_equal(c[:, 4:7], b_py)
