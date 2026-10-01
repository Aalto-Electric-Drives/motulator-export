"""
Test the Simulink S-function of flux-vector control against motulator.

The S-function is compiled with gcc against a mock of the Simulink API
(`tests/simulink_mock`), and its outputs are compared with motulator for a sequence
of measurements, in the sensorless and sensored modes. MATLAB is not needed.

Run from the repository root:

    pytest tests/test_simulink.py

"""

# %%
import ctypes
import subprocess
from math import pi
from pathlib import Path
from typing import Any, cast

import motulator.drive.control.sm as control
import numpy as np
import pytest
from motulator.drive import model
from motulator.drive.control._base import Measurements

from motulator_plecs._common import C_SOURCES
from motulator_plecs.simulink import sm as simulink_sm
from tests.c_port import arr

MOCK = Path(__file__).parent / "simulink_mock"


@pytest.fixture(scope="module")
def sfun(tmp_path_factory: pytest.TempPathFactory) -> ctypes.CDLL:
    lib = tmp_path_factory.mktemp("sfun") / "libsfun.so"
    cmd = ["gcc", "-std=c99", "-O2", "-shared", "-fPIC", "-Wall", "-Werror"]
    cmd += ["-Wno-unused-function", "-DMATLAB_MEX_FILE", f"-I{MOCK}", f"-I{C_SOURCES}"]
    cmd += [str(simulink_sm.SFUNCTION), "-lm", "-o", str(lib)]
    subprocess.run(cmd, check=True)
    dll = ctypes.CDLL(str(lib))
    dll.sfun_start.restype = ctypes.c_char_p
    dll.sfun_sample_time.restype = ctypes.c_double
    return dll


def start(dll: ctypes.CDLL, values: dict[str, Any]) -> str | None:
    """Start the S-function with the parameters, returning the error message."""
    params = [np.atleast_1d(np.asarray(values[n], dtype=float)) for n in PARAM_NAMES]
    params = [
        np.zeros(0) if values[n] is None else p
        for n, p in zip(PARAM_NAMES, params, strict=True)
    ]
    numel = (ctypes.c_int * len(params))(*[p.size for p in params])
    err = dll.sfun_start(len(params), numel, arr(np.concatenate(params)))
    return None if err is None else err.decode()


PARAM_NAMES = simulink_sm.PARAM_NAMES
PAR = {"n_p": 3, "R_s": 3.6, "L_d": 0.036, "L_q": 0.051, "psi_f": 0.545}
SPEED = {"J": 0.015, "alpha_s": 25.0}


def test_param_names() -> None:
    """The parameters of the S-function should be in the order of the mask."""
    assert simulink_sm.sfunction_param_names() == PARAM_NAMES


def test_parameter_errors(sfun: ctypes.CDLL) -> None:
    """Missing required parameters should be reported."""
    values: dict[str, Any] = dict.fromkeys(PARAM_NAMES)
    assert start(sfun, values) is not None
    values |= PAR | {"i_s_max": 6.5, "T_s": 125e-6}
    values |= {"speed_J": 0.015, "speed_alpha_s": 25.0, "k_o": [1.0, 2.0, 3.0]}
    assert start(sfun, values) == "k_o must be [] or [k0 k1]."
    values["k_o"] = None
    assert start(sfun, values) is None
    assert sfun.sfun_sample_time() == 125e-6
    sfun.sfun_terminate()


@pytest.mark.parametrize("sensorless", [True, False])
def test_control_system(sfun: ctypes.CDLL, sensorless: bool) -> None:
    """The S-function should give the same outputs as motulator."""
    par = model.SynchronousMachinePars(**PAR)
    cfg = control.FluxVectorControllerCfg(i_s_max=6.5, sensorless=sensorless)
    ctrl = control.VectorControlSystem(
        control.FluxVectorController(par, cfg), control.SpeedController(**SPEED)
    )
    values = simulink_sm.export_mask_values(ctrl, SPEED)
    assert start(sfun, values) is None
    assert sfun.sfun_sample_time() == cfg.T_s

    # Measurement sequence: rotating current vector with a varying amplitude
    rng = np.random.default_rng(0)
    n = 4000
    t = np.arange(n) * cfg.T_s
    i_s_ab = (2 + np.sin(2 * pi * 3 * t)) * np.exp(1j * 2 * pi * 20 * t)
    i_s_ab += 0.05 * (rng.standard_normal(n) + 1j * rng.standard_normal(n))
    theta_M = 2 * pi * 7 * t  # Rotor angle for the sensored mode
    ctrl.set_speed_ref(lambda t_: 50.0 if t_ > 0.05 else 0.0)

    y = (ctypes.c_double * 12)()
    err = np.zeros(12)
    for k in range(n):
        i_abc = [
            i_s_ab[k].real,
            0.5 * (-i_s_ab[k].real + np.sqrt(3) * i_s_ab[k].imag),
            0.5 * (-i_s_ab[k].real - np.sqrt(3) * i_s_ab[k].imag),
        ]
        # motulator
        meas = Measurements(i_s_ab[k], 540.0, theta_M=float(theta_M[k]))
        fbk = cast(Any, ctrl.get_feedback(meas))
        ref = cast(Any, ctrl.compute_output(fbk))
        ctrl.update(ref, fbk)
        py = [*ref.d_abc, ref.w_M, fbk.w_M, ref.tau_M, fbk.tau_M, ref.psi_s]
        py += [abs(fbk.psi_s), fbk.theta_m, fbk.i_s.real, fbk.i_s.imag]
        # S-function, with the speed reference sampled by motulator
        sfun.sfun_step(arr([ref.w_M, *i_abc, 540.0, theta_M[k]]), y)
        err = np.maximum(err, np.abs(np.array(y) - np.array(py)))
    sfun.sfun_terminate()
    names = ["d_a", "d_b", "d_c", *simulink_sm.CTRL_SIGNALS]
    for name, e in zip(names, err, strict=True):
        assert e < 1e-9, (name, e)
    print("  max errors:", dict(zip(names, np.round(err, 16), strict=True)))
