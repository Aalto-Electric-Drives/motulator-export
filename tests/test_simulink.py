"""
Test the generated Simulink S-functions against motulator.

The S-functions are generated from the C-Script code of the PLECS models and
compiled with gcc against a mock of the Simulink API (`tests/simulink_mock`). Each
control system runs in a closed-loop simulation of motulator, and the results are
compared with those of the control system of motulator. MATLAB is not needed.

Run from the repository root:

    pytest tests/test_simulink.py

"""

# %%
import ctypes
import subprocess
from collections.abc import Callable, Sequence
from math import pi
from pathlib import Path
from types import SimpleNamespace
from typing import Any, cast

import motulator.drive.control.im as im_control
import motulator.drive.control.sm as sm_control
import numpy as np
import pytest
from motulator.common.control._base import ControlSystem, TimeSeries
from motulator.common.utils import complex2abc
from motulator.drive import model
from motulator.drive.control._base import VectorControlSystem

from motulator_plecs import im, sm
from motulator_plecs._common import C_SOURCES, ControlBlock
from motulator_plecs.simulink._sfunction import control_sfunction
from tests.c_port import GCC, arr

MOCK = Path(__file__).parent / "simulink_mock"


def compile_sfunction(block: ControlBlock, out_dir: Path) -> ctypes.CDLL:
    """Generate the S-function of the block and compile it against the mock."""
    src = control_sfunction(block).write(out_dir)
    lib = out_dir / "libsfun.so"
    cmd = [*GCC, "-Wall", "-Werror", "-Wno-unused-function", "-DMATLAB_MEX_FILE"]
    cmd += [f"-I{MOCK}", f"-I{C_SOURCES}", str(src), "-lm", "-o", str(lib)]
    subprocess.run(cmd, check=True)
    dll = ctypes.CDLL(str(lib))
    dll.sfun_start.restype = ctypes.c_char_p
    dll.sfun_sample_time.restype = ctypes.c_double
    return dll


@pytest.fixture(scope="module")
def fvc(tmp_path_factory: pytest.TempPathFactory) -> ctypes.CDLL:
    return compile_sfunction(sm.FVC_BLOCK, tmp_path_factory.mktemp("fvc"))


@pytest.fixture(scope="module")
def cvc(tmp_path_factory: pytest.TempPathFactory) -> ctypes.CDLL:
    return compile_sfunction(im.CVC_BLOCK, tmp_path_factory.mktemp("cvc"))


def sfunction_params(block: ControlBlock, values: dict[str, Any]) -> list[Any]:
    """
    Values of the S-function parameters (the C-Script parameters) of a block.

    The values are those of the mask; without a GradNet flux map, its parameters
    are empty.

    """
    params = block.cscript_params or [m.variable for m in block.mask_params]
    out = []
    for p in params:
        if p == "isempty(psi_s_dq_fcn)":
            out.append(1.0)
        elif p.startswith("gn_"):
            out.append(None)
        else:
            out.append(values[p])
    return out


def start(dll: ctypes.CDLL, params: list[Any]) -> str | None:
    """Start the S-function with the parameters, returning the error message."""
    arrays = [
        np.zeros(0) if p is None else np.atleast_1d(np.asarray(p, dtype=float))
        for p in params
    ]
    numel = (ctypes.c_int * len(arrays))(*[a.size for a in arrays])
    err = dll.sfun_start(len(arrays), numel, arr(np.concatenate(arrays)))
    return None if err is None else err.decode()


class SFunctionControlSystem(VectorControlSystem):
    """Control system running the S-function, for closed-loop simulations."""

    def __init__(
        self,
        ctrl: VectorControlSystem,
        dll: ctypes.CDLL,
        block: ControlBlock,
        sensor: str,
    ) -> None:
        super().__init__(ctrl.vector_ctrl, ctrl.speed_ctrl)  # For the measurements
        self.ext_ref = ctrl.ext_ref
        self.T_s = cast(Any, ctrl.vector_ctrl).cfg.T_s
        self.dll = dll
        self.sensor = sensor  # Measured in the sensored mode: theta_M or w_M
        self.names = ["d_a", "d_b", "d_c", *block.signals]
        self.y = (ctypes.c_double * len(self.names))()

    def run_control_loop(self, mdl: model.Drive) -> tuple[float, Sequence[float]]:
        meas = self.get_measurement(mdl)
        w_M_ref = cast(Any, self.ext_ref.w_M)(self.t)
        x = getattr(meas, self.sensor) or 0.0
        u = [w_M_ref, *complex2abc(meas.i_c_ab), meas.u_dc, x]
        self.dll.sfun_step(arr(u), self.y)
        out = np.array(self.y)
        self.save(
            self.t, out=SimpleNamespace(**dict(zip(self.names, out, strict=True)))
        )
        self.t = (self.t + self.T_s) % 1e9  # As in motulator
        return self.T_s, list(out[:3])

    def post_process(self) -> TimeSeries:
        return ControlSystem.post_process(self)  # Only the saved S-function outputs


def compare(
    build: Callable[[], tuple[model.Drive, VectorControlSystem]],
    dll: ctypes.CDLL,
    block: ControlBlock,
    sensor: str,
    signals: dict[str, Callable[[Any, Any], Any]],
) -> None:
    """Simulate with the control systems of motulator and of the S-function."""
    mdl, ctrl = build()
    res = model.Simulation(mdl, ctrl).simulate(t_stop=0.3)
    fbk, ref = res.ctrl.fbk, res.ctrl.ref
    py = {
        "d_a": ref.d_abc[:, 0],
        "d_b": ref.d_abc[:, 1],
        "d_c": ref.d_abc[:, 2],
        **{name: f(fbk, ref) for name, f in signals.items()},
    }
    mdl, ctrl = build()
    sl_ctrl = SFunctionControlSystem(ctrl, dll, block, sensor)
    res_sl = model.Simulation(mdl, sl_ctrl).simulate(t_stop=0.3)
    assert np.allclose(res.ctrl.t, res_sl.ctrl.t)
    for name, value in py.items():
        err = np.max(np.abs(getattr(res_sl.ctrl.out, name) - value))
        scale = np.max(np.abs(value))
        print(f"  {name}: max error {err:.3g} (max value {scale:.3g})")
        assert err <= 1e-8 * max(scale, 1.0), name


# %%
SM_PAR = {"n_p": 3, "R_s": 3.6, "L_d": 0.036, "L_q": 0.051, "psi_f": 0.545}
SM_SPEED = {"J": 0.015, "alpha_s": 25.0}


def test_parameter_errors(fvc: ctypes.CDLL) -> None:
    """Missing or invalid parameters should be reported."""
    values: dict[str, Any] = dict.fromkeys(m.variable for m in sm.MASK_PARAMS)
    assert start(fvc, sfunction_params(sm.FVC_BLOCK, values)) is not None
    values |= SM_PAR | {"i_s_max": 6.5, "T_s": 125e-6}
    values |= {"speed_J": 0.015, "speed_alpha_s": 25.0, "k_o": [1.0, 2.0, 3.0]}
    assert start(fvc, sfunction_params(sm.FVC_BLOCK, values)) == (
        "k_o must be [] or [k0 k1]."
    )
    values["k_o"] = None
    assert start(fvc, sfunction_params(sm.FVC_BLOCK, values)) is None
    assert fvc.sfun_sample_time() == 125e-6
    fvc.sfun_terminate()


@pytest.mark.parametrize("sensorless", [True, False])
def test_flux_vector_control(fvc: ctypes.CDLL, sensorless: bool) -> None:
    """The S-function of flux-vector control should agree with motulator."""

    def build() -> tuple[model.Drive, VectorControlSystem]:
        par = model.SynchronousMachinePars(**SM_PAR)
        mdl = model.Drive(
            model.SynchronousMachine(par),
            model.MechanicalSystem(J=0.015),
            model.VoltageSourceConverter(u_dc=540),
        )
        mdl.mechanics.set_external_load_torque(lambda t: (t > 0.2) * 10.0)
        cfg = sm_control.FluxVectorControllerCfg(i_s_max=6.5, sensorless=sensorless)
        ctrl = VectorControlSystem(
            sm_control.FluxVectorController(par, cfg),
            sm_control.SpeedController(**SM_SPEED),
        )
        ctrl.set_speed_ref(lambda t: (t > 0.05) * 2 * pi * 20)
        return mdl, ctrl

    values, _ = sm.export_mask_values(build()[1], SM_SPEED)
    assert start(fvc, sfunction_params(sm.FVC_BLOCK, values)) is None
    signals = {
        "w_M": lambda fbk, _: fbk.w_M,
        "tau_M_ref": lambda _, ref: ref.tau_M,
        "psi_s_ref": lambda _, ref: ref.psi_s,
        "psi_s": lambda fbk, _: np.abs(fbk.psi_s),
    }
    compare(build, fvc, sm.FVC_BLOCK, "theta_M", signals)
    fvc.sfun_terminate()


@pytest.mark.parametrize("sensorless", [True, False])
def test_current_vector_control(cvc: ctypes.CDLL, sensorless: bool) -> None:
    """The S-function of current-vector control should agree with motulator."""
    par = model.InductionMachineInvGammaPars(
        n_p=2, R_s=3.7, R_R=2.1, L_sgm=0.021, L_M=0.224
    )
    speed = {"J": 0.015, "alpha_s": 2 * pi * 4}

    def build() -> tuple[model.Drive, VectorControlSystem]:
        mdl = model.Drive(
            model.InductionMachine(par),
            model.MechanicalSystem(J=0.015),
            model.VoltageSourceConverter(u_dc=540),
        )
        mdl.mechanics.set_external_load_torque(lambda t: (t > 0.2) * 14.6)
        cfg = im_control.CurrentVectorControllerCfg(
            psi_s_nom=1.04, i_s_max=10.6, sensorless=sensorless
        )
        ctrl = VectorControlSystem(
            im_control.CurrentVectorController(par, cfg),
            im_control.SpeedController(**speed),
        )
        ctrl.set_speed_ref(lambda t: (t > 0.05) * 2 * pi * 20)
        return mdl, ctrl

    values = im.export_mask_values(build()[1], speed)
    assert start(cvc, sfunction_params(im.CVC_BLOCK, values)) is None
    signals = {
        "w_M": lambda fbk, _: fbk.w_M,
        "tau_M_ref": lambda _, ref: ref.tau_M,
        "psi_R": lambda fbk, _: np.abs(fbk.psi_R),
    }
    compare(build, cvc, im.CVC_BLOCK, "w_M", signals)
    cvc.sfun_terminate()
