"""
S-functions compiled against a mock of the Simulink API (`tests/simulink_mock`),
run as control systems of motulator in closed-loop simulations.

"""

# %%
import ctypes
import subprocess
from collections.abc import Callable, Sequence
from math import pi
from pathlib import Path
from types import SimpleNamespace
from typing import Any, cast

import motulator.grid.control as grid_control
import motulator.grid.model as grid_model
import numpy as np
from motulator.common.control._base import ControlSystem, TimeSeries
from motulator.common.utils import complex2abc
from motulator.common.utils._utils import get_value, wrap
from motulator.drive import model
from motulator.drive.control._base import (
    Measurements,
    VectorControlSystem,
    VHzControlSystem,
)

from motulator_export._common import C_SOURCES
from motulator_export.plecs import _grid
from motulator_export.plecs._common import ControlBlock
from motulator_export.simulink._sfunction import SFunction
from tests.c_port import GCC, arr
from tests.monolithic import MonolithicBlock, control_sfunction
from tests.systems import BASE

MOCK = Path(__file__).parent / "simulink_mock"
Block = MonolithicBlock | ControlBlock


def compile_sfunction(sfun: MonolithicBlock | SFunction, out_dir: Path) -> ctypes.CDLL:
    """Generate the S-function (of a block) and compile it against the mock."""
    if isinstance(sfun, MonolithicBlock):
        sfun = control_sfunction(sfun)
    src = sfun.write(out_dir)
    lib = out_dir / "libsfun.so"
    cmd = [*GCC, "-Wall", "-Werror", "-Wno-unused-function", "-DMATLAB_MEX_FILE"]
    cmd += [f"-I{MOCK}", f"-I{C_SOURCES}", str(src), "-lm", "-o", str(lib)]
    subprocess.run(cmd, check=True)
    dll = ctypes.CDLL(str(lib))
    dll.sfun_start.restype = ctypes.c_char_p
    dll.sfun_sample_time.restype = ctypes.c_double
    return dll


# Values of the parameters of the started S-functions (see start)
PARAM_VALUES: dict[int, Any] = {}


def start(dll: ctypes.CDLL, params: list[Any]) -> str | None:
    """
    Start the S-function with the parameters, returning the error message.

    The parameters are passed as column vectors, the matrices in column-major order
    as in MATLAB.

    """
    arrays = [
        np.zeros(0) if p is None else np.asarray(p, dtype=float).ravel(order="F")
        for p in params
    ]
    numel = (ctypes.c_int * len(arrays))(*[a.size for a in arrays])
    # The mock refers to the values of the parameters, so they are kept, as Simulink
    # keeps the parameters
    values = PARAM_VALUES[id(dll)] = arr(np.concatenate([np.zeros(0), *arrays]))
    err = dll.sfun_start(len(arrays), numel, values)
    return None if err is None else err.decode()


def restarter(dll: ctypes.CDLL, params: list[Any]) -> Callable[[], None]:
    """Function restarting the S-function with the parameters."""

    def restart() -> None:
        assert start(dll, params) is None

    return restart


class SFunctionControlSystem(ControlSystem):
    """Control system running the S-function, for closed-loop simulations."""

    def __init__(
        self,
        dll: Any,
        block: Block,
        T_s: float,
        inputs: Callable[[Any, float], list[float]],
    ) -> None:
        super().__init__()
        self.dll = dll
        self.T_s = T_s
        self.inputs = inputs  # Inputs of the S-function from the model and the time
        self.names = ["d_a", "d_b", "d_c", *block.signals]
        self.y = (ctypes.c_double * len(self.names))()
        self.recorded: list[list[float]] = []  # The inputs of each step

    # The control loop is overridden, so the steps of the protocol are not used
    def get_measurement(self, mdl: Any) -> Any:
        raise NotImplementedError

    def get_feedback(self, meas: Any) -> Any:
        raise NotImplementedError

    def compute_output(self, fbk: Any) -> Any:
        raise NotImplementedError

    def compute_standby_output(self, fbk: Any) -> Any:
        raise NotImplementedError

    def run_control_loop(self, mdl: Any) -> tuple[float, Sequence[float]]:
        u = self.inputs(mdl, self.t)
        self.recorded.append(u)
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
    build: Callable[[], tuple[Any, Any]],
    simulation: Callable[[Any, Any], Any],
    sfun: Callable[[Any], SFunctionControlSystem],
    signals: dict[str, Callable[[Any, Any], Any]],
    t_stop: float,
) -> list[list[float]]:
    """
    Simulate with the control systems of motulator and of the S-function.

    The system is built twice by `build`, and `sfun` creates the control system of
    the S-function from the control system of motulator (for its references).
    Returns the inputs of the S-function at each step.

    """
    mdl, ctrl = build()
    res = simulation(mdl, ctrl).simulate(t_stop=t_stop)
    fbk, ref = res.ctrl.fbk, res.ctrl.ref
    py = {
        "d_a": ref.d_abc[:, 0],
        "d_b": ref.d_abc[:, 1],
        "d_c": ref.d_abc[:, 2],
        **{name: f(fbk, ref) for name, f in signals.items()},
    }
    mdl, ctrl = build()
    sl = sfun(ctrl)
    res_sl = simulation(mdl, sl).simulate(t_stop=t_stop)
    assert np.allclose(res.ctrl.t, res_sl.ctrl.t)
    for name, value in py.items():
        err = np.max(np.abs(getattr(res_sl.ctrl.out, name) - value))
        scale = np.max(np.abs(value))
        print(f"  {name}: max error {err:.3g} (max value {scale:.3g})")
        assert err <= 1e-8 * max(scale, 1.0), name
    return sl.recorded


def check_dc_bus_limit(
    dll: Any, block: Block, inputs: list[float], restart: Callable[[], None]
) -> None:
    """
    Check the lower limit of the measured DC-bus voltage.

    The S-function, started by `restart`, is run with the given constant inputs
    (without `enable`, which is 1). The outputs should stay finite without the DC-bus
    voltage, e.g., if the gates are enabled before the DC bus is charged, the
    measured voltage being limited to U_DC_MIN = 1 V.

    """
    y = (ctypes.c_double * (3 + len(block.signals)))()
    k_u_dc = sum(block.input_widths[1 : block.inputs.index("u_dc")])

    def run(u_dc: float) -> np.ndarray:
        restart()
        u = list(inputs)
        u[k_u_dc] = u_dc
        out = np.zeros((2000, len(y)))
        for k in range(len(out)):
            dll.sfun_step(arr([1.0, *u]), y)
            out[k] = y
        return out

    limited = run(1.0)
    assert np.all(np.isfinite(limited))
    for u_dc in (0.0, -5.0):
        assert np.array_equal(run(u_dc), limited)


def drive_sfun(dll: Any, block: Block) -> Callable[[Any], SFunctionControlSystem]:
    """
    S-function control system of a drive, with the measured angle or speed (not in
    V/Hz control).
    """

    def sfun(ctrl: VectorControlSystem | VHzControlSystem) -> SFunctionControlSystem:
        def inputs(mdl: model.Drive, t: float) -> list[float]:
            mech = mdl.mechanics
            i_s_abc = mdl.machine.meas_currents()
            u_dc = mdl.converter.meas_dc_voltage()
            enabled = float(bool(get_value(ctrl._enable, t)))  # The gate enable
            w_M_ref = cast(Any, ctrl.ext_ref.w_M)(t)
            u = [enabled, w_M_ref, *i_s_abc, u_dc]
            if "theta_M" in block.inputs:
                u.append(wrap(mech.meas_position()))
            elif "w_M" in block.inputs:
                u.append(mech.meas_speed())
            return u

        if isinstance(ctrl, VHzControlSystem):
            T_s = cast(Any, ctrl.vhz_ctrl).cfg.T_s
        else:
            T_s = cast(Any, ctrl.vector_ctrl).cfg.T_s
        sl = SFunctionControlSystem(dll, block, T_s, inputs)
        sl.set_enable(ctrl._enable)  # For blocking the gates of the system model
        return sl

    return sfun


def grid_sfun(
    dll: Any, block: Block
) -> Callable[[grid_control.GridConverterControlSystem], SFunctionControlSystem]:
    """S-function control system of a grid converter."""

    def sfun(ctrl: grid_control.GridConverterControlSystem) -> SFunctionControlSystem:
        ext_ref = cast(Any, ctrl.ext_ref)

        def inputs(mdl: grid_model.GridConverterSystem, t: float) -> list[float]:
            i_c_abc = mdl.ac_filter.meas_currents()
            u_dc = mdl.converter.meas_dc_voltage()
            enabled = float(bool(get_value(ctrl._enable, t)))  # The gate enable
            u_g_line = mdl.ac_filter.meas_pcc_voltages()
            if "q_g_ref" in block.inputs:
                refs = [ext_ref.p_g(t), ext_ref.q_g(t)]
            else:
                refs = [ext_ref.p_g(t), ext_ref.v_c]
            return [enabled, *refs, *i_c_abc, *u_g_line, u_dc]

        T_s = _grid.export_values(ctrl)["T_s"]
        sl = SFunctionControlSystem(dll, block, T_s, inputs)
        sl.set_enable(ctrl._enable)  # For blocking the gates of the system model
        return sl

    return sfun


# Constant inputs of `check_dc_bus_limit` (without enable), with zero converter
# voltages
SM_INPUTS = [100.0, 3.0, -1.0, -2.0, 540.0, 0.3]
IM_INPUTS = [100.0, 3.0, -1.0, -2.0, 540.0, 0.0]
VHZ_INPUTS = [100.0, 3.0, -1.0, -2.0, 540.0]
# Line-to-line PCC voltages u_ab and u_bc of the voltage vector BASE.u
GFL_INPUTS = [5e3, 1e3, 3.0, -1.0, -2.0, 1.5 * BASE.u, 0.0, 650.0]
GFM_INPUTS = [5e3, BASE.u, 3.0, -1.0, -2.0, 1.5 * BASE.u, 0.0, 650.0]


def gradnet_inputs(ctrl: VectorControlSystem, n: int) -> list[tuple[Any, list[float]]]:
    """
    Measurements of the control system with a GradNet flux map, and the inputs of
    the S-function: an accelerating rotor, and a current vector in rotor coordinates.
    Later, the flux decreases until the torque-production factor c_tau of the
    flux-torque controller approaches zero, where the rounding errors are amplified
    and the flux reduction (c_tau < 0) may start at a different sample.
    """
    T_s = cast(Any, ctrl.vector_ctrl).cfg.T_s
    t = np.arange(n) * T_s
    w_M = 100.0 * t / t[-1]
    theta_M = np.cumsum(w_M) * T_s
    i_s_dq = (5 + 10 * np.sin(2 * pi * 5 * t)) + 1j * (8 + 12 * np.cos(2 * pi * 3 * t))
    i_s_ab = np.exp(1j * 2 * theta_M) * i_s_dq
    w_M_ref = np.where(t > 0.05, 50.0, 0.0)
    out = []
    for k in range(n):
        th = float(wrap(theta_M[k]))
        meas = Measurements(i_s_ab[k], 540.0, w_M[k], th)
        u = [1.0, float(w_M_ref[k]), *complex2abc(i_s_ab[k]), 540.0, th]
        out.append((meas, u))
    return out
