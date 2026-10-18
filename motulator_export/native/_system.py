"""
Systems in the native libraries.

A native library (`c/native_<name>.c`) holds a system: a model and a control system
with their simulation. The libraries share the API of `c/sim.c` (the simulation, its
logs, the signals, and the converter with its DC bus and modulator) and provide
`system_new`, `system_free`, and `start`, while their other functions setting the
parameters are specific to the model (e.g., `c/drive.c`) and the control system
(e.g., `c/native_sm.c`). `NativeSystem` wraps a system of a library, and its
subclasses set the parameters from the motulator objects.

"""

import ctypes
import weakref
from collections.abc import Callable
from math import nan
from typing import Any

import numpy as np
from motulator.common.model import SolverCfg
from motulator.common.model._base import Model
from motulator.common.model._converter import (
    DCBus,
    DiodeBridgeDCBus,
    VoltageSourceConverter,
)
from motulator.common.model._pwm import ZOH, CarrierComparison

from motulator_export.native._build import Signatures, declare, load_library
from motulator_export.plecs._common import (
    StepSignal,
    _c_define,
    _check_supported_pwm,
    gate_enable,
)
from motulator_export.plecs._control import pwm_values

c_double_p = ctypes.POINTER(ctypes.c_double)
c_int = ctypes.c_int
c_double = ctypes.c_double
c_void_p = ctypes.c_void_p
SignalFunction = ctypes.CFUNCTYPE(c_double, c_double)

# Functions of all the libraries
API: Signatures = {
    "system_new": (c_void_p, []),
    "system_free": (None, [c_void_p]),
    "start": (None, [c_void_p, c_double, c_double_p] + [c_double] * 5),
    "sim_run": (c_int, [c_void_p, c_double, c_int]),
    "sim_state": (c_double, [c_void_p, c_double_p]),
    "sim_log": (c_void_p, [c_void_p, c_int]),
    "sim_fields": (ctypes.c_char_p, [c_void_p, c_int]),
    "log_rows": (c_int, [c_void_p]),
    "log_width": (c_int, [c_void_p]),
    "log_data": (c_double_p, [c_void_p]),
    "log_clear": (None, [c_void_p]),
    "signal_steps": (None, [c_void_p, c_double, c_int, c_double_p, c_double_p]),
    "signal_function": (None, [c_void_p, SignalFunction]),
    "signal_max_steps": (c_int, []),
    "set_converter": (None, [c_void_p, c_double, c_double]),
    "set_modulator": (None, [c_void_p, c_int, c_double, c_double, c_int, c_int]),
    "set_dc_bus": (None, [c_void_p, c_int] + [c_double] * 4),
}

# Function setting the PWM of the control system, in the libraries with a PWM
PWM_API: Signatures = {
    "set_pwm": (
        None,
        [c_void_p, c_int, c_double, c_double, c_double, c_int, c_double, c_int],
    )
}

# Function setting a GradNet, in the libraries using them
GRADNET_API: Signatures = {
    "gradnet_set": (
        None,
        [c_void_p, c_int, c_int, c_int, c_double_p, c_double_p, c_double_p]
        + [c_double_p, c_int, c_double, c_int, c_double, c_double],
    )
}

DELAY_MAX_LENGTH = _c_define("sim.h", "DELAY_MAX_LENGTH")
# Types of the DC-bus models (DC_BUS_* in sim.h)
DC_BUS_TYPES = {DCBus: 1, DiodeBridgeDCBus: 2}

# Status codes of the simulation (SIM_ERROR_* in sim.h)
ERRORS = {
    1: "Required step size is less than spacing between numbers.",
    2: "A state is not finite.",
    3: "The sampling period must be positive.",
    4: "Out of memory for the results.",
    5: "Too many switching intervals in a sampling period.",
    6: "A reference signal is not finite.",
}


def array(values: Any) -> Any:
    """Array of doubles for ctypes (None becomes NaN)."""
    values = np.asarray(values, dtype=float).ravel()
    return (c_double * len(values))(*values)


def value(x: float | None) -> float:
    """Double for ctypes, None as NaN (the default values are resolved in C)."""
    return nan if x is None else float(x)


class NativeSystem:
    """
    System in a native library.

    Parameters
    ----------
    library : str
        Name of the library (`c/native_<library>.c`).
    api : Signatures
        Signatures of the functions specific to the system.

    """

    def __init__(self, library: str, api: Signatures) -> None:
        self.lib = load_library(f"native_{library}")
        declare(self.lib, {**API, **api})
        handle = self.lib.system_new()
        if not handle:
            raise MemoryError("The system could not be allocated")
        self.handle = handle
        weakref.finalize(self, self.lib.system_free, handle)
        # Functions of the signals, kept alive while the library may call them
        self._functions: dict[int, Any] = {}
        # Exception raised by a function of a signal, raised again after the run
        self.error: BaseException | None = None

    def set_signals(self, mdl: Any, ctrl: Any) -> None:
        """Set the signals of the system from `mdl` and `ctrl` (e.g., references)."""
        raise NotImplementedError

    def set_signal(self, signal: int, x: float | Callable[[float], float]) -> None:
        """
        Set a signal of the system (e.g., a reference), given as a constant, a
        `StepSignal`, or a function of time. A function is called from C, which is
        slower than the steps.
        """
        self._functions.pop(signal, None)
        if isinstance(x, StepSignal):
            times, afters = np.atleast_1d(x.time), np.atleast_1d(x.after)
            max_steps = self.lib.signal_max_steps()
            if len(times) > max_steps:
                raise NotImplementedError(f"At most {max_steps} steps supported")
            self.lib.signal_steps(
                signal, float(x.before), len(times), array(times), array(afters)
            )
        elif callable(x):
            fcn = SignalFunction(self._safe(x))
            self._functions[signal] = fcn
            self.lib.signal_function(signal, fcn)
        else:
            self.lib.signal_steps(signal, float(x), 0, None, None)

    def _safe(self, fcn: Callable[[float], float]) -> Callable[[float], float]:
        """Function storing its exception and returning NaN, which stops the run."""

        def wrapped(t: float) -> float:
            try:
                return float(fcn(t))
            except BaseException as err:  # noqa: BLE001 (raised again after the run)
                if self.error is None:
                    self.error = err
                return nan

        return wrapped

    def set_gradnet(self, net: int, g: dict[str, Any]) -> None:
        """
        Set a GradNet of the system from its parameters (`export_gradnet`), in a library
        with the function of `GRADNET_API`.
        """
        self.lib.gradnet_set(
            net,
            g["in_dim"],
            g["mu_dim"],
            len(g["b"]),
            array(g["W"]),
            array(g["b"]),
            array(g["mu_log"]),
            array(g["bias"]),
            g["activation"],
            g["beta_log"],
            g["p"],
            g["in_base"],
            g["out_base"],
        )

    def set_converter(self, mdl: Model) -> None:
        """Set the converter, its DC bus, and its modulator from the model."""
        lib, h, converter = self.lib, self.handle, mdl.converter
        lib.set_converter(h, value(converter.u_dc), converter.u_th)
        dc_bus = mdl.dc_bus
        if dc_bus is not None:
            bridge = dc_bus if isinstance(dc_bus, DiodeBridgeDCBus) else None
            lib.set_dc_bus(
                h,
                DC_BUS_TYPES[type(dc_bus)],
                dc_bus.C_dc,
                nan if bridge is None else bridge.L_dc,
                nan if bridge is None else bridge.u_g,
                nan if bridge is None else bridge.w_g,
            )
        pwm = mdl.pwm
        N = pwm.N if isinstance(pwm, CarrierComparison) else 0
        delay = len(mdl.delay.data)
        lib.set_modulator(h, N > 0, N, pwm.t_d, delay, mdl.single_update)

    def set_pwm(self, pwm: Any) -> None:
        """Set the PWM of the control system, in a library of `PWM_API`."""
        v = pwm_values(pwm)
        self.lib.set_pwm(
            self.handle,
            v["pwm_overmodulation"] == "MME",
            v["pwm_t_d"],
            v["pwm_u_th"],
            v["pwm_i_0"],
            v["pwm_feedforward"],
            v["pwm_d_min"],
            v["pwm_single_update"],
        )

    def start(self, mdl: Model, t_start: float, t_ctrl: float, cfg: SolverCfg) -> None:
        """
        Start the simulation from the time and the states of the model, with the first
        sampling instant `t_start` and the controller time `t_ctrl`.
        """
        names = [
            n for s in mdl.subsystems if s.state is not None for n in vars(s.state)
        ]
        if names != self.fields(control=False):
            raise NotImplementedError(
                f"The states {self.fields(control=False)} of the model expected"
            )
        x = np.asarray(mdl.get_initial_values(), dtype=complex)
        xy = array(np.column_stack((np.real(x), np.imag(x))))
        self.lib.start(
            self.handle, mdl.t0, xy, t_start, t_ctrl, cfg.max_step, cfg.rtol, cfg.atol
        )

    def run(self, t_stop: float, N_eval: int) -> int:
        """
        Run until the first sampling instant after `t_stop`, returning the status. An
        exception raised by a function of a signal is raised again.
        """
        status = self.lib.sim_run(self.handle, t_stop, N_eval)
        if self.error is not None:
            err, self.error = self.error, None
            raise err
        return status

    def state(self) -> tuple[float, np.ndarray]:
        """Time and states of the model."""
        n = len(self.fields(control=False))
        xy = (c_double * (2 * n))()
        t0 = self.lib.sim_state(self.handle, xy)
        x = np.array(xy).reshape(n, 2)
        return t0, x[:, 0] + 1j * x[:, 1]

    def fields(self, control: bool) -> list[str]:
        """Names of the states of the model or of the logged fields of the control."""
        return self.lib.sim_fields(self.handle, int(control)).decode().split()

    def fetch(self, control: bool) -> np.ndarray:
        """Rows of the log of the model or the control system, clearing the log."""
        log = self.lib.sim_log(self.handle, int(control))
        rows, width = self.lib.log_rows(log), self.lib.log_width(log)
        if rows == 0:
            return np.empty((0, width))
        data = np.ctypeslib.as_array(self.lib.log_data(log), shape=(rows, width)).copy()
        self.lib.log_clear(log)
        return data


def check_supported_converter(mdl: Model) -> None:
    """
    Raise an error if the converter, its DC bus, its PWM model, or the delay is not
    supported.
    """
    converter, pwm, dc_bus = mdl.converter, mdl.pwm, mdl.dc_bus
    if type(converter) is not VoltageSourceConverter or converter.sign is not np.sign:
        raise NotImplementedError(
            "Only VoltageSourceConverter with sign=np.sign supported"
        )
    if converter.u_dc_sensor is not None:
        raise NotImplementedError("Sensor models not supported")
    if dc_bus is None and converter.u_dc is None:
        raise ValueError(
            "The DC-bus voltage must be given, unless the converter is fed from a "
            "DC-bus model"
        )
    if dc_bus is not None:
        if type(dc_bus) not in DC_BUS_TYPES:
            raise NotImplementedError("Only DCBus and DiodeBridgeDCBus supported")
        if dc_bus.converters != [converter]:
            raise NotImplementedError(
                "DC bus shared with other converters not supported"
            )
        if dc_bus.inp.i_dc is not None:
            raise NotImplementedError("External DC current not supported")
    if type(pwm) not in (ZOH, CarrierComparison):
        raise NotImplementedError(
            "Only the PWM models ZOH and CarrierComparison supported"
        )
    delay = mdl.delay.data
    if len(delay) > DELAY_MAX_LENGTH:
        raise NotImplementedError(f"At most the delay {DELAY_MAX_LENGTH} supported")
    initial = not np.any(delay)
    if isinstance(pwm, CarrierComparison):
        initial &= pwm._rising_edge and pwm._command is None
    if not initial:
        raise NotImplementedError("Only the initial state of the PWM model supported")


def check_supported_control(ctrl: Any, d_err: bool) -> None:
    """
    Raise an error if the PWM of the control system is not supported (the duty-ratio
    error model only if `d_err`, and then without the minimum pulses if compensated)
    or the gates are blocked.
    """
    _check_supported_pwm(ctrl, d_err=d_err)
    pwm = ctrl.pwm
    pwm_values(pwm)  # Raises if the duty-ratio error model is not supported
    if pwm.d_err is not None and pwm.feedforward and pwm.d_min > 0:
        raise NotImplementedError(
            "Minimum pulses (d_min) with the compensation of the duty-ratio error not "
            "supported"
        )
    if gate_enable(ctrl) != 1:
        raise NotImplementedError("Gate blocking (set_enable) not supported")
