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
from collections.abc import Callable, Sequence
from math import nan
from typing import Any, cast

import numpy as np
from motulator.common.model import SolverCfg, ThreePhaseSource
from motulator.common.model._base import Model
from motulator.common.model._converter import DCBus, DiodeBridge, VoltageSourceConverter
from motulator.common.model._pwm import ZOH, CarrierComparison
from motulator.grid.model import GridConverterSystem, LFilter

from motulator_export._common import (
    StepSignal,
    c_define,
    check_supported_dc_bus,
    check_supported_front,
    check_supported_pwm,
    gate_enable,
)
from motulator_export.native._build import Signatures, declare, load_library
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
    "sim_converter_state": (c_int, [c_void_p, c_double_p, c_double_p]),
    "sim_set_converter_state": (None, [c_void_p, c_int, c_double_p]),
    "sim_bridge_state": (None, [c_void_p, c_double_p]),
    "sim_set_bridge_state": (None, [c_void_p, c_double_p]),
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
    "set_dc_bus": (None, [c_void_p, c_int, c_double]),
    "set_diode_bridge": (None, [c_void_p] + [c_double] * 5),
    "enable_signal": (c_void_p, [c_void_p]),
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

DELAY_MAX_LENGTH = c_define("sim.h", "DELAY_MAX_LENGTH")
# Types of the DC-bus models (DC_BUS_* in sim.h)
DC_BUS_TYPES = {DCBus: 1}

# Status codes of the simulation (SIM_ERROR_* in sim.h)
ERRORS = {
    1: "Required step size is less than spacing between numbers.",
    2: "A state is not finite.",
    3: "The sampling period must be positive.",
    4: "Out of memory for the results.",
    5: "Too many switching intervals in a sampling period.",
    6: "A reference signal is not finite.",
    7: "Diode transitions stuck at the same instant.",
}


def array(values: Any) -> Any:
    """Array of doubles for ctypes (None becomes NaN)."""
    values = np.asarray(values, dtype=float).ravel()
    return (c_double * len(values))(*values)


def value(x: float | None) -> float:
    """Double for ctypes, None as NaN (the default values are resolved in C)."""
    return nan if x is None else float(x)


def c_args(values: dict[str, Any], fields: Sequence[str]) -> list[Any]:
    """
    Arguments of a function of a library from the parameter values of the fields
    (e.g., `_sm_control.CFG_FIELDS`), None as NaN and a list as its elements.
    """
    args: list[Any] = []
    for x in (values[name] for name in fields):
        args += [value(v) for v in x] if isinstance(x, list) else [x]
    return [nan if x is None else x for x in args]


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

    def set_enable(self, ctrl: Any) -> None:
        """Set the gate enable of the control system, see `enable_signal`."""
        self.set_signal(self.lib.enable_signal(self.handle), enable_signal(ctrl))

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

    def set_diode_bridge(self, front: GridConverterSystem) -> None:
        """Set the diode bridge feeding the DC bus, see `check_supported_front`."""
        f = cast(LFilter, front.ac_filter)
        src = cast(ThreePhaseSource, front.ac_source)
        L_t, R_t = f.L_f + f.L_g, f.R_f + f.R_g
        u_th = front.converter.u_th
        e_g, w_g = float(cast(float, src.e_g)), float(cast(float, src.w_g))
        self.lib.set_diode_bridge(self.handle, u_th, L_t, R_t, e_g, w_g)

    def set_converter(self, mdl: Model) -> None:
        """Set the converter, its DC bus, and its modulator from the model."""
        lib, h, converter = self.lib, self.handle, mdl.converter
        lib.set_converter(h, value(converter.inp.u_dc), converter.u_th)
        dc_bus = mdl.dc_bus
        if dc_bus is not None:
            lib.set_dc_bus(h, DC_BUS_TYPES[type(dc_bus)], dc_bus.C_dc)
        modulator = cast(ZOH | CarrierComparison, converter.modulator)
        N = modulator.N if isinstance(modulator, CarrierComparison) else 0
        delay, single_update = modulator.delay.length, modulator.single_update
        lib.set_modulator(h, N > 0, N, modulator.t_d, delay, single_update)

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
            n
            for s in mdl.subsystems.values()
            if s.state is not None
            for n in vars(s.state)
        ]
        if names != self.fields(control=False):
            raise NotImplementedError(
                f"The states {self.fields(control=False)} of the model expected"
            )
        x = np.asarray(mdl.get_initial_values(), dtype=complex)
        xy = array(np.column_stack((np.real(x), np.imag(x))))
        # The default maximum step (None) is resolved in C, see `Simulation`
        max_step = value(cfg.max_step)
        self.lib.start(
            self.handle, mdl.t0, xy, t_start, t_ctrl, max_step, cfg.rtol, cfg.atol
        )
        # The gate enable and the diode states of the converter (e.g., blocked by an
        # earlier simulation), and those of the diode bridge, NaN if not latched yet
        for converter in [c for c in mdl.converters if isinstance(c, DiodeBridge)]:
            diodes = [nan] * 3 if converter._diodes is None else converter._diodes
            if isinstance(converter, VoltageSourceConverter):
                enabled = int(converter.inp.enabled)
                self.lib.sim_set_converter_state(self.handle, enabled, array(diodes))
            else:
                self.lib.sim_set_bridge_state(self.handle, array(diodes))

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

    def converter_state(self) -> tuple[complex, complex, bool, np.ndarray | None]:
        """
        Switching state and current-direction vector of the converter, its gate
        enable, and its diode states with the gates blocked (None otherwise, as in
        `VoltageSourceConverter`).
        """
        q, diodes = (c_double * 4)(), (c_double * 3)()
        enabled = bool(self.lib.sim_converter_state(self.handle, q, diodes))
        latched = None if enabled else np.array(diodes)
        return complex(q[0], q[1]), complex(q[2], q[3]), enabled, latched

    def bridge_state(self) -> np.ndarray:
        """Diode states of the diode bridge feeding the DC bus."""
        diodes = (c_double * 3)()
        self.lib.sim_bridge_state(self.handle, diodes)
        return np.array(diodes)

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


def check_supported_converter(
    mdl: Model, front: GridConverterSystem | None = None
) -> None:
    """
    Raise an error if the converter, its DC bus (fed from the diode bridge `front` if
    given), or its modulator is not supported.
    """
    converter, dc_bus = mdl.converter, mdl.dc_bus
    if type(converter) is not VoltageSourceConverter or converter.sign is not np.sign:
        raise NotImplementedError(
            "Only VoltageSourceConverter with sign=np.sign supported"
        )
    if converter.u_dc_sensor is not None:
        raise NotImplementedError("Sensor models not supported")
    if dc_bus is None and converter.inp.u_dc is None:
        raise ValueError(
            "The DC-bus voltage must be given, unless the converter is fed from a "
            "DC-bus model"
        )
    if dc_bus is not None:
        check_supported_dc_bus(dc_bus, converter, front)
    if front is not None:
        check_supported_front(front)
    modulator = converter.modulator
    if type(modulator) not in (ZOH, CarrierComparison):
        raise NotImplementedError("Only the modulators ZOH and CarrierComparison")
    if modulator.delay.length > DELAY_MAX_LENGTH:
        raise NotImplementedError(f"At most the delay {DELAY_MAX_LENGTH} supported")
    # The delay buffer is filled at the first call of the modulator
    initial = not modulator.delay.data
    if isinstance(modulator, CarrierComparison):
        initial &= modulator._rising_edge and modulator._gate_cmd is None
    if not initial:
        raise NotImplementedError("Only the initial state of the modulator supported")


def enable_signal(ctrl: Any) -> StepSignal | float | Callable[[float], float]:
    """
    Gate enable of the control system (`ControlSystem.set_enable`) as a signal of the
    native system: a constant or a `StepSignal` (see `gate_enable`), or a function of
    the controller time, whose value is taken as a bool, as in motulator.
    """
    enable = ctrl._enable
    if callable(enable) and not isinstance(enable, StepSignal):
        return lambda t: float(bool(enable(t)))
    return gate_enable(ctrl)


def check_supported_control(ctrl: Any, d_err: bool) -> None:
    """
    Raise an error if the PWM of the control system is not supported (the duty-ratio
    error model only if `d_err`) or the gate enable is not supported (see
    `gate_enable`).
    """
    check_supported_pwm(ctrl, d_err=d_err, enable=False)
    pwm_values(ctrl.pwm)  # Raises if the duty-ratio error model is not supported
    enable_signal(ctrl)  # Raises if not supported
