"""
Parts shared by the PLECS and Simulink exports and the native simulation: the step
signals, the references and the gate enable read from the motulator objects, the
type of the system, the checks of the PWM and the DC bus, and the C sources.

"""

import re
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from motulator.common.control._pwm import PWM
from motulator.common.model._converter import (
    DCBus,
    DiodeBridgeDCBus,
    VoltageSourceConverter,
)
from motulator.drive.control._base import DriveControlSystem
from motulator.drive.model import Drive, InductionMachine, SynchronousMachine
from motulator.grid.control._base import GridConverterControlSystem
from motulator.grid.model import GridConverterSystem

# Directory of the C sources (the C port, the C models, and the native libraries)
C_SOURCES = Path(__file__).parent / "c"


def c_define(header: str, name: str) -> int:
    """Value of an integer #define in a C header of `C_SOURCES`."""
    match = re.search(
        rf"^#define {name} (\d+)$", (C_SOURCES / header).read_text(), re.M
    )
    if match is None:
        raise RuntimeError(f"{name} not found in {header}")
    return int(match.group(1))


# %%
@dataclass
class StepSignal:
    """
    Step signal: `before` at first and `after` at `t > time`.

    Several steps are given as sequences: `after[k]` is the value after the step at
    `time[k]`, the times being increasing. The signal is callable, as the reference
    functions of motulator.

    """

    time: float | Sequence[float]
    after: float | Sequence[float]
    before: float = 0.0

    def __post_init__(self) -> None:
        times, afters = np.atleast_1d(self.time), np.atleast_1d(self.after)
        if times.ndim != 1 or times.size == 0 or times.shape != afters.shape:
            raise ValueError("time and after must have the same number of steps")
        if not np.all(np.isfinite([*times, self.before, *afters])):
            raise ValueError("The step times and the levels must be finite")
        if np.any(np.diff(times) < 0):
            raise ValueError("The step times must be increasing")

    def __call__(self, t: Any) -> Any:
        """Value at the time `t` (a float or an array)."""
        levels = [self.before, *np.atleast_1d(self.after)]
        times = np.atleast_1d(self.time)
        return self.before + sum(
            (t > t_k) * (levels[k + 1] - levels[k]) for k, t_k in enumerate(times)
        )


def sampled_step(sig: StepSignal, T_s: float) -> StepSignal:
    """
    Step signal that switches at the same sample as a reference of motulator.

    The controller of motulator evaluates the reference `(t > sig.time)` at its time
    `t`, which is accumulated by adding the sampling period. Due to rounding, the
    switching sample may differ from that of a Step block at `sig.time`. This
    returns a step switching halfway between the samples, as in motulator.

    """
    if not (np.isfinite(T_s) and T_s > 0):
        raise ValueError("The sampling period T_s must be positive and finite")
    times = []
    for time in np.atleast_1d(sig.time):
        if not (np.isfinite(time) and time < 1e9):
            raise ValueError("The step times must be finite and below 1e9 s")
        t, k = 0.0, 0
        while not t > time:
            t = (t + T_s) % 1e9  # As in ControlSystem.update of motulator
            k += 1
        times.append((k - 0.5) * T_s)
    time = times[0] if np.isscalar(sig.time) else times
    return StepSignal(time=time, after=sig.after, before=sig.before)


def reference(x: Any, name: str, T_s: float | None = None) -> StepSignal | float:
    """
    Reference signal (or load torque) of the exported model, read from the motulator
    object: a constant or a `StepSignal`, which the model has as a Constant or a Step
    block. A reference of the control system, which evaluates it at its sampling
    instants, is sampled (`sampled_step`) to switch at the same sampling instant as
    in motulator if the sampling period `T_s` is given.
    """
    if x is None:
        raise ValueError(f"The {name} must be set")
    if isinstance(x, StepSignal):
        return sampled_step(x, T_s) if T_s is not None else x
    if isinstance(x, (int, float, np.integer, np.floating)):
        if not np.isfinite(x):
            raise ValueError(f"The {name} must be finite")
        return float(x)
    raise NotImplementedError(
        f"Only a constant {name} or a StepSignal supported (the model has a Constant "
        "or a Step block)"
    )


def gate_enable(
    ctrl: DriveControlSystem | GridConverterControlSystem, T_s: float | None = None
) -> StepSignal | float:
    """
    Get the gate enable of the control system (`ControlSystem.set_enable` of
    motulator), the input enable of the exported control system.

    Only a constant gate enable (True or False, 1 or 0) or a `StepSignal` with the
    levels 0 and 1 is supported. The control system evaluates the gate enable at its
    sampling instants, so a step is sampled (`sampled_step`) to switch at the same
    sampling instant as in motulator, if the sampling period `T_s` is given.

    """
    enable = ctrl._enable
    if isinstance(enable, StepSignal):
        levels = [enable.before, *np.atleast_1d(enable.after)]
        if any(v not in (0, 1) for v in levels):
            raise ValueError("The levels of the gate enable must be 0 or 1")
        return sampled_step(enable, T_s) if T_s is not None else enable
    if isinstance(enable, (bool, np.bool_, int, float)) and enable in (0, 1):
        return float(enable)
    raise NotImplementedError(
        "Only a constant gate enable (set_enable) or a StepSignal of 0 and 1 supported"
    )


# %%
def system_type(mdl: Any, ctrl: Any) -> str:
    """
    Type of the system: "sm" (a drive with a synchronous machine), "im" (a drive
    with an induction machine), or "grid" (a grid converter system), with the control
    system of its kind.
    """
    if isinstance(mdl, GridConverterSystem):
        if not isinstance(ctrl, GridConverterControlSystem):
            raise NotImplementedError(
                "Only GridConverterControlSystem supported for a grid converter system"
            )
        return "grid"
    if isinstance(mdl, Drive):
        if not isinstance(ctrl, DriveControlSystem):
            raise NotImplementedError(
                "Only VectorControlSystem and VHzControlSystem supported for a drive"
            )
        if isinstance(mdl.machine, SynchronousMachine):
            return "sm"
        if isinstance(mdl.machine, InductionMachine):
            return "im"
    raise NotImplementedError(
        "Only machine drives (Drive) with a synchronous or an induction machine and "
        "grid converter systems (GridConverterSystem) supported"
    )


def check_supported_pwm(
    ctrl: DriveControlSystem | GridConverterControlSystem, d_err: bool = False
) -> None:
    """
    Raise an error if the PWM, the angle-advance factor `k_comp` of the voltage
    reference (given by the controller), or the gate enable of the control system is
    not supported (see `gate_enable`).

    The duty-ratio error model `d_err` is supported only if `d_err` is True.

    """
    pwm = ctrl.pwm
    if not isinstance(pwm, PWM) or pwm.overmodulation not in ("MPE", "MME"):
        raise NotImplementedError("Only the MPE and MME overmodulation supported")
    if ctrl.k_comp != 1.5:
        raise NotImplementedError("Only k_comp = 1.5 of the controller supported")
    if pwm.k_pred != 1.5:
        raise NotImplementedError("Only k_pred = 1.5 supported")
    if pwm.d_err is not None and not d_err:
        raise NotImplementedError("Duty-ratio error model d_err not supported")
    gate_enable(ctrl)  # Raises if not supported


def check_supported_dc_bus(dc_bus: DCBus, converter: VoltageSourceConverter) -> None:
    """
    Raise an error if the DC bus is not supported: it must feed only the converter of
    the drive (e.g., not an active front end), without an external DC current.
    """
    if type(dc_bus) not in (DCBus, DiodeBridgeDCBus):
        raise NotImplementedError("Only DCBus and DiodeBridgeDCBus supported")
    if dc_bus.converters != [converter]:
        raise NotImplementedError("DC bus shared with other converters not supported")
    if dc_bus.inp.i_dc is not None:
        raise NotImplementedError("External DC current not supported")
