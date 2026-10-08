"""Common parts of the Simulink export of machine drives (see `build_drive.m`)."""

from collections.abc import Sequence
from pathlib import Path
from typing import Any, cast

import numpy as np
from motulator.common.model._converter import VoltageSourceConverter
from motulator.common.model._pwm import CarrierComparison
from motulator.drive.model import Drive

from motulator_export.plecs._common import ControlBlock, StepSignal
from motulator_export.plecs._control import VALUES
from motulator_export.plecs._drive import MDL_OUTPUTS
from motulator_export.simulink._common import (
    _m_source,
    _m_steps,
    scope_indices,
    simulate,
    write_script,
)
from motulator_export.simulink._sfunction import SFunction


def check_supported_converter(mdl: Drive) -> None:
    """
    Raise an error if the converter is not supported.

    The dead time is not modeled in the system model, but its compensation in the
    control system is supported.

    """
    if type(mdl.converter) is not VoltageSourceConverter:
        raise NotImplementedError("Only VoltageSourceConverter supported")
    if cast(CarrierComparison, mdl.pwm).t_d != 0:
        raise NotImplementedError("Dead time not supported in the system model")


def write_drive_model(
    path: str | Path,
    block: ControlBlock,
    init_script: str,
    machine: str,
    w_M_ref: StepSignal,
    tau_L: StepSignal,
    t_stop: float,
    sfunctions: Sequence[SFunction] = (),
    enable: StepSignal | float = 1.0,
) -> Path:
    """
    Write the S-functions and the MATLAB script building the model of a drive.

    Parameters
    ----------
    path : str | Path
        Path of the model (.slx).
    block : ControlBlock
        Control-system block.
    init_script : str
        Script in the folder of the model defining the parameters in the base
        workspace (see `write_init` and `blocks.new_model`).
    machine : str
        "sm" (synchronous machine), "gn" (synchronous machine with a GradNet current
        map, see `gradnet_machine_sfunction`), or "im" (induction machine).
    w_M_ref : StepSignal
        Speed reference (mechanical rad/s).
    tau_L : StepSignal
        External load torque (Nm).
    t_stop : float
        Simulation stop time (s).
    sfunctions : Sequence[SFunction], optional
        S-function of the machine ("gn"), whose parameters are passed to the
        builder.
    enable : StepSignal | float, optional
        Input `enable` of the control system, defaults to 1 (enabled). While it is
        not positive, the duty ratios are 0.5 and the state of the control system is
        reset to its initial value.

    Returns
    -------
    Path
        Path of the script.

    """
    path = Path(path)
    signals = block.signals
    flux = [f"ctrl.{n}" for n in signals if n.startswith("psi")]
    speed, torque = list(block.outputs.values())[:2]
    scope = [
        ("Speed", [*(f"ctrl.{n}" for n in speed), "mdl.w_M"]),
        ("Torque", [*(f"ctrl.{n}" for n in torque), "mdl.tau_M"]),
        ("Current", ["mdl.i_a", "mdl.i_b", "mdl.i_c"]),
        ("Flux", flux),
    ]
    fields: list[tuple[str, Any]] = [
        ("init_script", init_script),
        ("machine", machine),
        ("enable", _m_source(enable)),
        ("w_M_ref", _m_steps(w_M_ref)),
        ("tau_L", _m_steps(tau_L)),
        ("scope", scope_indices(scope, MDL_OUTPUTS, signals)),
    ]
    if sfunctions:
        fields.append(("machine_params", sfunctions[0].params))
    return write_script(
        path, block, VALUES, fields, "build_drive", VALUES["T_s"], t_stop, sfunctions
    )


def simulate_drive(
    path: str | Path, t_eval: np.ndarray, block: ControlBlock, build: bool = True
) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
    """Simulate the Simulink model of a drive, see `_common.simulate`."""
    return simulate(path, t_eval, MDL_OUTPUTS, block.signals, build)
