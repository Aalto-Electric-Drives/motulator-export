"""
Test the files of the libraries and the MATLAB classes of motulator.

The PLECS library `motulator_export/plecs/motulator_lib.plecs` (written by
`plecs._library`) and the files of the folder `motulator_export/simulink/library`
(the data and the code of the blocks of the Simulink library, written by
`simulink._library`) are part of the package, so they must be those that the
netlists and the C and MATLAB ports give now, and the MATLAB classes of the package
`+motulator` must have the defaults of the classes of motulator. Run from the
repository root:

    pytest tests/test_library.py

After a change in the netlists, the ports, or the MATLAB classes, rewrite the files
with `python -m motulator_export.plecs._library` and
`python -m motulator_export.simulink._library`.

"""

import dataclasses
import inspect
import re
from pathlib import Path
from typing import Any

import motulator.drive.control.im as im_control
import motulator.drive.control.sm as sm_control
import motulator.grid.control as grid_control
import numpy as np
import pytest
from motulator.common.control._pwm import PWM, InverterNonlinearity
from motulator.common.utils import BaseValues, NominalValues
from motulator.drive import model
from motulator.drive.control._base import VectorControlSystem, VHzControlSystem
from motulator.grid import model as grid_model

from motulator_export.plecs._common import LIBRARY as PLECS_LIBRARY
from motulator_export.plecs._library import library_text
from motulator_export.simulink._library import LIBRARY, library_files
from tests.matlab import evaluate

PACKAGE = Path(__file__).parents[1] / "motulator_export" / "simulink" / "+motulator"


def test_plecs_library() -> None:
    """The PLECS library should be up to date."""
    on_disk = PLECS_LIBRARY.read_text(encoding="utf-8").replace("\r\n", "\n")
    assert on_disk == library_text(), "motulator_lib.plecs is out of date"


def test_library_files() -> None:
    """The files of the Simulink library should be up to date."""
    files = library_files()
    written = {p.name for p in LIBRARY.iterdir() if p.suffix in (".c", ".m")}
    written.discard("slblocks.m")  # Written by motulator.build_library
    assert written == set(files)
    for name, text in files.items():
        on_disk = (LIBRARY / name).read_text(encoding="utf-8").replace("\r\n", "\n")
        assert on_disk == text, f"{name} is out of date"


def properties(path: Path) -> dict[str, str]:
    """The public properties of a MATLAB class and their defaults (expressions)."""
    text = path.read_text(encoding="utf-8")
    m = re.search(r"^    properties\n(.*?)^    end$", text, re.M | re.S)
    assert m is not None, path
    out: dict[str, str] = {}
    for line in m.group(1).splitlines():
        code = line.split("%", 1)[0].strip()
        if code:
            name, _, default = code.partition("=")
            out[name.strip()] = default.strip()
    return out


# The MATLAB classes by their files and the motulator classes they mirror
CLASSES = {
    "+common/+utils/NominalValues.m": NominalValues,
    "+common/+model/VoltageSourceConverter.m": model.VoltageSourceConverter,
    "+common/+control/PWM.m": PWM,
    "+common/+control/InverterNonlinearity.m": InverterNonlinearity,
    "+drive/+model/SynchronousMachinePars.m": model.SynchronousMachinePars,
    "+drive/+model/SaturatedSynchronousMachinePars.m": (
        model.SaturatedSynchronousMachinePars
    ),
    "+drive/+model/SpatialSaturatedSynchronousMachinePars.m": (
        model.SpatialSaturatedSynchronousMachinePars
    ),
    "+drive/+model/InductionMachinePars.m": model.InductionMachinePars,
    "+drive/+model/InductionMachineInvGammaPars.m": model.InductionMachineInvGammaPars,
    "+drive/+model/SynchronousMachine.m": model.SynchronousMachine,
    "+drive/+model/InductionMachine.m": model.InductionMachine,
    "+drive/+model/MechanicalSystem.m": model.MechanicalSystem,
    "+drive/+model/Drive.m": model.Drive,
    "+drive/+control/SpeedController.m": sm_control.SpeedController,
    "+drive/+control/VectorControlSystem.m": VectorControlSystem,
    "+drive/+control/VHzControlSystem.m": VHzControlSystem,
    "+drive/+control/+sm/FluxVectorControllerCfg.m": sm_control.FluxVectorControllerCfg,
    "+drive/+control/+sm/FluxVectorController.m": sm_control.FluxVectorController,
    "+drive/+control/+im/CurrentVectorControllerCfg.m": (
        im_control.CurrentVectorControllerCfg
    ),
    "+drive/+control/+im/CurrentVectorController.m": im_control.CurrentVectorController,
    "+drive/+control/+im/ObserverBasedVHzControllerCfg.m": (
        im_control.ObserverBasedVHzControllerCfg
    ),
    "+drive/+control/+im/ObserverBasedVHzController.m": (
        im_control.ObserverBasedVHzController
    ),
    "+grid/+model/LCLFilter.m": grid_model.LCLFilter,
    "+grid/+model/LFilter.m": grid_model.LFilter,
    "+grid/+model/ThreePhaseSource.m": grid_model.ThreePhaseSource,
    "+grid/+model/GridConverterSystem.m": grid_model.GridConverterSystem,
    "+grid/+control/CurrentVectorControllerCfg.m": (
        grid_control.CurrentVectorControllerCfg
    ),
    "+grid/+control/CurrentVectorController.m": grid_control.CurrentVectorController,
    "+grid/+control/ObserverBasedGridFormingControllerCfg.m": (
        grid_control.ObserverBasedGridFormingControllerCfg
    ),
    "+grid/+control/ObserverBasedGridFormingController.m": (
        grid_control.ObserverBasedGridFormingController
    ),
    "+grid/+control/GridConverterControlSystem.m": (
        grid_control.GridConverterControlSystem
    ),
}
# Properties of the MATLAB classes that are not arguments of motulator: the current
# scale of the sign function (motulator has the function), the state and the load
# torque of the mechanics, and the references
EXTRA = {"i_0", "state", "tau_L", "w_M_ref", "p_g_ref", "q_g_ref", "v_c_ref"}


def _defaults(cls: Any) -> dict[str, Any]:
    """The arguments of a motulator class and their defaults (empty if none)."""
    if dataclasses.is_dataclass(cls):
        return {f.name: f.default for f in dataclasses.fields(cls)}
    return {n: p.default for n, p in inspect.signature(cls).parameters.items()}


@pytest.mark.parametrize("path", list(CLASSES), ids=[Path(p).stem for p in CLASSES])
def test_defaults(path: str) -> None:
    """
    The properties of the MATLAB classes should be the arguments of the motulator
    classes, the supported ones, with their defaults, [] for None (and for the
    optional objects, e.g., the PWM, created in the constructor).
    """
    cls = CLASSES[path]
    props = properties(PACKAGE / path)
    args = _defaults(cls)
    for name, expr in props.items():
        if name in EXTRA:
            continue
        assert name in args, f"{name} is not an argument of {cls.__name__}"
        default = args[name]
        empty = (dataclasses.MISSING, inspect.Parameter.empty)
        if default in empty or not expr:
            assert not expr or expr == "[]", f"{name}: no default in motulator"
            continue
        value = evaluate(expr, {})
        if default is None:
            assert np.size(value) == 0, f"{name}: [] expected for None"
        elif isinstance(default, str):
            assert value == default, name
        else:
            assert value == default, f"{name}: {value} != {default}"


def test_base_values() -> None:
    """The base values of the MATLAB class should be those of motulator."""
    text = (PACKAGE / "+common" / "+utils" / "BaseValues.m").read_text()
    assert set(properties(PACKAGE / "+common" / "+utils" / "BaseValues.m")) == {
        f.name for f in dataclasses.fields(BaseValues)
    }
    assert "u = sqrt(2/3)*nom.U;" in text
