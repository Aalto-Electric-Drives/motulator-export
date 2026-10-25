"""
Library of the blocks of motulator for PLECS (`motulator_lib.plecs`).

The library has the blocks of the control systems of the PLECS models
(`CONTROL_BLOCKS`), whose masks refer to the structs of the arguments of their
top-level classes in the model initialization (e.g., `vector_ctrl`, `speed_ctrl`,
and `pwm`), and the synchronous machine with a GradNet current map, which refers to
the structs `machine` and `mechanics`. Its C-Script blocks include the C sources of
the package by a path relative to the library. The models written with
``library=True`` refer to it by Model References. The library is part of the
package, and `tests/test_library.py` checks that it is up to date: run
``python -m motulator_export.plecs._library`` after changing the blocks.

"""

from pathlib import Path

from motulator_export.plecs import (
    _grid_control,
    _im_control,
    _im_fvc_control,
    _im_vhz_control,
    _sm_control,
    _sm_cvc_control,
    _sm_vhz_control,
)
from motulator_export.plecs._common import LIBRARY, _control_subsystem, _model_text
from motulator_export.plecs._control import VALUES
from motulator_export.plecs._schematic import _Schematic
from motulator_export.plecs._sm import GRADNET_MACHINE, gradnet_machine

# Blocks of the control systems in the libraries of PLECS and Simulink
CONTROL_BLOCKS = [
    _sm_control.FVC_BLOCK,
    _sm_cvc_control.CVC_BLOCK,
    _im_control.CVC_BLOCK,
    _im_fvc_control.FVC_BLOCK,
    _im_vhz_control.VHZ_BLOCK,
    _sm_vhz_control.VHZ_BLOCK,
    _grid_control.GFL_BLOCK,
    _grid_control.GFM_BLOCK,
    _sm_control.FVC_TORQUE_BLOCK,
    _sm_cvc_control.CVC_TORQUE_BLOCK,
    _im_control.CVC_TORQUE_BLOCK,
    _im_fvc_control.FVC_TORQUE_BLOCK,
]


def library_text(path: Path = LIBRARY) -> str:
    """
    Text of the library file `path`: the control systems in the first column and
    the machine in the second. The settings of the model are not used.
    """
    sch = _Schematic()
    for k, block in enumerate(CONTROL_BLOCKS):
        header, trailer = _control_subsystem(block, VALUES)
        pos = (100, 60 + 110 * k)
        sch.component(
            "Subsystem", block.name, pos, direction="up", extra=header, trailer=trailer
        )
    header, trailer = gradnet_machine()
    sch.component(
        "Subsystem",
        GRADNET_MACHINE,
        (320, 60),
        direction="up",
        extra=header,
        trailer=trailer,
    )
    return _model_text(path, "", 1.0, 1e-4, sch, (440, 800), False)


def write_library(path: str | Path = LIBRARY) -> Path:
    """Write the library file (by default, that of the package)."""
    path = Path(path)
    path.write_text(library_text(path), encoding="utf-8")
    return path


if __name__ == "__main__":
    print(f"Wrote {write_library()}")
