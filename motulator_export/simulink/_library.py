"""
Data and code of the blocks of the Simulink library of motulator.

The library is built in MATLAB by `motulator.build_library` from the files of the
folder `library` of this package, which `write_library` writes: the data of the
blocks of the control systems (`control_blocks_c.m` and `control_blocks_matlab.m`:
the netlists of the control systems with their masks, as for `add_netlist.m`, and
their inputs and monitored signals), the S-functions (`sfun_*.c`) and the MATLAB
functions (`mfun_*.m`) of the blocks of the classes and of the GradNet machine, and
the functions called by the masks (`sm_reference_luts.m`). The blocks of the system
models are built by the MATLAB functions of the package `+motulator/+internal`.

The files are part of the package, so that the library can be built without
Python, and `tests/test_library.py` checks that they are up to date. Their version,
a hash of the inputs of the build (`_version`), is the description of the built
library, from which `motulator.internal.library` sees when to rebuild it.

"""

import hashlib
from pathlib import Path
from typing import Any

from motulator_export._common import C_SOURCES
from motulator_export.plecs._common import ControlBlock
from motulator_export.plecs._control import VALUES
from motulator_export.plecs._library import CONTROL_BLOCKS
from motulator_export.simulink._common import _m, control_fields
from motulator_export.simulink._mfunction import (
    gradnet_machine_mfunction,
    path_functions,
)
from motulator_export.simulink._render import CODES, mfunctions, sfunctions
from motulator_export.simulink._sfunction import gradnet_machine_sfunction

SIMULINK = Path(__file__).parent
LIBRARY = SIMULINK / "library"
VERSION = "%VERSION%"  # Placeholder of the version in the data


def _block_data(block: ControlBlock, code: str) -> str:
    """MATLAB code setting the fields of the struct c of the data of a block."""
    netlist = block.netlist()
    sfuns = sfunctions(netlist) if code == "c" else []
    fields: list[tuple[str, Any]] = control_fields(block, sfuns, VALUES, code)
    fields += [
        ("input_widths", block.input_widths),
        ("signals", block.signals),
        ("outputs", [[name, signals] for name, signals in block.outputs.items()]),
    ]
    return "c = struct();\n" + "".join(f"c.{n} = {_m(v)};\n" for n, v in fields)


def _data(code: str) -> str:
    """The function of the data of the blocks of the control systems."""
    name = f"control_blocks_{code}"
    blocks = "".join(
        f"\n% {b.name}\n" + _block_data(b, code) + f"blocks{{{k + 1}}} = c;\n"
        for k, b in enumerate(CONTROL_BLOCKS)
    )
    params = gradnet_machine_sfunction().params
    return (
        f"function [blocks, version, plant] = {name}()\n"
        f"%{name.upper()} Data of the blocks of the control systems of the library.\n"
        "%   [BLOCKS, VERSION, PLANT] = "
        f"{name.upper()}() gives the data of the blocks\n"
        "%   of the control systems (see motulator.internal.add_control_system), the\n"
        "%   version of the data, and the parameters of the S-function of the GradNet\n"
        "%   machine. Written by motulator_export.simulink._library from the\n"
        "%   netlists of the control systems; do not edit.\n"
        f"version = '{VERSION}';\n"
        f"plant.gradnet_params = {_m(params)};\n"
        f"blocks = cell(1, {len(CONTROL_BLOCKS)});\n" + blocks + "end\n"
    )


def library_files() -> dict[str, str]:
    """
    Files of the library by their names: the data of the blocks of the control
    systems with their version, and the code of the blocks.
    """
    files: dict[str, str] = {}
    for b in CONTROL_BLOCKS:
        netlist = b.netlist()
        for f in [*sfunctions(netlist), gradnet_machine_sfunction()]:
            files[f"{f.name}.c"] = f.source()
        for f in [*mfunctions(netlist), *path_functions(netlist)]:
            files[f"{f.name}.m"] = f.source
    f = gradnet_machine_mfunction()
    files[f"{f.name}.m"] = f.source
    data = {f"control_blocks_{code}.m": _data(code) for code in CODES}
    version = _version({**files, **data})
    return {n: s.replace(VERSION, version) for n, s in data.items()} | files


def _version(files: dict[str, str]) -> str:
    """
    Version of the inputs of the build of the library: a hash of the files, the
    MATLAB code that builds the library (`motulator.build_library` and the package
    `+motulator/+internal`), and the C port, which the S-functions include, with
    the newlines normalized.
    """
    sha = hashlib.sha256()
    package = SIMULINK / "+motulator"
    inputs = [package / "build_library.m", *(package / "+internal").glob("*.m")]
    inputs += [*C_SOURCES.glob("*.c"), *C_SOURCES.glob("*.h")]
    code = {
        p.relative_to(SIMULINK.parent).as_posix(): p.read_text(encoding="utf-8")
        for p in inputs
    }
    for name, text in sorted({**files, **code}.items()):
        sha.update(name.encode() + b"\0" + text.replace("\r\n", "\n").encode() + b"\0")
    return sha.hexdigest()[:16]


def write_library(folder: str | Path = LIBRARY) -> None:
    """Write the files of the library (see `library_files`) in the folder."""
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    for name, text in library_files().items():
        (folder / name).write_text(text, encoding="utf-8")


if __name__ == "__main__":
    write_library()
    print(f"Wrote {LIBRARY}")
