"""
Test that the signatures of the functions of the native libraries, declared for
ctypes in `motulator_export.native`, match their definitions in C.

A wrong number or type of the arguments would pass garbage to the C function
without an error. The definitions are read from the C sources of each library
(`c/native_<name>.c` and the sources it includes), so neither gcc nor MATLAB is
needed.

Run from the repository root:

    pytest tests/test_native_api.py

"""

# %%
import ctypes
import re
from typing import Any

import pytest

from motulator_export._common import C_SOURCES
from motulator_export.native import _drive, _grid, _im, _sm, _system
from motulator_export.native._build import Signatures

LIBRARIES: dict[str, Signatures] = {
    "native_sm": {**_system.API, **_drive.DRIVE_API, **_sm.API},
    "native_im": {**_system.API, **_drive.DRIVE_API, **_im.API},
    "native_gfl": {**_system.API, **_grid.GRID_API, **_grid.GFL_API},
    "native_gfm": {**_system.API, **_grid.GRID_API, **_grid.GFM_API},
}

# Definitions of the functions with external linkage: a return type and a name at
# the start of a line, and the parameters up to the opening brace
DEFINITION = re.compile(
    r"^(?!static\b|typedef\b)([A-Za-z_][\w ]*?\**)\s*\b(\w+)\(([^)]*)\)\s*\{", re.M
)


def sources(name: str) -> list[str]:
    """Texts of the C source of the library and of the sources it includes."""
    text = (C_SOURCES / f"{name}.c").read_text(encoding="utf-8")
    included = re.findall(r'^#include "(\w+\.c)"', text, re.M)
    return [text, *((C_SOURCES / f).read_text(encoding="utf-8") for f in included)]


def ctype(decl: str) -> Any:
    """Type of ctypes of a C type (e.g., "const double *" or "System *")."""
    decl = re.sub(r"\bconst\b", "", decl).replace(" ", "")
    types = {
        "void": None,
        "double": ctypes.c_double,
        "int": ctypes.c_int,
        "double*": _system.c_double_p,
        "char*": ctypes.c_char_p,
        "SignalFunction": _system.SignalFunction,
    }
    if decl in types:
        return types[decl]
    if decl.endswith("*"):  # An opaque pointer, e.g., to the system
        return ctypes.c_void_p
    raise ValueError(f"Unknown C type {decl}")


def definitions(name: str) -> dict[str, tuple[Any, list[Any]]]:
    """Return types and parameter types of the functions of the library."""
    out = {}
    for text in sources(name):
        for m in DEFINITION.finditer(text):
            params = [p.strip() for p in m[3].split(",") if p.strip() != "void"]
            # The type of a parameter without its name (and without [] of an array)
            types = [re.sub(r"\b\w+(\[\w*\])?$", "", p).strip() for p in params]
            types = [
                t + " *" if "[" in p else t for t, p in zip(types, params, strict=True)
            ]
            out[m[2]] = (ctype(m[1]), [ctype(t) for t in types])
    return out


# %%
@pytest.mark.parametrize("name", list(LIBRARIES))
def test_signatures(name: str) -> None:
    """The declared signatures should match the definitions of the library."""
    defined = definitions(name)
    for function, (restype, argtypes) in LIBRARIES[name].items():
        assert function in defined, function
        assert defined[function] == (restype, list(argtypes)), function
