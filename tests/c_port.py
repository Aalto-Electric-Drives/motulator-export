"""
Helpers of the C-port tests: compiling the C sources and converting the arguments.

Each test module defines its C test API (the functions called via ctypes) and
compiles it once per module with `compile_library` in a fixture:

    @pytest.fixture(scope="module")
    def dll(tmp_path_factory: pytest.TempPathFactory) -> ctypes.CDLL:
        return compile_library(CAPI, tmp_path_factory.mktemp("sm"))

"""

import ctypes
import subprocess
import sys
from pathlib import Path
from typing import Any

import numpy as np

from motulator_plecs._common import C_SOURCES

# gcc for a shared library. With MinGW-w64 on Windows, the runtime libraries are
# linked statically, since Python does not search PATH for the dependencies of a DLL.
GCC = ["gcc", "-std=c99", "-O2", "-shared", "-fPIC"]
if sys.platform == "win32":
    GCC.append("-static")


def compile_library(capi: str, out_dir: Path) -> ctypes.CDLL:
    """Compile the C sources with the test API `capi` into a shared library."""
    src = out_dir / "capi.c"
    src.write_text(capi)
    lib = out_dir / "libcapi.so"
    cmd = [*GCC, "-Wall"]
    cmd += ["-Wno-unused-function", f"-I{C_SOURCES}", str(src), "-lm", "-o", str(lib)]
    subprocess.run(cmd, check=True)
    return ctypes.CDLL(str(lib))


def arr(values: Any) -> Any:
    """Array of doubles for ctypes (None becomes NaN, as in the C port)."""
    values = np.asarray(values, dtype=float).ravel()
    return (ctypes.c_double * len(values))(*values)


def d(x: float) -> ctypes.c_double:
    """Double for ctypes."""
    return ctypes.c_double(x)
