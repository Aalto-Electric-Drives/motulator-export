"""
Compilation of the native libraries.

A library is compiled from its translation unit `motulator_export/c/<name>.c`, which
includes the C sources it needs, with the C compiler given by the environment
variable `CC` (by default gcc). The library is cached in the temporary directory
under a name containing a hash of the C sources and the compiler command, so that
it is compiled once per change of the sources.

"""

import ctypes
import hashlib
import os
import subprocess
import sys
import tempfile
import threading
from collections.abc import Mapping, Sequence
from functools import cache
from pathlib import Path
from typing import Any

C_SOURCES = Path(__file__).parents[1] / "c"

# Flags for a shared library. With MinGW-w64 on Windows, the runtime libraries are
# linked statically, since Python does not search PATH for the dependencies of a DLL.
FLAGS = ["-std=c99", "-O2", "-shared", "-fPIC"]
if sys.platform == "win32":
    FLAGS.append("-static")

# Signatures of the functions: name -> (result type, argument types)
Signatures = Mapping[str, tuple[Any, Sequence[Any]]]

# The libraries are compiled and loaded by one thread at a time
_lock = threading.Lock()


def load_library(name: str) -> ctypes.CDLL:
    """Compile the library `name` if needed and load it."""
    with _lock:
        return _load_library(name)


@cache
def _load_library(name: str) -> ctypes.CDLL:
    """Compile the library `name` if needed and load it."""
    compiler = os.environ.get("CC", "gcc")
    source = C_SOURCES / f"{name}.c"
    digest = hashlib.sha256(" ".join([compiler, *FLAGS]).encode())
    for path in sorted(C_SOURCES.glob("*.[ch]")):
        digest.update(path.name.encode() + path.read_bytes())
    suffix = ".dll" if sys.platform == "win32" else ".so"
    cache_dir = Path(tempfile.gettempdir()) / "motulator_export"
    library = cache_dir / f"{name}-{digest.hexdigest()[:16]}{suffix}"
    if not library.exists():
        cache_dir.mkdir(parents=True, exist_ok=True)
        # Compile into a temporary file first, so that concurrent processes do not
        # load a partially written library
        tmp = library.with_name(f"{library.stem}-{os.getpid()}{suffix}")
        cmd = [compiler, *FLAGS, str(source), "-lm", "-o", str(tmp)]
        try:
            subprocess.run(cmd, check=True)
        except FileNotFoundError:
            raise FileNotFoundError(
                f"C compiler {compiler!r} not found: install gcc on the PATH, or give "
                "the compiler in the environment variable CC"
            ) from None
        os.replace(tmp, library)
    return ctypes.CDLL(str(library))


def declare(lib: ctypes.CDLL, signatures: Signatures) -> None:
    """Declare the result and argument types of the functions of the library."""
    for name, (restype, argtypes) in signatures.items():
        fcn = getattr(lib, name)
        fcn.restype = restype
        fcn.argtypes = list(argtypes)
