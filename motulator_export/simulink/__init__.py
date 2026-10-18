"""
Export motulator systems to Simulink.

The control system has the blocks of the PLECS model: each class of motulator is a
block with a mask of the arguments of its class, running the C-Script code of the
PLECS model wrapped in a generated C MEX S-function or, with `code="matlab"`, the
MATLAB port of the same code in a MATLAB Function block, which needs no C compiler.
The modules `sm`, `im`, and `grid` export synchronous machine drives (flux-vector
control), induction machine drives (current-vector control and observer-based V/Hz
control), and grid converter systems (grid-following and grid-forming control). Each
provides `write_model` and `simulate`.

"""

from motulator_export.simulink import grid, im, sm

__all__ = ["grid", "im", "sm"]
