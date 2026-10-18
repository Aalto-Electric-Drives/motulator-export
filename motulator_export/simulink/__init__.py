"""
Export motulator systems to Simulink.

The system model and the control system of motulator are written into a Simulink
model by `write_model` (the MATLAB scripts building the model and the code of its
blocks), and `simulate` builds and runs the model via the MATLAB Engine API for
Python. The control system has the blocks of the PLECS model: each class of
motulator is a block with a mask of the arguments of its class, running the C-Script
code of the PLECS model wrapped in a generated C MEX S-function or, with
`code="matlab"`, the MATLAB port of the same code in a MATLAB Function block, which
needs no C compiler. The supported systems are synchronous machine drives
(flux-vector control), induction machine drives (current-vector control and
observer-based V/Hz control), and grid converter systems (grid-following and
grid-forming control).

"""

from motulator_export.simulink._export import simulate, write_model

__all__ = ["simulate", "write_model"]
