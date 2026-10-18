"""
Export motulator systems to PLECS and Simulink, and simulate them natively.

The system model and the control system of motulator are written into a PLECS
Standalone model (`plecs`) or a Simulink model (`simulink`). Each class of the
control system is a block with a mask of the arguments of its class, in the hierarchy
of motulator, running a C port of the motulator algorithms (`c`) in a C-Script block
(PLECS) or in a generated C MEX S-function (Simulink), or a MATLAB port of the same
code (`matlab`) in a MATLAB Function block (Simulink). Both subpackages provide
`write_model` and `simulate` for synchronous machine drives, induction machine
drives, and grid converter systems. The same C port, with C models of the systems,
simulates a motulator system natively (`native`), with the interface and the results
of the simulation of motulator.

"""

from motulator_export import native, plecs, simulink
from motulator_export.plecs._common import StepSignal

__all__ = ["StepSignal", "native", "plecs", "simulink"]
