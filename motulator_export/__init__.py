"""
Export motulator systems to PLECS and Simulink.

The system model and the control system of motulator are written into a PLECS
Standalone model (`plecs`) or a Simulink model (`simulink`). The control system
runs as a C port of the motulator algorithms (`c`) in a C-Script block or in a
generated C MEX S-function, inside a masked subsystem whose parameters follow the
motulator API. Both subpackages have the modules `sm`, `im`, and `grid` for
synchronous machine drives, induction machine drives, and grid converter systems,
each providing `write_model` and `simulate`.

"""

from motulator_export import plecs, simulink
from motulator_export.plecs._common import StepSignal, sampled_step

__all__ = ["StepSignal", "plecs", "sampled_step", "simulink"]
