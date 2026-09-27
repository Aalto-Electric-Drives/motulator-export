"""
Export motulator systems to PLECS.

The system model and the control system of motulator are written into a PLECS
Standalone model. The control system runs as a C port of the motulator algorithms in
a C-Script block inside a masked subsystem, whose parameters follow the motulator API.
The modules `sm`, `im`, and `grid` export synchronous machine drives, induction
machine drives, and grid converter systems. Each provides `write_model` and
`simulate`.

"""

from motulator_plecs import grid, im, sm
from motulator_plecs._common import StepSignal, sampled_step

__all__ = ["StepSignal", "grid", "im", "sampled_step", "sm"]
