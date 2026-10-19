"""
Export motulator systems to PLECS and Simulink, and simulate them natively.

The subpackages `plecs` and `simulink` write the models of a system built in
motulator, and `native` simulates the system in C. All of them run the same C port of
the control algorithms of motulator.

"""

from motulator_export import native, plecs, simulink
from motulator_export.plecs._common import StepSignal

__all__ = ["StepSignal", "native", "plecs", "simulink"]
