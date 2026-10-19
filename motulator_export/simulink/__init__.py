"""
Export motulator systems to Simulink.

`write_model` writes the MATLAB scripts that build a Simulink model of a system built
in motulator, and `simulate` builds and runs the model via the MATLAB Engine API for
Python.

"""

from motulator_export.simulink._export import simulate, write_model

__all__ = ["simulate", "write_model"]
