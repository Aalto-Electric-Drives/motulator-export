"""
Export motulator systems to Simulink.

`write_model` writes the MATLAB script that builds a Simulink model of a system
built in motulator from the blocks of the library of motulator, with the MATLAB
classes of motulator (the package `+motulator` of this folder), and `simulate`
builds and runs the model via the MATLAB Engine API for Python.

"""

from motulator_export.simulink._export import simulate, write_model

__all__ = ["simulate", "write_model"]
