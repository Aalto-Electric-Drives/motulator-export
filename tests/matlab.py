"""
Interpreter of the subset of MATLAB used in the model initializations and the mask
initializations of the control systems, see `motulator_export.simulink._matlab`.
"""

from motulator_export.simulink._matlab import (
    FUNCTIONS,
    Struct,
    block_params,
    evaluate,
    run,
)

__all__ = ["FUNCTIONS", "Struct", "block_params", "evaluate", "run"]
