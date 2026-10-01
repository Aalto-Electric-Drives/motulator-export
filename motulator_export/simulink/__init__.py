"""
Export motulator systems to Simulink.

The control system is the C-Script code of the PLECS model, wrapped in a generated
C MEX S-function inside a masked subsystem, whose parameters follow the motulator
API. The module `sm` exports synchronous machine drives with flux-vector control,
`im` induction machine drives with current-vector control, and `grid` grid
converter systems with grid-following or grid-forming control.

"""

from motulator_export.simulink import grid, im, sm

__all__ = ["grid", "im", "sm"]
