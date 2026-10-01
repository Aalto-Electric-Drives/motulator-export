"""
Export motulator systems to Simulink.

The control system is the C-Script code of the PLECS model, wrapped in a generated
C MEX S-function inside a masked subsystem, whose parameters follow the motulator
API. The module `sm` exports synchronous machine drives with flux-vector control,
and `im` induction machine drives with current-vector control.

"""

from motulator_plecs.simulink import im, sm

__all__ = ["im", "sm"]
