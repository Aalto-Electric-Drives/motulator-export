"""
Export motulator systems to Simulink (pilot).

The control system runs as the C port of the motulator algorithms in a C MEX
S-function inside a masked subsystem, whose parameters follow the motulator API, as
in the PLECS models. The module `sm` exports synchronous machine drives with
flux-vector control.

"""

from motulator_plecs.simulink import sm

__all__ = ["sm"]
