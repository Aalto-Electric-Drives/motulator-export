"""
Export motulator systems to PLECS.

The system model and the control system of motulator are written into a PLECS
Standalone model by `write_model`, and `simulate` runs the model via the RPC
interface of PLECS Standalone. Each class of the control system is a block with a
mask of the arguments of its class, in the hierarchy of motulator, running a C port
of the motulator algorithms in a C-Script block. The supported systems are
synchronous machine drives (flux-vector control), induction machine drives
(current-vector control and observer-based V/Hz control), and grid converter systems
(grid-following and grid-forming control).

"""

from motulator_export.plecs._export import simulate, write_model

__all__ = ["simulate", "write_model"]
