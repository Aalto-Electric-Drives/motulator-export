"""
Export motulator systems to PLECS.

The system model and the control system of motulator are written into a PLECS
Standalone model. Each class of the control system is a block with a mask of the
arguments of its class, in the hierarchy of motulator, running a C port of the
motulator algorithms in a C-Script block. The modules `sm`, `im`, and `grid` export
synchronous machine drives (flux-vector control), induction machine drives
(current-vector control and observer-based V/Hz control), and grid converter systems
(grid-following and grid-forming control). Each provides `write_model` and
`simulate`.

"""

from motulator_export.plecs import grid, im, sm

__all__ = ["grid", "im", "sm"]
