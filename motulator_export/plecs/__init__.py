"""
Export motulator systems to PLECS.

`write_model` writes a PLECS Standalone model of a system built in motulator, and
`simulate` runs the model via the RPC interface of PLECS Standalone.

"""

from motulator_export.plecs._export import simulate, write_model

__all__ = ["simulate", "write_model"]
