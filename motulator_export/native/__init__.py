"""
Native simulation of motulator systems.

A system built in motulator (a continuous-time system model and a discrete-time
control system) is simulated in C: the control system runs as the C port of the
motulator algorithms (`c`), and the system model as C models of its subsystems,
integrated by a C port of the simulation loop of motulator and the RK45 solver of
scipy (`c/sim.c`). `Simulation` replaces the simulation of motulator, with the same
interface and results. Each type of system has a module (e.g., `sm`) and a library
(e.g., `c/native_sm.c`), which is compiled with the C compiler at the first use.

"""

from motulator_export.native._simulation import Simulation

__all__ = ["Simulation"]
