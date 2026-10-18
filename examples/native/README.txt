Native simulation
=================

These examples simulate systems built in *motulator* natively, i.e., in C, with
:class:`motulator_export.native.Simulation`. The systems are built as in the examples
of *motulator*, and only the simulation object is replaced; the results are those of
*motulator* and can be plotted with its plotting functions. The control system runs
the C port of the control algorithms of *motulator*, which is also used by the PLECS
and Simulink models of this package.
