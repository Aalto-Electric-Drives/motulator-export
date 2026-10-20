# MATLAB port

MATLAB port of the *motulator* control algorithms used by the blocks of the control systems, run in the MATLAB Function blocks of the Simulink models (`code="matlab"` of the Simulink export, see [the Simulink export](../simulink/README.md)).
It is a port of the [C port](../c/README.md) function by function, so both follow the Python code of *motulator* in the same way, including the order of the state updates, and a change in *motulator* is mirrored in both.
Where the C port loops over arrays (e.g., the GradNets), the MATLAB port uses the vector operations of MATLAB, which give the results of the C port to the rounding errors.
The whole-system functions of the C port (e.g., `vector_control_system_compute_output`), which only the reference control systems of the tests use, are not ported, nor the functions that the blocks do not need (e.g., `matched_pole`, which the masks compute, and the PM-flux linkage of the GradNet machine, which the model takes from *motulator*); `tests/test_matlab_function.py` checks that the blocks use every function of the port.

Each file corresponds to a file of the C port and lists its Python counterparts.
The files are collections of functions: the generator `../simulink/_mfunction.py` copies the functions that a block uses (directly or via other functions) into its MATLAB function (`mfun_*.m` next to the model, called by the MATLAB Function block) as local functions, so that the file of the function is self-contained and the model does not need the files of the port.

| File                   | Contents                                                                                                                                                                                                                                                      |
| ---------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `common.m`             | `PIController`, `ComplexPIController`, `SpeedController`, `SpeedObserver`, `CurrentLimiter`, `PowerReferenceLimiter`, `RateLimiter`, `PWM` (with `InverterNonlinearity`), the discretization of the observers, root finding (`brentq`), and utility functions |
| `gradnet.m`            | GradNet inference (`FluxMap`, `CurrentMap`, and `CurrentMapWithHarmonics`)                                                                                                                                                                                    |
| `sm_parameters.m`      | `SynchronousMachinePars` and `SaturatedSynchronousMachinePars`                                                                                                                                                                                                |
| `sm_control_loci.m`    | `ControlLoci` (MTPA, MTPV, and current-limit loci) as the lookup tables of the `ReferenceGenerator`, computed by the function `sm_reference_luts.m` next to the model (see [the Simulink export](../simulink/README.md#matlab-function-blocks))               |
| `sm_flux_vector.m`     | `FluxObserver`, `ReferenceGenerator`, and `FluxTorqueController` for synchronous machines                                                                                                                                                                     |
| `im_current_vector.m`  | `FluxObserver`, `CurrentReferenceGenerator`, and `CurrentController` for induction machines                                                                                                                                                                   |
| `im_flux_vector.m`     | `ReferenceGenerator` and `FluxTorqueController` of observer-based V/Hz control                                                                                                                                                                                |
| `gfl_current_vector.m` | `PLL`, `CurrentController`, and the current and power references of grid-following control                                                                                                                                                                    |
| `gfm_observer.m`       | `Observer` and the current and voltage references of disturbance-observer-based grid-forming control                                                                                                                                                          |
| `sm_machine.m`         | `SynchronousMachine` with `SpatialSaturatedSynchronousMachinePars` (the GradNet machine model of the system model)                                                                                                                                            |

## Conventions

- A class is a struct with its functions, which return the modified struct, e.g., `self = pi_update(self, T_s, u)` for `pi_update(&self, T_s, u)` of the C port. A function that modifies its state and gives an output returns both, e.g., `[self, u] = pi_compute_output(self, y_ref, y, u_ff)`.
- Complex space vectors are complex numbers in double precision, with peak-value scaling, as in *motulator*. Since the code generation fixes the complexity of a variable at its first assignment, the complex variables and fields are complex from their initialization (e.g., `complex(0)`).
- Optional parameters (`None` in *motulator*) are `NaN`, as in the C port, and their defaults are resolved in the code. The struct `p` of the parameters of a block has `NaN` for an empty parameter, since a MATLAB Function block does not accept empty parameters.
- The sizes of the arrays are fixed by the code generation (e.g., the GradNets take their sizes from their weights), and the code avoids the functions not supported by it.
- The function files start with a function and end each function with `end` at the start of a line, and a function extends to the next one; the generator splits the files at the lines starting with `function`.

## Tests

`tests/test_matlab_function.py` builds each control system with MATLAB Function blocks alone in a Simulink model and runs it with the inputs of the closed-loop simulations of `tests/test_control.py` (also with the gates blocked, and a sequence of constant inputs with the gates blocked and enabled), comparing the outputs with those of the S-functions of the C port.
They agree to about 1e-14 relative to the signal magnitudes (often exactly for hundreds of steps).
In the vector control of the drives, the replayed inputs amplify the rounding errors exponentially: the observers see the voltages of the control system itself, but the recorded currents do not respond to them (in sensorless current-vector control, already from the start with zero flux).
The outputs are therefore compared within their sensitivity, which the S-functions show with the inputs perturbed at the level of the rounding errors, until the sensitivity of any output reaches 1e-6 relative to its magnitude.
An output that does not respond to the perturbation (e.g., the torque reference at its limit) does not show that the states still agree.
The differences stay below 1e-3 of the tolerances.
The test also checks that the lines of the built models connect the ports of the netlists (`tests/model_lines.m`), since the lines are drawn through the points of the layout.
The test needs MATLAB with the MATLAB Engine API for Python, so it is skipped in GitHub Actions; the C port is tested there against *motulator* step by step.
The example scripts with `--matlab` (e.g., `python examples/ipmsm_2kw_fvc_simulink.py --matlab`) compare the closed-loop simulations with *motulator*.
