# C port

C port of the motulator control algorithms and models, run in the C-Script blocks of the PLECS models.
The code follows the Python code of motulator closely, including the order of the state updates, and each header lists its Python counterparts.
Optional parameters (`None` in motulator) are represented by `NAN`, and their defaults are resolved in the C code, as in motulator.

| File                   | Contents                                                                                                                                                           |
| ---------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `common.c`             | `PIController`, `ComplexPIController`, `SpeedController`, `SpeedObserver`, `PWM`, root finding (`brentq`), and utility functions                                   |
| `gradnet.c`            | GradNet inference (`FluxMap`, `CurrentMap`, and `CurrentMapWithHarmonics`)                                                                                         |
| `sm_parameters.c`      | `SynchronousMachinePars` and `SaturatedSynchronousMachinePars` (flux linkage, incremental inductances, and current iteration)                                      |
| `sm_control_loci.c`    | `ControlLoci` (MTPA, MTPV, and current-limit loci)                                                                                                                 |
| `sm_flux_vector.c`     | `FluxObserver`, `SpeedFluxObserver`, `ReferenceGenerator`, `FluxTorqueController`, `FluxVectorController`, and `VectorControlSystem` for synchronous machines      |
| `sm_machine.c`         | `SynchronousMachine` with `SpatialSaturatedSynchronousMachinePars` and `MechanicalSystem`                                                                          |
| `im_current_vector.c`  | `FluxObserver`, `SpeedFluxObserver`, `CurrentReferenceGenerator`, `CurrentController`, `CurrentVectorController`, and `VectorControlSystem` for induction machines |
| `gfl_current_vector.c` | Grid-following `CurrentVectorController` (PLL, current controller, and current limiter) and `GridConverterControlSystem`                                           |
| `gfm_observer.c`       | Disturbance-observer-based grid-forming control (`Observer` and `ObserverBasedGridFormingController`) and `GridConverterControlSystem`                             |

## Tests

The tests in `tests/` compile the C sources with gcc and compare the results with motulator step by step, without PLECS.
They run in GitHub Actions on every push and weekly against the main branch of motulator, to detect changes in motulator that must be mirrored here.

## Numerical precision of GradNets

motulator evaluates the GradNets in single precision (PyTorch), while the C port uses double precision.
The incremental inductances are computed from the analytic Jacobian of the flux map in both, so the differences remain at the level of the single-precision rounding errors (below 1e-5), and the tests compare the C port with motulator using the relative tolerance of 1e-4.
The sizes of the networks are limited by `GRADNET_MAX_IN_DIM` and `GRADNET_MAX_EMBED_DIM` in `gradnet.h`, which are checked both when a model is written and at the start of the simulation.

## Portability

The C port uses the C99 complex type (`complex.h`) in double precision, mirroring the complex space vectors of motulator.
It compiles with gcc, clang, and the compiler bundled with PLECS, but not with MSVC, and embedded compilers may lack `complex.h`.
A conversion to a custom complex type or to single precision is straightforward with the C-port tests, but it is left until a concrete embedded target exists.
