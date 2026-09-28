# motulator-plecs

Export [motulator](https://github.com/Aalto-Electric-Drives/motulator) drive systems and grid converter systems to [PLECS](https://www.plexim.com/products/plecs) Standalone.
The control system is ported to C and runs in a C-Script block inside a masked subsystem.
The mask parameters are the same as in the motulator API (e.g., `SynchronousMachinePars` or `SaturatedSynchronousMachinePars`, `FluxVectorControllerCfg`, and `SpeedController`), including the defaults: an empty parameter (`[]`) corresponds to `None` in motulator.
The derived quantities, such as the gains and the lookup tables of the reference generator, are computed by the C code at the start of the simulation, as in motulator.
Hence, the PLECS model is self-contained: the parameters can be changed in the mask without Python.

The system model is built from PLECS blocks as far as possible.
The synchronous machine is either the PLECS permanent-magnet synchronous machine (for `SynchronousMachinePars`) or a subsystem that looks like it (for `SpatialSaturatedSynchronousMachinePars` with a GradNet current map with spatial harmonics): inside, a C-Script block computes the currents and the torque from the flux linkage, and they are injected with controlled current and torque sources.
The induction machine is the PLECS squirrel-cage induction machine, parametrized as the T model with zero rotor leakage inductance, which is identical to the inverse-Γ model of motulator.
The mechanics, the converter, the DC bus, the AC filters, the grid, and the computational delay are PLECS blocks.

## Installation

```bash
pip install git+https://github.com/mhinkkan/motulator-plecs
```

For development, clone the repository and install it in editable mode with the development tools, `pip install -e .[dev]`.
The package requires motulator 0.8.0 or later.

## Contents

- `motulator_plecs/`: the Python package, which reads the parameters from motulator objects, writes the PLECS models, and simulates them via the RPC interface
  - `sm.py`: synchronous machine drives with flux-vector control
  - `im.py`: induction machine drives with current-vector control
  - `grid.py`: grid converter systems with grid-following or grid-forming control
  - `_schematic.py`, `_common.py`, `_drive.py`, `_rpc.py`: the shared building blocks (the PLECS file format, the control-system block and the converter, the drive mechanics and scope, and the RPC simulation)
  - `c/`: C port of the motulator models and control algorithms
    - `common.c`: `PIController`, `ComplexPIController`, `SpeedController`, `SpeedObserver`, `PWM`, root finding (`brentq`), and utility functions
    - `gradnet.c`: GradNet inference (`FluxMap`, `CurrentMap`, and `CurrentMapWithHarmonics`)
    - `sm_parameters.c`: `SynchronousMachinePars` and `SaturatedSynchronousMachinePars` (flux linkage, incremental inductances, and current iteration)
    - `sm_control_loci.c`: `ControlLoci` (MTPA, MTPV, and current-limit loci)
    - `sm_flux_vector.c`: `FluxObserver`, `SpeedFluxObserver`, `ReferenceGenerator`, `FluxTorqueController`, `FluxVectorController`, and `VectorControlSystem` for synchronous machines
    - `sm_machine.c`: `SynchronousMachine` with `SpatialSaturatedSynchronousMachinePars` and `MechanicalSystem`
    - `im_current_vector.c`: `FluxObserver`, `SpeedFluxObserver`, `CurrentReferenceGenerator`, `CurrentController`, `CurrentVectorController`, and `VectorControlSystem` for induction machines
    - `gfl_current_vector.c`: grid-following `CurrentVectorController` (PLL, current controller, and current limiter) and `GridConverterControlSystem`
    - `gfm_observer.c`: disturbance-observer-based grid-forming control (`Observer` and `ObserverBasedGridFormingController`) and `GridConverterControlSystem`
- `examples/`: comparison scripts and the generated PLECS models
  - `ipmsm_2kw_fvc.py`: 2.2-kW IPMSM, sensorless FVC (the README example of motulator), with `--diode` using a diode bridge with a DC-bus inductor and capacitor
  - `pmsyrm_6kw_gn_fvc.py`: 5.6-kW PM-SyRM, GradNet models from FEM data, sensored FVC (the motulator example `plot_6kw_pmsyrm_gn_fvc_fem_harm.py`); the trained GradNets are in `trained_models/`
  - `im_2kw_cvc.py`: 2.2-kW IM, sensorless CVC (the motulator example `plot_2kw_im_sat_cvc.py` with the constant-parameter machine model)
  - `gfl_10kva_lcl.py`: 10-kVA grid converter with an LCL filter, grid-following control (the motulator example `plot_10kva_lcl_gfl.py`)
  - `gfm_13kva_do.py`: 12.5-kVA grid converter with an L filter in a weak grid, disturbance-observer-based grid-forming control (the motulator example `plot_13kva_do_gfm.py`)
- `tests/`: tests comparing the C port with motulator without PLECS (require gcc); run in GitHub Actions on every push and weekly against the main branch of motulator

## Usage

```python
from motulator_plecs import sm

sm.write_model("ipmsm.plecs", mdl, ctrl, w_M_ref, tau_L, t_stop, speed_ctrl_args)
```

The modules `sm`, `im`, and `grid` provide `write_model` and `simulate`, see the examples.
To run the comparisons, start PLECS Standalone and enable the RPC interface (Preferences > General > RPC interface, port 1080).
Then, run from the repository root:

```bash
python examples/ipmsm_2kw_fvc.py
python examples/ipmsm_2kw_fvc.py --diode
python examples/pmsyrm_6kw_gn_fvc.py
python examples/im_2kw_cvc.py
python examples/gfl_10kva_lcl.py
python examples/gfm_13kva_do.py
pytest
```

The example scripts write the PLECS models, simulate the systems in both motulator and PLECS, and print the maximum differences.
For the comparison, the scripts write a temporary copy of the model (`*_tmp.plecs`, removed at exit) with output ports, through which the RPC interface returns the signals; the models in `examples/` have no output ports.
The scripts close the model in PLECS before simulating it, since an open model is not reloaded from the file.
The models can also be opened and simulated directly in PLECS.
The C-Script blocks include the C files of the package via a path relative to the model file, so a model must be regenerated if it is moved relative to the package.

## Model structure

- The references (e.g., the speed reference or the power references) and the load torque are Step blocks; several steps are given as vector parameters of one Step block and summed by a Gain block. A constant reference (the converter voltage reference of grid-forming control) is a Constant block.
- The control system samples the references and the measurements (e.g., the phase currents, the DC-bus voltage from a voltmeter, and the rotor angle or speed in the sensored mode) with the sampling period `T_s`.
  The duty ratios pass through a Delay block, which models the computational delay of one sampling period, as in motulator.
- The Symmetrical PWM block of PLECS (regular sampling with double update, carrier frequency `1/(2*T_s)`) generates the gate signals of the ideal two-level converter of PLECS. This corresponds to carrier comparison in motulator (`pwm=True`), except that the counter quantization of the duty ratios is not modeled: the comparison scripts use a fine quantization in motulator (`CarrierComparison(N=2**24)`), with which the models agree with motulator to about 1e-5 (with the default quantization of 4096 levels, the differences are about 1e-2).
- The DC bus is a DC voltage source, a capacitor, or a diode bridge fed by the grid with a DC-bus inductor and capacitor (`FrequencyConverter`, zero grid inductance).
  The circuits are not grounded: as in the space-vector models of motulator, there is no zero-sequence path (grounding both the DC bus and the grid neutral would let the common-mode voltage of the converter drive a zero-sequence current).
- The AC filter of the grid converter is either an LCL filter or an L filter with its series resistance and the grid inductance (`LFilter`), with the grid as three AC voltage sources.
- Voltmeters and ammeters are used only for the signals fed back to the control system. The AC voltages are measured line to line (u_ab and u_bc), as in practice. The other signals, e.g., the inductor currents shown in the scope, are measured with PLECS probes.
- The parameters of the control system are in the mask of the control-system subsystem, named after the control method, e.g., `Flux-vector control` (double-click the block), on tabs corresponding to the motulator API, e.g., the machine model, `FluxVectorControllerCfg`, and `SpeedController`. A GradNet flux map is given as a struct in the model workspace (`est_flux_map`). The controller signals are available as probe signals of the subsystem.
- The parameters of the system model are in the initialization commands of the model (Simulation > Simulation Parameters > Initialization): e.g., `machine`, `mechanics`, and `converter`, or `converter`, `ac_filter`, and `ac_source`.

## Numerical precision of GradNets

motulator evaluates the GradNets in single precision (PyTorch), while the C port uses double precision.
The incremental inductances are computed from the analytic Jacobian of the flux map in both, so the differences remain at the level of the single-precision rounding errors (below 1e-5), and the tests compare the C port with motulator using the relative tolerance of 1e-4.

## Limitations

Currently supported drive systems: synchronous machines (constant inductances or GradNet models) with flux-vector control with offline reference generation, induction machines with constant parameters (`InductionMachineInvGammaPars`) with current-vector control with the default observer gain, the sensorless or sensored mode, speed-control mode, the carrier-comparison converter model (`pwm=True`), and a stiff, capacitive (without an external DC current), or diode-bridge-fed DC bus.
Grid converter systems: grid-following current-vector control or disturbance-observer-based grid-forming control in the power-control mode, an LCL filter without resistances and grid impedance or an L filter with its series resistance and the grid inductance (without the grid resistance), a balanced grid voltage with constant magnitude and frequency, and a stiff DC bus.
Other configurations raise `NotImplementedError` in the writer modules.

The C port uses the C99 complex type (`complex.h`) in double precision, mirroring the complex space vectors of motulator. It compiles with gcc, clang, and the compiler bundled with PLECS, but not with MSVC, and embedded compilers may lack `complex.h`. A conversion to a custom complex type or to single precision is straightforward with the C-port tests, but it is left until a concrete embedded target exists.
