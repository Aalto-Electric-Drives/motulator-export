# Simulink export (pilot)

Pilot of exporting motulator systems to Simulink, with the 2.2-kW IPMSM drive with sensorless flux-vector control as the first case ([examples/ipmsm_2kw_fvc_simulink.py](../../examples/ipmsm_2kw_fvc_simulink.py)).
The approach is the same as in the PLECS models: the control system is the C port of the motulator control algorithms (`../c`, unchanged), and it runs in a masked subsystem whose parameters follow the motulator API.

| File             | Contents                                                                                                                                                                  |
| ---------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `sfun_sm_fvc.c`  | Level-2 C MEX S-function wrapping `VectorControlSystem` of the C port (flux-vector control and speed control)                                                             |
| `build_sm_fvc.m` | MATLAB function that compiles the S-function and builds the model with the Simulink API (`add_block`, `add_line`)                                                         |
| `sm.py`          | Python writer: `write_model` writes a short MATLAB script with the parameters, which calls `build_sm_fvc`; `simulate` runs the model via the MATLAB Engine API for Python |

## Usage

```bash
python examples/ipmsm_2kw_fvc_simulink.py
```

This writes `examples/simulink/build_ipmsm_2kw_fvc.m`.
Running this script in MATLAB compiles the S-function (`mex`) and saves `ipmsm_2kw_fvc.slx` in the same folder.
If the MATLAB Engine API for Python (`pip install matlabengine`) is installed, the Python script also builds and simulates the model and prints the maximum differences from motulator, as the PLECS comparison scripts do.

The C port uses the C99 complex type (`complex.h`), so `mex` must use gcc (Linux), clang (macOS), or MinGW-w64 (Windows, the MATLAB Support for MinGW-w64 C/C++ Compiler add-on, selected with `mex -setup C`); MSVC does not support it.

## Model structure

- **Control system**: a masked subsystem `Flux-vector control` with the S-function, with the mask parameters on tabs as in the PLECS model (constant inductances only). The S-function runs at the sampling period `T_s`. It computes the outputs from the measurements of the same sampling instant and updates the states in `mdlUpdate`, as in motulator. Its second output contains the monitored signals (`w_M_ref`, `w_M`, `tau_M_ref`, `tau_M`, `psi_s_ref`, `psi_s`, `theta_m`, `i_d`, `i_q`).
- **Computational delay**: a Unit Delay block.
- **PWM**: a triangular carrier (Repeating Sequence, period `2*T_s`, starting from its maximum) compared with the duty ratios by a Relational Operator block. Simulink locates the switching instants by zero-crossing detection. This corresponds to `CarrierComparison` of motulator without the counter quantization.
- **Converter**: the ideal two-level converter with a stiff DC bus, `u_s_ab = u_dc*q_ab`.
- **Machine**: the stator flux linkage in rotor coordinates as the state (an Integrator block), and a MATLAB Function block with the current, the voltage equation, and the torque, as in `SynchronousMachine` of motulator.
- **Mechanics**: integrators for the speed and the angle (`MechanicalSystem` without friction).
- **Solver**: `ode45` (Dormand–Prince) with the relative tolerance of 1e-6 and the maximum step `T_s`, as in PLECS. The parameters of the system model are in the model workspace (`machine`, `mechanics`, and `converter`).
- **Outputs**: the root-level output ports `mdl` (`i_a`, `i_b`, `i_c`, `w_M`, `theta_M`, `tau_M`) and `ctrl` (the monitored signals) for the comparison, and a scope.

The system model uses basic Simulink blocks only, so no toolboxes are needed.
A natural next step is a variant with Simscape Electrical blocks (converter, machine, and mechanics), corresponding to the PLECS components.

## Verification status

The model has been built and simulated in MATLAB R2026a with the MinGW-w64 compiler add-on (gcc 14.2). Over the 1.2-s run, `python examples/ipmsm_2kw_fvc_simulink.py` gives the maximum differences from motulator w_M 7.0e-6, tau_M 4.9e-6, i_s_ab 2.0e-6, ctrl.w_M 5.8e-6, ctrl.tau_M_ref 4.8e-6, and ctrl.psi_s_ref 1.2e-8, which is the level of the PLECS model (cf. [examples/README.md](../../examples/README.md)).
The simulation itself takes about 1.5 s in the normal mode (about 100 000 solver steps, mostly at the switching instants located by the zero-crossing detection) and 0.6 s in the accelerator mode, plus about 1 s of initialization.
Most of the run time of the comparison script is the start of MATLAB, the build of the model, and the compilation of the MATLAB Function block.

In addition, `tests/test_simulink.py` compiles the S-function with gcc against a mock of the Simulink API (`tests/simulink_mock`) and compares its outputs with motulator step by step, in the sensorless and sensored modes (maximum differences about 1e-12). It also checks that the parameter order of the S-function matches the mask.
