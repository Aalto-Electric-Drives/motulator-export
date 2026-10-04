# Simulink export

Export of *motulator* systems to Simulink, corresponding to the PLECS export.
The control system runs the same C code as in the PLECS model: the code sections of the C-Script block (the C port of the *motulator* control algorithms in `../c`) are wrapped in a generated C MEX S-function, in a masked subsystem whose parameters follow the *motulator* API.
The system model is built from basic Simulink blocks, so no toolboxes are needed.

| Module | System                                                                                                                                                    | Example                                                                                                                                              |
| ------ | --------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------- |
| `sm`   | Synchronous machine drive (`SynchronousMachinePars`, or a GradNet current map with spatial harmonics), flux-vector control (also with a GradNet flux map) | [ipmsm_2kw_fvc_simulink.py](../../examples/ipmsm_2kw_fvc_simulink.py), [pmsyrm_6kw_gn_fvc_simulink.py](../../examples/pmsyrm_6kw_gn_fvc_simulink.py) |
| `im`   | Induction machine drive (constant parameters), current-vector control                                                                                     | [im_2kw_cvc_simulink.py](../../examples/im_2kw_cvc_simulink.py)                                                                                      |
| `grid` | Grid converter system (LCL filter or L filter and grid inductance), grid-following or grid-forming control                                                | [gfl_10kva_lcl_simulink.py](../../examples/gfl_10kva_lcl_simulink.py), [gfm_13kva_do_simulink.py](../../examples/gfm_13kva_do_simulink.py)           |

## Usage

```bash
python examples/ipmsm_2kw_fvc_simulink.py
```

This writes the MATLAB script `examples/simulink/build_ipmsm_2kw_fvc.m` and the source of the S-function (`sfun_flux_vector_control.c`) in the same folder.
Running the script in MATLAB compiles the S-function (`mex`) and saves `ipmsm_2kw_fvc.slx`.
If the MATLAB Engine API for Python matching the MATLAB release (e.g., `pip install matlabengine==26.1.*` for R2026a) is installed, the Python script also builds and simulates the model and prints the maximum differences from *motulator*, as the PLECS comparison scripts do.

The C port uses the C99 complex type (`complex.h`), so `mex` must use gcc (Linux), clang (macOS), or MinGW-w64 (Windows, the MATLAB Support for MinGW-w64 C/C++ Compiler add-on, selected with `mex -setup C`); MSVC does not support it.
A compiled S-function stays loaded in MATLAB after its model is closed, so run `clear mex` before rebuilding a model that was open in MATLAB.

## Files

| File                                     | Contents                                                                                                                                                             |
| ---------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `_sfunction.py`                          | Generator of the S-functions from the code sections of a C-Script block                                                                                              |
| `_common.py`                             | Writer of the build scripts (the struct `s` of the parameters, the control system, and the signals) and the simulation via the MATLAB Engine API                     |
| `_drive.py`, `sm.py`, `im.py`, `grid.py` | Writers of the drives and the grid converter systems: `write_model` writes the S-function and the build script; `simulate` builds and runs the model                 |
| `build_drive.m`                          | Builder of the drive models: the layout of the system and the machine subsystems                                                                                     |
| `build_grid.m`                           | Builder of the grid converter models: the layout of the system, the AC filter, and the grid                                                                          |
| `+blocks/`                               | MATLAB package of the common parts: the model settings, the compilation, the control-system subsystem, the PWM, the converter, the mechanics, and the layout helpers |

## S-function

The S-function defines the C-Script macros used in the code (`InputSignal`, `OutputSignal`, `ParamRealData`, `ParamDim`, `SetErrorMessage`, `ContState`, and `ContDeriv`) with the Simulink API and calls the code sections in its callbacks: the start function in `mdlStart`, the output function in `mdlOutputs`, and the update function in `mdlUpdate`.
It has the ports, the parameters, the sample time, and the continuous states of the C-Script block.
As in *motulator*, the outputs are computed from the measurements of the same sampling instant and the states are updated in `mdlUpdate`.
The state of the C code is in static variables, as in the C-Script block, so a model can contain one instance of the block.

## Model structure

- **Control system**: a masked subsystem with the S-function, with the mask parameters on tabs as in the PLECS model. A GradNet flux map is a struct in the model workspace (`est_flux_map`), whose fields the mask initialization passes to the S-function. The first output of the S-function is the duty ratios, and the others are the groups of the monitored signals, which are combined into the output `signals`.
- **Computational delay**: a Unit Delay block.
- **PWM**: a triangular carrier (Repeating Sequence, period `2*T_s`, starting from its maximum) compared with the duty ratios by a Relational Operator block. Simulink locates the switching instants by zero-crossing detection. This corresponds to `CarrierComparison` of *motulator* without the counter quantization.
- **Converter**: the ideal two-level converter with a stiff DC bus, `u_s_ab = u_dc*q_ab` (`VoltageSourceConverter`). The dead time of the converter is not modeled, but the control system of an induction machine drive can compensate for it (`PWM(d_err=...)` with `dead_time_error` and the current-direction function `np.sign` or `tanh(i/i_0)`), as in the PLECS export; this is intended for the control system running on the hardware. The other DC buses of the PLECS export (a capacitor or a diode bridge, `ipmsm_2kw_fvc.py --diode`) are not supported, since their switching circuits would need Simscape Electrical or the zero-crossing detection of the diode states.
- **AC filter and grid** (grid converters): the LCL filter, or the L filter with the grid inductance, modeled with space vectors in stationary coordinates in a MATLAB Function block, with the states in an Integrator block, as in `LCLFilter` and `LFilter` of *motulator*. The grid voltage `e_g*exp(1j*w_g*t)` is generated by Sine Wave blocks. Without the grid impedance, the PCC voltage measured by grid-following control is the grid voltage.
- **References**: Step blocks, or a sum of Step blocks for several steps, and Constant blocks.
- **Machine**: the flux linkages as the state (an Integrator block), and a MATLAB Function block with the currents, the voltage equations, and the torque. The synchronous machine is modeled in rotor coordinates, as `SynchronousMachine` of *motulator*. The induction machine is modeled with the inverse-Γ model in stator coordinates, which is equivalent to the Γ model of `InductionMachine` of *motulator*.
- **Mechanics**: integrators for the speed and the angle (`MechanicalSystem` without friction).
- **Solver**: `ode45` (Dormand–Prince) with the relative tolerance of 1e-6 and the maximum step `T_s`, as in PLECS. The parameters of the system model are in the model workspace (`machine`, `mechanics`, and `converter`).
- **Outputs**: the root-level output ports `mdl` (the drives: `i_a`, `i_b`, `i_c`, `w_M`, `theta_M`, `tau_M`; the grid converters: the converter currents and, with the LCL filter, the grid currents) and `ctrl` (the monitored signals) for the comparison, and a scope.
- **Layout**: the blocks are aligned with the ports they connect to, so that the lines are straight, and the feedback lines are drawn through given corners.

## Agreement with *motulator*

Maximum differences (Simulink − *motulator*) printed by the comparison scripts, with *motulator* 0.8.1 and MATLAB R2026a with the MinGW-w64 compiler add-on (gcc 14.2), at the level of the PLECS models (cf. [examples/README.md](../../examples/README.md)).

| Script                          | System model                               | Control system                                               |
| ------------------------------- | ------------------------------------------ | ------------------------------------------------------------ |
| `ipmsm_2kw_fvc_simulink.py`     | w_M 7.0e-6, tau_M 4.9e-6, i_s_ab 2.0e-6    | w_M 5.8e-6, tau_M 4.9e-6, tau_M_ref 4.8e-6, psi_s_ref 1.2e-8 |
| `im_2kw_cvc_simulink.py`        | w_M 8.2e-7, tau_M 2.6e-6, i_s_ab 9.1e-7    | w_M 4.3e-6, tau_M 2.8e-6, tau_M_ref 3.2e-6, psi_R 3.5e-8     |
| `pmsyrm_6kw_gn_fvc_simulink.py` | w_M 3.4e-4, tau_M 2.1e-3, i_s_ab 6.9e-4    |                                                              |
| `gfl_10kva_lcl_simulink.py`     | i_c_ab 3.1e-6, i_g_ab 2.2e-6, i_c_a 3.1e-6 | p_g 1.3e-3, q_g 7.4e-4, u_g 1.7e-13, w_g 2.3e-13             |
| `gfm_13kva_do_simulink.py`      | i_c_ab 9.4e-7, i_c_a 8.0e-7                | p_g 3.9e-4, q_g 4.0e-4, v_c 2.7e-6, theta_c 0                |

The simulation of the IPMSM drive takes about 1.5 s in the normal mode (about 100 000 solver steps, mostly at the switching instants located by the zero-crossing detection) and 0.6 s in the accelerator mode, plus about 1 s of initialization.
Most of the run time of a comparison script is the start of MATLAB, the build of the model, and the compilation of the MATLAB Function block.

In `pmsyrm_6kw_gn_fvc_simulink.py`, the differences are larger, since *motulator* evaluates the GradNets in single precision, as in the PLECS model.

In addition, `tests/test_simulink.py` generates the S-functions, compiles them with gcc against a mock of the Simulink API (`tests/simulink_mock`), and runs them in closed-loop simulations of *motulator* (the drives in the sensorless and sensored modes); the results agree with those of the control systems of *motulator* to about 1e-12 relative to the signal magnitudes. The S-functions with GradNets (flux-vector control with a flux map and the machine) agree with *motulator* to its single precision.
