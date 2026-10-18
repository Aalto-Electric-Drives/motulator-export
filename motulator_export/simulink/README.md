# Simulink export

Export of *motulator* systems to Simulink, corresponding to the PLECS export.
The control system runs the same C code as in the PLECS model: each class of *motulator* is a block with a mask of the arguments of its class, in the hierarchy of *motulator*, and the code sections of the C-Script blocks of the classes (the C port of the *motulator* control algorithms in `../c`) are wrapped in generated C MEX S-functions.
Alternatively (`code="matlab"` of `write_model`), the blocks of the classes are MATLAB Function blocks running the MATLAB port of the same code (`../matlab`), in the same hierarchy and with the same masks; these models need no C compiler, since Simulink compiles the MATLAB Function blocks itself, and they carry the code of their blocks in the model.
The system model is built from basic Simulink blocks, so no toolboxes are needed.

| Module | System                                                                                                                                                    | Example                                                                                                                                              |
| ------ | --------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------- |
| `sm`   | Synchronous machine drive (`SynchronousMachinePars`, or a GradNet current map with spatial harmonics), flux-vector control (also with a GradNet flux map) | [ipmsm_2kw_fvc_simulink.py](../../examples/ipmsm_2kw_fvc_simulink.py), [pmsyrm_6kw_gn_fvc_simulink.py](../../examples/pmsyrm_6kw_gn_fvc_simulink.py) |
| `im`   | Induction machine drive (constant parameters), current-vector control or observer-based V/Hz control (also open-loop V/Hz control)                        | [im_2kw_cvc_simulink.py](../../examples/im_2kw_cvc_simulink.py), [im_2kw_vhz_simulink.py](../../examples/im_2kw_vhz_simulink.py)                     |
| `grid` | Grid converter system (LCL filter or L filter and grid inductance), grid-following or grid-forming control                                                | [gfl_10kva_lcl_simulink.py](../../examples/gfl_10kva_lcl_simulink.py), [gfm_13kva_do_simulink.py](../../examples/gfm_13kva_do_simulink.py)           |

## Usage

```bash
python examples/ipmsm_2kw_fvc_simulink.py
```

This writes the MATLAB scripts `examples/simulink/build_ipmsm_2kw_fvc.m` and `init_ipmsm_2kw_fvc.m` and the sources of the S-functions of the blocks (`sfun_fvc_*.c`) in the same folder.
Running the build script in MATLAB compiles the S-functions (`mex`) and saves `ipmsm_2kw_fvc.slx`.
The parameters are defined by the script `init_ipmsm_2kw_fvc.m` in the base workspace, as in the *motulator* example (the system model and the structs `par`, `cfg`, `speed_ctrl`, and `pwm` of the control system); the model runs it after loading and at the start of each simulation, so a change in the script takes effect in the next simulation.
If the MATLAB Engine API for Python matching the MATLAB release (e.g., `pip install matlabengine==26.1.*` for R2026a) is installed, the Python script also builds and simulates the model and prints the maximum differences from *motulator*, as the PLECS comparison scripts do.

The C port uses the C99 complex type (`complex.h`), so `mex` must use gcc (Linux), clang (macOS), or MinGW-w64 (Windows, the MATLAB Support for MinGW-w64 C/C++ Compiler add-on, selected with `mex -setup C`); MSVC does not support it.
A compiled S-function stays loaded in MATLAB after its model is closed, so run `clear mex` before rebuilding a model that was open in MATLAB (on Windows, the compiled file cannot be overwritten while it is loaded).

With MATLAB Function blocks:

```bash
python examples/ipmsm_2kw_fvc_simulink.py --matlab
```

This writes `build_ipmsm_2kw_fvc_matlab.m` and `init_ipmsm_2kw_fvc_matlab.m` and the MATLAB functions of the blocks (`mfun_fvc_*.m`), and the build script saves `ipmsm_2kw_fvc_matlab.slx`, whose blocks contain the code of the functions, so the model does not need the files.
The models of the synchronous machine drives (with either code) also need the function `sm_reference_luts.m`, written next to the model, which the mask initialization of the `ReferenceGenerator` calls to compute the lookup tables of the loci.
In your own scripts, pass `code="matlab"` to `write_model` (e.g., `simulink.sm.write_model("ipmsm.slx", ..., code="matlab")`); `simulate` is the same.

## Files

| File                                     | Contents                                                                                                                                                                                         |
| ---------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `_sfunction.py`                          | Generator of the S-functions from the code sections of a C-Script block                                                                                                                          |
| `_mfunction.py`                          | Generator of the MATLAB functions of the MATLAB Function blocks from the MATLAB code of a block (`MCode`) and the MATLAB port, and of the masks collecting their parameters                      |
| `_render.py`                             | Control systems: the S-functions (or the MATLAB functions) of the blocks and the netlist rows (blocks, alignments, lines, and masks) for `blocks.add_netlist`                                    |
| `_common.py`                             | Writer of the build and initialization scripts (the struct `s` of the control system and the signals) and the simulation via the MATLAB Engine API                                               |
| `_drive.py`, `sm.py`, `im.py`, `grid.py` | Writers of the drives and the grid converter systems: `write_model` writes the S-functions (or the MATLAB functions) and the scripts; `simulate` builds and runs the model                       |
| `build_drive.m`                          | Builder of the drive models: the layout of the system and the machine subsystems                                                                                                                 |
| `build_grid.m`                           | Builder of the grid converter models: the layout of the system, the AC filter, and the grid                                                                                                      |
| `+blocks/`                               | MATLAB package of the common parts: the model settings, the compilation, the control-system subsystem, the MATLAB Function blocks, the PWM, the converter, the mechanics, and the layout helpers |

## S-function

The S-function defines the C-Script macros used in the code (`InputSignal`, `OutputSignal`, `ParamRealData`, `ParamDim`, `SetErrorMessage`, `ContState`, and `ContDeriv`) with the Simulink API and calls the code sections in its callbacks: the start function in `mdlStart`, the output function in `mdlOutputs`, and the update function in `mdlUpdate`.
It has the ports, the parameters, the sample time, and the continuous states of the C-Script block.
As in *motulator*, the outputs are computed from the measurements of the same sampling instant and the states are updated in `mdlUpdate`.
The state of the C code is in static variables, as in the C-Script block, so a model can contain one instance of the block.

The first input of every control system is `enable`, the gate enable of *motulator* (`ControlSystem.set_enable`), and the control systems have the input `u_c_line` of the line-to-line converter voltages u_ab and u_bc, used while the gates are blocked.
While `enable` is zero, the control system runs in standby, as in *motulator* (see [the README](../../README.md)).
The system model of Simulink does not block the gates, so in the exported models `enable` is 1 and `u_c_line` zero (`write_model` rejects a control system with the gates blocked); on the hardware, the gate enable also blocks the gates of the converter.
The control systems limit the measured DC-bus voltage to at least 1 V (`U_DC_MIN`), which avoids the division by zero in the PWM if the DC bus is not charged.

## MATLAB Function blocks

With `code="matlab"`, each block of a class runs a MATLAB function generated by `_mfunction.py` from the MATLAB code of the block (`MCode` in the netlists of `../plecs`, the counterpart of its C-Script code) and the MATLAB port (`../matlab`), whose functions that the block uses are appended as local functions:

- The function has the inputs and outputs of the block, the complex signals and the signal vectors (`meas`, `fbk`, and `ref`) converted to complex numbers and structs (e.g., `meas = measurements_unpack(meas_in)`).
- The state is in persistent variables: `self` is initialized from the parameters at the first call (the start function), the outputs are computed, and then the state is updated, as in the S-function. Each MATLAB Function block has its own state, so a model can contain several instances of a block.
- The parameters are the fields of the struct `p`, a nontunable parameter of the MATLAB Function block. The mask of the block (that of its class, or a mask of its parameters for the blocks without one) collects the parameters into `p` in its initialization, an empty parameter (`[]`, `None`) being `NaN`, since a MATLAB Function block does not accept empty parameters, and checks them (e.g., `J must be a scalar.`). The parameters are constants of the code generation, so the code for, e.g., the GradNet flux map is compiled only if the machine model has one.
- A MATLAB Function block has no start function, so a long initialization (the control loci of the `ReferenceGenerator` of synchronous machines, with 48 root findings) would run at the first step. Instead, the mask initialization of the block computes the lookup tables with the function `sm_reference_luts.m` written next to the model (`Mask.simulink_init` of the netlists, `PATH_FUNCTIONS` of `_mfunction.py`), and the S-function version of the block uses the same tables; in PLECS, the C-Script block computes them at its start.
- All inputs of a MATLAB Function block have direct feedthrough. The inputs that the S-functions use only in the update of the state (e.g., the limited torque reference of the speed controller, the duty ratios of the PWM, and the inputs of the speed observer), which would close algebraic loops, come through Unit Delay blocks, and the update with them is done at the start of the next call, which gives the same result. Such a block is a masked subsystem containing the MATLAB Function block (named by its function) and the Unit Delay blocks.
- Simulink compiles the MATLAB Function blocks just in time, so no C compiler is needed, and the functions are copied into the blocks by `blocks.add_mfunction`, so the model does not need the files `mfun_*.m`. Simulink resizes a MATLAB Function block when its code is set, so `blocks.add_mfunction` restores the size of the layout: the blocks are aligned and the lines drawn through the points of the layout, and a line starting at another port would join its line.

The synchronous machine with a GradNet current map is a MATLAB Function block too (`mfun_gradnet_machine`), with the stator flux linkage in an Integrator block, whose initial value is the PM-flux linkage of *motulator* (`machine.psi_f`).

## Model structure

- **Control system**: a masked subsystem containing the netlist of the blocks, built by `blocks.add_netlist`: the S-functions (or the MATLAB Function blocks) of the classes (masked, showing the name of the class and the port labels), the subsystems (masked for the composite classes), and the Selector, Mux, Goto, and From blocks, as in the PLECS model (see [examples/README.md](../../examples/README.md)). The layout is computed in Python (`plecs/_netlist.py`), and the blocks are aligned with the actual ports of Simulink. The unconnected outputs (e.g., the load torque estimate of the speed observer) are terminated. The S-functions get their parameters after the masks are created. A GradNet flux map is a struct of the initialization script (`est_flux_map`), whose fields the mask initialization passes to the S-functions. The monitored signals of the block `Monitored signals` are combined into the output `signals`.
- **Computational delay**: a Unit Delay block.
- **PWM**: a triangular carrier (Repeating Sequence, period `2*T_s`, starting from its maximum) compared with the duty ratios by a Relational Operator block. Simulink locates the switching instants by zero-crossing detection. This corresponds to `CarrierComparison` of *motulator* without the counter quantization.
- **Converter**: the ideal two-level converter with a stiff DC bus, `u_s_ab = u_dc*q_ab` (`VoltageSourceConverter`). The dead time and the threshold voltage of the converter are not modeled, and the duty ratios are updated twice per carrier period, but the control system of a drive can compensate for the dead time and the threshold voltage (`PWM(d_err=InverterNonlinearity(t_d, u_th, sign))` with the current-direction function `np.sign` or `tanh(i/i_0)`), also with single update (`PWM(single_update=True)`), as in the PLECS export; this is intended for the control system running on the hardware. The DC-bus models of the PLECS export (a capacitor or a diode bridge, `DCBus` and `DiodeBridgeDCBus`, `ipmsm_2kw_fvc.py --diode`) are not supported, since their switching circuits would need Simscape Electrical or the zero-crossing detection of the diode states.
- **AC filter and grid** (grid converters): the LCL filter, or the L filter with the grid inductance, modeled with space vectors in stationary coordinates in a MATLAB Function block, with the states in an Integrator block, as in `LCLFilter` and `LFilter` of *motulator*. The grid voltage `e_g*exp(1j*w_g*t)` is generated by Sine Wave blocks. Without the grid impedance, the PCC voltage measured by grid-following control is the grid voltage.
- **References and the inputs `enable` and `u_c_line`**: Step blocks, or a sum of Step blocks for several steps, and Constant blocks (`enable` 1 and `u_c_line` zero).
- **Machine**: the flux linkages as the state (an Integrator block), and a MATLAB Function block with the currents, the voltage equations, and the torque. The synchronous machine is modeled in rotor coordinates, as `SynchronousMachine` of *motulator*. The induction machine is modeled with the inverse-Γ model in stator coordinates, which is equivalent to the Γ model of `InductionMachine` of *motulator*.
- **Mechanics**: integrators for the speed and the angle (`MechanicalSystem` without friction), starting from the state of the mechanics of *motulator* (`mechanics.w_M0` and `mechanics.theta_M0`).
- **Solver**: `ode45` (Dormand–Prince) with the relative tolerance of 1e-6 and the maximum step `T_s`, as in PLECS.
- **Outputs**: the root-level output ports `mdl` (the drives: `i_a`, `i_b`, `i_c`, `w_M`, `theta_M`, `tau_M`; the grid converters: the converter currents and, with the LCL filter, the grid currents) and `ctrl` (the monitored signals) for the comparison, and a scope.
- **Layout**: the blocks are aligned with the ports they connect to, so that the lines are straight, and the feedback lines are drawn through given corners.

## Agreement with *motulator*

Maximum differences (Simulink − *motulator*) printed by the comparison scripts, with *motulator* 0.14.1 and MATLAB R2026b with the MinGW-w64 compiler (gcc 14.2), at the level of the PLECS models (cf. [examples/README.md](../../examples/README.md)).

| Script                               | System model                               | Control system                                               |
| ------------------------------------ | ------------------------------------------ | ------------------------------------------------------------ |
| `ipmsm_2kw_fvc_simulink.py`          | w_M 1.2e-6, tau_M 2.0e-6, i_s_ab 1.4e-6    | w_M 3.0e-6, tau_M 2.0e-6, tau_M_ref 2.2e-6, psi_s_ref 8.1e-9 |
| `im_2kw_cvc_simulink.py`             | w_M 6.5e-7, tau_M 2.5e-6, i_s_ab 1.1e-6    | w_M 4.6e-6, tau_M 3.3e-6, tau_M_ref 3.5e-6, psi_R 3.2e-8     |
| `im_2kw_vhz_simulink.py`             | w_M 2.2e-5, tau_M 2.0e-5, i_s_ab 8.1e-6    | w_s 2.4e-5, tau_M 2.0e-5, tau_M_ref 2.1e-6, psi_s 6.6e-8     |
| `im_2kw_vhz_simulink.py --open-loop` | w_M 2.3e-5, tau_M 2.5e-5, i_s_ab 8.9e-6    | w_s 0, tau_M 0, tau_M_ref 0, psi_s 4.4e-16                   |
| `pmsyrm_6kw_gn_fvc_simulink.py`      | w_M 4.3e-4, tau_M 2.0e-3, i_s_ab 7.3e-4    |                                                              |
| `gfl_10kva_lcl_simulink.py`          | i_c_ab 3.1e-6, i_g_ab 2.2e-6, i_c_a 3.1e-6 | p_g 1.3e-3, q_g 7.4e-4, u_g 0, w_g 2.8e-13                   |
| `gfm_13kva_do_simulink.py`           | i_c_ab 1.3e-6, i_c_a 1.2e-6                | p_g 5.2e-4, q_g 3.9e-4, v_c 3.3e-6, theta_c 0                |

With the MATLAB Function blocks (`--matlab`), the differences are the same to the printed digits (psi_s 2.2e-16 in open-loop V/Hz control), since the MATLAB port gives the results of the C port to the rounding errors.

The simulation of the IPMSM drive takes about 1.5 s in the normal mode (about 100 000 solver steps, mostly at the switching instants located by the zero-crossing detection) and 0.6 s in the accelerator mode, plus about 1 s of initialization.
Most of the run time of a comparison script is the start of MATLAB, the build of the model, and the compilation of the MATLAB Function block.
With the MATLAB Function blocks, the simulation takes about as long (in a measurement on the same computer, 2.3 s vs. 2.9 s with the S-functions in the normal mode, and 2.2 s vs. 1.9 s in the accelerator mode), but the first simulation after loading the model compiles the blocks (21 s vs. 13 s).

In `pmsyrm_6kw_gn_fvc_simulink.py`, the differences are larger, since *motulator* evaluates the GradNets in single precision, as in the PLECS model.

In addition, `tests/test_control.py` generates the S-functions of the blocks, compiles them with gcc against a mock of the Simulink API (`tests/simulink_mock`), and executes them in the order of their direct feedthrough, as Simulink does, in closed-loop simulations of *motulator* (the drives in the sensorless and sensored modes), with the parameters computed from the initialization script through the masks; the results agree with those of the control systems of *motulator* to about 1e-12 relative to the signal magnitudes, and they equal those of the monolithic reference control systems of the tests exactly. The S-functions with GradNets (flux-vector control with a flux map and the machine) agree with *motulator* to its single precision (`tests/test_simulink.py`).
`tests/test_matlab_function.py` (with the MATLAB Engine API) runs the control systems with MATLAB Function blocks in Simulink with the inputs of these simulations and checks that they give the outputs of the S-functions, see [the MATLAB port](../matlab/README.md).
