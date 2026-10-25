# Simulink export

Export of *motulator* systems to Simulink, corresponding to the PLECS export.
The models are built from the blocks of the Simulink library of *motulator*: each class of *motulator* is a block with a mask of the arguments of its class, in the hierarchy of *motulator*.
A model is built in MATLAB by a script that builds the system with the MATLAB classes of *motulator* (the package `+motulator`, see [MATLAB classes](#matlab-classes)), as a *motulator* script does, and `write_model` writes such a script from the *motulator* objects.
The blocks of the classes of the control systems run the same C code as in the PLECS model, the code sections of the C-Script blocks (the C port of the *motulator* control algorithms in `../c`) wrapped in generated C MEX S-functions, or, in the library with MATLAB Function blocks (`code="matlab"`), the MATLAB port of the same code (`../matlab`), with the same masks; these need no C compiler (see [MATLAB Function blocks](#matlab-function-blocks)).
The system models are built from basic Simulink blocks, so no toolboxes are needed.

| System                                                                                                                                                                                                            | Example                                                                                                                                                                                                                                                                                            |
| ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Synchronous machine drive (`SynchronousMachinePars`, or a GradNet current map with spatial harmonics), flux-vector control (also with a GradNet flux map), current-vector control, or observer-based V/Hz control | [ipmsm_2kw_fvc_simulink.py](../../examples/ipmsm_2kw_fvc_simulink.py), [pmsyrm_6kw_gn_fvc_simulink.py](../../examples/pmsyrm_6kw_gn_fvc_simulink.py), [ipmsm_2kw_cvc_simulink.py](../../examples/ipmsm_2kw_cvc_simulink.py), [ipmsm_2kw_vhz_simulink.py](../../examples/ipmsm_2kw_vhz_simulink.py) |
| Induction machine drive (constant parameters), current-vector control, flux-vector control, or observer-based V/Hz control (also open-loop V/Hz control)                                                          | [im_2kw_cvc_simulink.py](../../examples/im_2kw_cvc_simulink.py), [im_2kw_fvc_simulink.py](../../examples/im_2kw_fvc_simulink.py), [im_2kw_vhz_simulink.py](../../examples/im_2kw_vhz_simulink.py)                                                                                                  |
| Grid converter system (LCL filter or L filter and grid inductance), grid-following or grid-forming control                                                                                                        | [gfl_10kva_lcl_simulink.py](../../examples/gfl_10kva_lcl_simulink.py), [gfm_13kva_do_simulink.py](../../examples/gfm_13kva_do_simulink.py)                                                                                                                                                         |

## Usage

The use of the models is described in the user guide ([Simulink models](https://aalto-electric-drives.github.io/motulator-export/guide/simulink.html)).
In the examples, `python examples/ipmsm_2kw_fvc_simulink.py` writes the build script `examples/simulink/build_ipmsm_2kw_fvc.m`, and with `--matlab`, `build_ipmsm_2kw_fvc_matlab.m`, which builds the model with MATLAB Function blocks.
With the MATLAB Engine API for Python, the script also builds and simulates the model and prints the maximum differences from *motulator*.
On Windows, the tests and the examples use the MinGW-w64 compiler of MATLAB (the MATLAB Support for MinGW-w64 C/C++ Compiler add-on); the compiled S-functions cannot be overwritten while they are loaded in MATLAB (`clear mex`).

## Files

| File                    | Contents                                                                                                                                                                                                           |
| ----------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `_export.py`            | `write_model` and `simulate` of the package, with the checks of the supported systems                                                                                                                              |
| `_script.py`            | Writer of the build scripts: the system with the MATLAB classes of *motulator*, from the *motulator* objects                                                                                                       |
| `_library.py`           | Writer of the files of the library (`library/`): the data of the blocks of the control systems and the code of their blocks                                                                                        |
| `_render.py`            | Control systems: the S-functions (or the MATLAB functions) of the blocks and the netlist rows (blocks, alignments, lines, and masks) for `motulator.internal.add_netlist`                                          |
| `_sfunction.py`         | Generator of the S-functions from the code sections of a C-Script block                                                                                                                                            |
| `_mfunction.py`         | Generator of the MATLAB functions of the MATLAB Function blocks from the MATLAB code of a block (`MCode`) and the MATLAB port, and of the masks collecting their parameters                                        |
| `_common.py`            | The checks of the system model, the data of the blocks of the control systems, and the simulation via the MATLAB Engine API                                                                                        |
| `_matlab.py`            | Interpreter of the subset of MATLAB of the model and mask initializations, which computes the parameters of the blocks through the masks without MATLAB (used by the tests)                                        |
| `+motulator/`           | The MATLAB classes of *motulator* (the packages `common`, `drive`, and `grid`), `motulator.build_library`, and `motulator.setup`                                                                                   |
| `+motulator/+internal/` | The base class of the MATLAB classes and the builders of the library and the models: the blocks of the system models, the control-system blocks from their data (`add_netlist`), and the layout helpers            |
| `library/`              | The files written by `_library.py` (part of the package, checked by `tests/test_library.py`) and the libraries built from them (`motulator_lib.slx` and `motulator_lib_matlab.slx`, with the compiled S-functions) |

## MATLAB classes

The classes of the package `+motulator` have the names, the arguments, and the defaults of the classes of *motulator* (checked by `tests/test_library.py`), in the packages of *motulator* (e.g., `motulator.drive.control.sm`); the classes that *motulator* exports from several modules (e.g., `SpeedController` and `PWM`) are defined once and re-exported by functions, so that a script imports the same packages as a *motulator* script.
The arguments are the public properties, set by name-value arguments (`motulator.internal.Object`), `[]` being `None`, and `params` gives them as a struct, nested for the arguments that are objects (e.g., the struct `vector_ctrl` with the fields `par` and `cfg`).
`Simulation.build` builds the model: the parameters of the model, the structs of the parts of the system model and the control system named as in *motulator* (`machine`, `mechanics`, `converter`, `vector_ctrl`, `speed_ctrl`, `pwm`, etc.), are put in its model workspace, and the blocks of the library are added, whose masks refer to them by default, and connected (`motulator.internal.build_drive` and `build_grid`).
The references of the control system are sampled to switch at the same sampling instant as in *motulator* (`StepSignal.sampled`, the counterpart of `sampled_step`).
With `control_block`, `Simulation.build` takes the control system from a block of another library with the same inputs and outputs, keeping the values of its mask (e.g., a block with tunable parameters), and shows its further outputs in Display blocks (`motulator.internal.control_block` and `add_outputs`); `write_script` writes such a build script with its argument `control_block`.
`Simulation.simulate` runs the model and returns the signals of its output ports (`SimulationResults`), which `plot` shows as the scope of the model.

## Library

The library is built in MATLAB by `motulator.build_library` (on the first use by `Simulation.build`, `motulator.internal.library`) from the files of `library/`: the data of the blocks of the control systems (`control_blocks_c.m` and `control_blocks_matlab.m`, written by `_library.py` from the netlists), the S-functions and the MATLAB functions of their blocks, and the blocks of the system models, built by the functions of `+motulator/+internal` (e.g., `add_synchronous_machine`).
The version of the inputs of the build (a hash of the files, of the MATLAB code that builds the library, and of the C port) is the description of the built library, so a library built from other inputs is built again.
`motulator.setup` warns of compiled S-functions of other folders that shadow those of the library, e.g., those next to the models of earlier versions.
The library is registered in the Library Browser by `slblocks.m`, which `motulator.build_library` writes next to it, and the models add the folder of the library to the MATLAB path when they are loaded (`motulator.setup`).

## S-function

The S-function defines the C-Script macros used in the code (`InputSignal`, `OutputSignal`, `ParamRealData`, `ParamDim`, `SetErrorMessage`, `ContState`, and `ContDeriv`) with the Simulink API and calls the code sections in its callbacks: the start function in `mdlStart`, the output function in `mdlOutputs`, and the update function in `mdlUpdate`.
It has the ports, the parameters, the sample time, and the continuous states of the C-Script block.
As in *motulator*, the outputs are computed from the measurements of the same sampling instant and the states are updated in `mdlUpdate`.
The state of the C code is in static variables, as in the C-Script block, so a model can contain one instance of a control system with S-functions.

## Gate enable

The first input of every control system is `enable`, the gate enable of *motulator* (`ControlSystem.set_enable`).
While `enable` is zero, the control system runs in standby, and after the release, a sensorless drive catches the rotating machine with a flying start, as in *motulator* (see [the PLECS export](../plecs/README.md#structure-of-the-models)).
The system model of Simulink does not block the gates, so in the exported models `enable` is 1 (`write_model` rejects a control system with the gates blocked); on the hardware, the gate enable also blocks the gates of the converter.
The control systems limit the measured DC-bus voltage to at least 1 V (`U_DC_MIN`), which avoids the division by zero in the PWM if the DC bus is not charged.

## MATLAB Function blocks

With `code="matlab"`, each block of a class runs a MATLAB function generated by `_mfunction.py` from the MATLAB code of the block (`MCode` in the netlists of `../plecs`, the counterpart of its C-Script code) and the MATLAB port (`../matlab`), whose functions that the block uses are appended as local functions:

- The function has the inputs and outputs of the block, the complex signals and the signal vectors (`meas`, `fbk`, and `ref`) converted to complex numbers and structs (e.g., `meas = measurements_unpack(meas_in)`).
- The state is in persistent variables: `self` is initialized from the parameters at the first call (the start function), the outputs are computed, and then the state is updated, as in the S-function. Each MATLAB Function block has its own copy of the persistent variables, also if several blocks call the same function, so a model can contain several instances of a block.
- The parameters are the fields of the struct `p`, a nontunable parameter of the MATLAB Function block. The mask of the block (that of its class, or a mask of its parameters for the blocks without one) collects the parameters into `p` in its initialization, an empty parameter (`[]`, `None`) being `NaN`, since a MATLAB Function block does not accept empty parameters, and checks them (e.g., `J must be a scalar.`). The parameters are constants of the code generation, so the code for, e.g., the GradNet flux map is compiled only if the machine model has one.
- A MATLAB Function block has no start function, so a long initialization (the control loci of the `ReferenceGenerator` of synchronous machines, with 48 root findings) would run at the first step. Instead, the mask initialization of the block computes the lookup tables with the function `sm_reference_luts.m` of the library (`Mask.simulink_init` of the netlists, `PATH_FUNCTIONS` of `_mfunction.py`), and the S-function version of the block uses the same tables; in PLECS, the C-Script block computes them at its start.
- All inputs of a MATLAB Function block have direct feedthrough. The inputs that the S-functions use only in the update of the state (e.g., the limited torque reference of the speed controller, the duty ratios of the PWM, and the inputs of the speed observer), which would close algebraic loops, come through Unit Delay blocks, and the update with them is done at the start of the next call, which gives the same result. Such a block is a masked subsystem containing the MATLAB Function block (named by its function) and the Unit Delay blocks.
- Simulink compiles the MATLAB Function blocks just in time, so no C compiler is needed. A block only calls the function of its file `mfun_*.m` in the folder of the library (`motulator.internal.add_mfunction`, with the arguments of the first line of the file), so that the code of the block is in the file, as that of an S-function is in its C source. Simulink resizes a MATLAB Function block when its code is set, so `motulator.internal.add_mfunction` restores the size of the layout: the blocks are aligned and the lines drawn through the points of the layout, and a line starting at another port would join its line.

The synchronous machine with a GradNet current map is a MATLAB Function block too (`mfun_gradnet_machine`), with the stator flux linkage in an Integrator block, whose initial value is the PM-flux linkage of *motulator* (`psi_f` of the struct of the current map).

## Model structure

- **Control system**: a masked subsystem with the arguments of its top-level class (e.g., `vector_ctrl`, `speed_ctrl`, and `pwm`), containing the netlist of the blocks, built by `motulator.internal.add_netlist`: the S-functions (or the MATLAB Function blocks) of the classes (masked, showing the name of the class and the port labels), the subsystems (masked for the composite classes), and the Selector, Mux, Goto, and From blocks, as in the PLECS model (see [the PLECS export](../plecs/README.md)). The layout is computed in Python (`plecs/_netlist.py`), and the blocks are aligned with the actual ports of Simulink. The unconnected outputs (e.g., the load torque estimate of the speed observer) are terminated. The S-functions get their parameters after the lines are created and before the masks, since setting the parameters evaluates the mask of an S-function, which fails in the library. A GradNet flux map is a struct (`par.psi_s_dq_fcn`), whose fields the mask initialization passes to the S-functions. The monitored signals of the block `Monitored signals` are combined into the output `signals`. In the torque-control mode (`VectorControlSystem` without the speed controller), the control system is the block of the torque-control mode (e.g., `Flux-vector control (torque)`, see [the PLECS export](../plecs/README.md)), whose mask has no `speed_ctrl`.
- **Computational delay**: a Unit Delay block.
- **PWM**: a triangular carrier (Repeating Sequence, period `2*T_s`, starting from its maximum) compared with the duty ratios by a Relational Operator block. Simulink locates the switching instants by zero-crossing detection. This corresponds to `CarrierComparison` of *motulator* without the counter quantization.
- **Converter**: the ideal two-level converter with a stiff DC bus, `u_s_ab = u_dc*q_ab` (`VoltageSourceConverter`). The dead time and the threshold voltage of the converter are not modeled, and the duty ratios are updated twice per carrier period, but the control system of a drive can compensate for the dead time and the threshold voltage (`PWM(d_err=InverterNonlinearity(t_d, u_th, sign))` with the current-direction function `np.sign` or `tanh(i/i_0)`), also with single update (`PWM(single_update=True)`), as in the PLECS export; this is intended for the control system running on the hardware. The DC-bus models of the PLECS export (a capacitor or a diode bridge, `DCBus` and `DiodeBridgeDCBus`, `ipmsm_2kw_fvc.py --diode`) are not supported, since their switching circuits would need Simscape Electrical or the zero-crossing detection of the diode states.
- **AC filter and grid** (grid converters): the LCL filter, or the L filter with the grid inductance, modeled with space vectors in stationary coordinates in a MATLAB Function block, with the states in an Integrator block, as in `LCLFilter` and `LFilter` of *motulator*. The grid voltage `e_g*exp(1j*w_g*t)` is generated by Sine Wave blocks. Without the grid impedance, the PCC voltage measured by grid-following control is the grid voltage.
- **References and the input `enable`**: Step blocks, or a sum of Step blocks for several steps, and Constant blocks (`enable` 1).
- **Machine**: the flux linkages as the state (an Integrator block), and a MATLAB Function block with the currents, the voltage equations, and the torque. The synchronous machine is modeled in rotor coordinates, as `SynchronousMachine` of *motulator*. The induction machine is modeled with the inverse-Γ model in stator coordinates, which is equivalent to the Γ model of `InductionMachine` of *motulator*.
- **Mechanics**: integrators for the speed and the angle (`MechanicalSystem` without friction), starting from the state of the mechanics of *motulator* (`mechanics.state.w_M` and `mechanics.state.theta_M`), or the integrator of the angle from the external rotor speed (`ExternalRotorSpeed`, the speed given by a Step or Constant block `w_M`).
- **Solver**: `ode45` (Dormand–Prince) with the relative tolerance of 1e-6 and the maximum step `T_s`, as in PLECS.
- **Outputs**: the root-level output ports `mdl` (the drives: `i_a`, `i_b`, `i_c`, `w_M`, `theta_M`, `tau_M`; the grid converters: the converter currents and, with the LCL filter, the grid currents) and `ctrl` (the monitored signals) for the comparison, and a scope.
- **Layout**: the blocks are aligned with the ports they connect to, so that the lines are straight, and the feedback lines are drawn through given corners.

## Agreement with *motulator*

Maximum differences (Simulink − *motulator*) printed by the comparison scripts, with *motulator* 0.17.1 and MATLAB R2026b with gcc 14.2 on Linux, at the level of the PLECS models (cf. [the PLECS export](../plecs/README.md#agreement-with-motulator)).

| Script                                 | System model                               | Control system                                               |
| -------------------------------------- | ------------------------------------------ | ------------------------------------------------------------ |
| `ipmsm_2kw_fvc_simulink.py`            | w_M 9.3e-7, tau_M 2.0e-6, i_s_ab 1.1e-6    | w_M 3.0e-6, tau_M 2.1e-6, tau_M_ref 2.2e-6, psi_s_ref 1.0e-8 |
| `ipmsm_2kw_cvc_simulink.py`            | w_M 1.8e-5, tau_M 1.2e-5, i_s_ab 5.1e-6    | w_M 1.5e-5, tau_M 1.2e-5, tau_M_ref 1.3e-5, i_q_ref 5.0e-6   |
| `ipmsm_2kw_cvc_simulink.py --discrete` | w_M 1.8e-5, tau_M 1.3e-5, i_s_ab 5.5e-6    | w_M 1.5e-5, tau_M 1.3e-5, tau_M_ref 1.3e-5, i_q_ref 5.2e-6   |
| `ipmsm_2kw_vhz_simulink.py`            | w_M 2.1e-5, tau_M 1.1e-5, i_s_ab 1.0e-5    | w_c 6.6e-5, tau_M 1.1e-5, tau_M_ref 1.8e-6, psi_s 1.2e-8     |
| `im_2kw_cvc_simulink.py`               | w_M 6.5e-7, tau_M 2.4e-6, i_s_ab 9.7e-7    | w_M 4.3e-6, tau_M 3.1e-6, tau_M_ref 3.3e-6, psi_R 3.2e-8     |
| `im_2kw_fvc_simulink.py`               | w_M 4.2e-5, tau_M 3.3e-5, i_s_ab 3.3e-5    | w_M 4.4e-5, tau_M 3.2e-5, tau_M_ref 3.5e-5, psi_s_ref 5.0e-8 |
| `im_2kw_vhz_simulink.py`               | w_M 2.2e-5, tau_M 2.0e-5, i_s_ab 8.1e-6    | w_s 2.4e-5, tau_M 2.0e-5, tau_M_ref 2.1e-6, psi_s 6.6e-8     |
| `im_2kw_vhz_simulink.py --open-loop`   | w_M 2.3e-5, tau_M 2.5e-5, i_s_ab 8.9e-6    | w_s 0, tau_M 0, tau_M_ref 0, psi_s 2.2e-16                   |
| `pmsyrm_6kw_gn_fvc_simulink.py`        | w_M 5.4e-5, tau_M 2.4e-4, i_s_ab 9.6e-5    |                                                              |
| `gfl_10kva_lcl_simulink.py`            | i_c_ab 3.1e-6, i_g_ab 2.1e-6, i_c_a 3.1e-6 | p_g 1.3e-3, q_g 7.4e-4, u_g 0, w_g 2.8e-13                   |
| `gfm_13kva_do_simulink.py`             | i_c_ab 1.2e-6, i_c_a 1.2e-6                | p_g 4.9e-4, q_g 3.9e-4, v_c 3.3e-6, theta_c 1.8e-8           |

With the MATLAB Function blocks (`--matlab`), the differences are the same to the printed digits (psi_s 4.4e-16 in open-loop V/Hz control), since the MATLAB port gives the results of the C port to the rounding errors.
In `pmsyrm_6kw_gn_fvc_simulink.py`, the differences are larger, as in the PLECS model.

## Run times

Run times of the simulation of the IPMSM drive (`ipmsm_2kw_fvc_simulink.py`, 1.2 s) with `sim` on a laptop with MATLAB R2026b, in a new MATLAB session with an empty cache folder (the times vary by about 30 % between the runs):

| Simulation                                         | S-functions | MATLAB Function blocks |
| -------------------------------------------------- | ----------: | ---------------------: |
| First after loading the model (compiles the model) |        10 s |                35–45 s |
| Later, normal mode                                 |       2–3 s |                  2–3 s |
| Later, accelerator mode                            |         2 s |                    2 s |

The normal mode takes about 100 000 solver steps, mostly at the switching instants located by the zero-crossing detection.
The first simulation with MATLAB Function blocks compiles them just in time; since the blocks call the functions of the files, this takes about 1.5 times as long as with the code of the functions copied into the blocks (as up to version 0.8.0, 22–25 s).
Most of the run time of a comparison script is the start of MATLAB, the build of the model, and the first simulation.

## Tests

`tests/test_control.py` generates the S-functions of the blocks, compiles them with gcc against a mock of the Simulink API (`tests/simulink_mock`), and executes them in the order of their direct feedthrough, as Simulink does, in closed-loop simulations of *motulator* (the drives in the sensorless and sensored modes), with the parameters computed from the model initialization (`init_script` of the control modules) through the masks; the results agree with those of the control systems of *motulator* to about 1e-12 relative to the signal magnitudes, and they equal those of the monolithic reference control systems of the tests exactly. The S-functions with GradNets (flux-vector control with a flux map and the machine) agree with *motulator* to the rounding errors (`tests/test_simulink.py`).
`tests/test_matlab_function.py` (with the MATLAB Engine API) runs the control systems of the library with MATLAB Function blocks in Simulink with the inputs of these simulations and checks that they give the outputs of the S-functions, see [the MATLAB port](../matlab/README.md).
It also builds both libraries, checking that the lines of their control systems connect the ports of the netlists, and the models of the systems of the examples by the scripts of `write_model`, comparing the signals of their system models with the native simulation (to 1e-4 relative to their magnitudes), so that it tests the MATLAB code of `+motulator`, which runs only in MATLAB; run it before a release.
`tests/test_library.py` checks that the files of `library/` are up to date and that the MATLAB classes have the defaults of *motulator*.
