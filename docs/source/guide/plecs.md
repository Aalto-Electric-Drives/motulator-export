# PLECS models

`plecs.write_model` writes a PLECS Standalone model of the system. The system model is built from the native blocks of PLECS, and the control system is a subsystem on the left:

```{image} ../images/plecs_drive.png
---
target: ../_images/plecs_drive.png
alt: Top level of the PLECS model of a 2.2-kW IPMSM drive
class: only-light
---
```

```{image} ../images/plecs_drive_black.png
---
target: ../_images/plecs_drive_black.png
alt: Top level of the PLECS model of a 2.2-kW IPMSM drive
class: only-dark
---
```

*Top level of the model written by `examples/ipmsm_2kw_fvc.py`: the control system with its references and measurements, the computational delay, the PWM, the converter, the machine, the load, and the scope.*

## Control system

Inside the control system, each class of *motulator* is a block, nested as in *motulator*. The blocks run the C port of the control algorithms in C-Script blocks:

```{image} ../images/plecs_control.png
---
target: ../_images/plecs_control.png
alt: Blocks of the flux-vector control system in PLECS
class: only-light
---
```

```{image} ../images/plecs_control_black.png
---
target: ../_images/plecs_control_black.png
alt: Blocks of the flux-vector control system in PLECS
class: only-dark
---
```

*Inside the flux-vector control system (`VectorControlSystem`): the measurements, the `SpeedController`, the `FluxVectorController`, and the `PWM`.*

The mask of each block shows the arguments of its class. The mask of the control system itself has the arguments of its class, e.g., `vector_ctrl`, `speed_ctrl`, and `pwm` of `VectorControlSystem`, as structs from the parameters of the model (see below), and passes them to the blocks inside it, as the constructors of *motulator* do.

```{image} ../images/plecs_mask.png
---
target: ../_images/plecs_mask.png
alt: Mask dialog of the SpeedController block
width: 500px
class: only-light
---
```

```{image} ../images/plecs_mask_black.png
---
target: ../_images/plecs_mask_black.png
alt: Mask dialog of the SpeedController block
width: 500px
class: only-dark
---
```

*Mask of the `SpeedController` block.*

## Parameters

The parameters are defined in the initialization commands of the model (Simulation > Simulation Parameters > Initialization), which read as a *motulator* script. An empty value (`[]`) means `None`, i.e., the default of *motulator*. For example:

```matlab
% Machine model of the control system (SynchronousMachinePars)
par.n_p = 3;
par.R_s = 3.6;
par.L_d = 0.036;
par.L_q = 0.051;
par.psi_f = 0.545;
% Flux-vector control (FluxVectorControllerCfg)
cfg.i_s_max = 6.5;
cfg.alpha_tau = 2*pi*100;
cfg.alpha_psi = [];
...
% Flux-vector controller (FluxVectorController)
vector_ctrl.par = par;
vector_ctrl.cfg = cfg;
% Speed controller (SpeedController)
speed_ctrl.J = 0.015;
speed_ctrl.alpha_s = 25;
...
```

Changing a parameter needs no Python: edit the initialization commands and run the simulation.

## Simulation results

The scope shows the speed, the torque, the stator currents, and the flux linkage:

```{image} ../images/plecs_scope.png
---
target: ../_images/plecs_scope.png
alt: Scope of the PLECS model of a 2.2-kW IPMSM drive
width: 600px
class: only-light
---
```

```{image} ../images/plecs_scope_black.png
---
target: ../_images/plecs_scope_black.png
alt: Scope of the PLECS model of a 2.2-kW IPMSM drive
width: 600px
class: only-dark
---
```

*Scope of the model written by `examples/ipmsm_2kw_fvc.py`: a speed step at 0.1 s and a load torque step at 0.7 s. The speed and the torque plots show the reference, the estimate of the control system, and the actual value, and the flux linkage plot the reference and the estimate.*

## Grid converter systems

The grid converter systems have the same structure, with the AC filter and the grid in place of the machine:

```{image} ../images/plecs_grid.png
---
target: ../_images/plecs_grid.png
alt: Top level of the PLECS model of a 10-kVA grid converter system
class: only-light
---
```

```{image} ../images/plecs_grid_black.png
---
target: ../_images/plecs_grid_black.png
alt: Top level of the PLECS model of a 10-kVA grid converter system
class: only-dark
---
```

*Top level of the model written by `examples/gfl_10kva_lcl.py`: grid-following control of a converter with an LCL filter.*

## The library

The blocks of the control systems and the GradNet machine are also in the PLECS library `motulator_lib.plecs`, in the folder `motulator_export/plecs` of the package (see [](getting_started.md#installation)). To use them in your own models, open the Library page of File > PLECS Preferences, add the folder to the Search Path table, and add `motulator_lib.plecs` to the User Libraries table with a description, e.g., *motulator*, which names it in the Library Browser. The blocks dragged from the Library Browser are links to the library, so they follow the updates of the package. Their masks refer to the structs of the initialization commands (see [Parameters](#parameters)), which you can copy from a model written by `plecs.write_model`.

With `library=True`, `plecs.write_model` writes a model whose control system (and GradNet machine) refers to the library by a path relative to the model file, without the settings above:

```python
plecs.write_model("ipmsm.plecs", mdl, ctrl, t_stop=1.2, library=True)
```

The model file is then small, but it needs the library. Before you send it to someone without the package, break the links (Edit > Break all external links…).

## Simulating from Python

`plecs.simulate` runs a model through the RPC interface of PLECS and returns the signals of the system model and the monitored signals of the control system, e.g., for comparing with *motulator*. The model needs output ports for this:

```python
import numpy as np

path = plecs.write_model("ipmsm.plecs", mdl, ctrl, t_stop=1.2, outputs=True)
t_eval = np.linspace(0, 1.2, 1201)
mdl_signals, ctrl_signals = plecs.simulate(path, t_eval, mdl, ctrl)
```

The signals are arrays by their names, with the time `"t"`, e.g., `mdl_signals["w_M"]`, the stator current `mdl_signals["i_s_ab"]` as a space vector, and `ctrl_signals["tau_M_ref"]`. The names are listed in the API reference of `plecs.simulate`.

## Accuracy

In the comparisons of the examples, the PLECS models agree with *motulator* to about 1e-6 relative to the signal magnitudes. The comparisons use tight tolerances in *motulator* and a fine quantization of the duty ratios (`CarrierComparison(N=2**24)`), since the PLECS model does not quantize them; with the default quantization of 4096 levels, the differences are about 1e-2. With the dead time of the converter, the differences are larger, since PLECS models the switching transients during the blanking in more detail.

## Good to know

- The first input of the control system is the gate enable (`ctrl.set_enable`). While it is zero, the gates of the converter are blocked, and the control system runs in standby, as in *motulator*; after the release, a sensorless drive catches the rotating machine with a flying start.
- The model refers to the C sources of the installed package (with `library=True`, to its library) by a path relative to the model file. If you move the model or update the package, write the model again.
- `plecs.simulate` closes the model in PLECS before loading it, discarding unsaved changes, since PLECS does not reload an open model from the file.
