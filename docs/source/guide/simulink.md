# Simulink models

The Simulink models are built from a library of the blocks of *motulator*: each class is a block with a mask of the arguments of its class, nested as in *motulator*. A model is built in MATLAB by a script that builds the system with the MATLAB classes of *motulator* (the package `+motulator` of *motulator-export*), as a *motulator* script does:

```matlab
addpath('motulator-export/motulator_export/simulink')  % The package, see Setup
import motulator.drive.model.*
import motulator.drive.control.sm.*
import motulator.drive.utils.*

par = SynchronousMachinePars(n_p=3, R_s=3.6, L_d=0.036, L_q=0.051, psi_f=0.545);
machine = SynchronousMachine(par);
mechanics = MechanicalSystem(J=0.015);
converter = VoltageSourceConverter(u_dc=540);
mdl = Drive(machine, mechanics, converter);

cfg = FluxVectorControllerCfg(i_s_max=6.5);
vector_ctrl = FluxVectorController(par, cfg);
speed_ctrl = SpeedController(J=0.015, alpha_s=25);
ctrl = VectorControlSystem(vector_ctrl, speed_ctrl);
ctrl.set_speed_ref(StepSignal(time=0.1, after=50));
mdl.mechanics.set_external_load_torque(StepSignal(time=0.7, after=10));

simulation = Simulation(mdl, ctrl);
simulation.build('ipmsm', t_stop=1.2);  % Saves ipmsm.slx in the current folder
res = simulation.simulate();
res.plot();
```

```{image} ../images/simulink_drive.png
---
target: ../_images/simulink_drive.png
alt: Simulink model of a 5.6-kW PM-SyRM drive
class: only-light
---
```

```{image} ../images/simulink_drive_black.png
---
target: ../_images/simulink_drive_black.png
alt: Simulink model of a 5.6-kW PM-SyRM drive
class: only-dark
---
```

*Simulink model built by `examples/simulink/build_pmsyrm_6kw_gn_fvc.m`. The output ports `mdl` and `ctrl` give the signals of the results.*

## Setup

Add the folder `motulator_export/simulink` of the package (see [](getting_started.md#installation)) to the MATLAB path, e.g., `addpath('motulator-export/motulator_export/simulink')` for a clone of the repository. The first model builds the library of the blocks (`motulator.build_library`), which takes a few minutes, since it compiles the blocks of the control systems into S-functions; this needs a C compiler, see [](getting_started.md#requirements). The library is built again only when the package changes.

The MATLAB classes have the names of the classes of *motulator*, in the same packages, e.g., `motulator.drive.control.sm.FluxVectorController`, and the arguments that the export supports, with the defaults of *motulator* (`[]` means `None`). They are listed in [](supported.md), and their help shows their arguments, e.g., `help motulator.drive.control.sm.FluxVectorControllerCfg`. The references and the load torque are constants or step signals (`StepSignal`).

## The library

Once built, the library is in the Library Browser as *motulator (S-functions)*, or *motulator (MATLAB Function blocks)* (see below); in a new MATLAB session, `motulator.setup()` adds it to the path. It has the blocks of the control systems (e.g., `Flux-vector control`, a `VectorControlSystem` with a `FluxVectorController`) and those of the parts of the system models (e.g., `SynchronousMachine` and `MechanicalSystem`). Inside the control systems, each class is a block, as in the [PLECS models](plecs.md#control-system).

The mask of a control system has the arguments of its top-level class as structs, e.g., `vector_ctrl`, `speed_ctrl`, and `pwm` of `VectorControlSystem`, and passes them to the blocks inside it. A model built by `Simulation.build` has the arguments of the parts of the system and the control system as structs in its model workspace, named as in *motulator* (e.g., `machine`, `mechanics`, `vector_ctrl`, and `pwm`), and the masks refer to them. To change a parameter, change the script and build the model again, or change the struct in the model workspace (in the Model Explorer). In a model of your own, drag the blocks from the Library Browser and give their masks the structs of the arguments, which the method `params` of the objects gives, e.g., `vector_ctrl = vector_ctrl.params()`.

## MATLAB Function blocks

Without a C compiler, build the model with MATLAB Function blocks:

```matlab
simulation.build('ipmsm', t_stop=1.2, code='matlab');
```

The blocks then run a MATLAB port of the same control code, which is in the files `mfun_*.m` of the folder `motulator_export/simulink/library`, so you can read it without MATLAB. The results are the same as with the S-functions, to the rounding errors. The first simulation compiles the MATLAB Function blocks, which takes up to a minute.

## From Python

`simulink.write_model` writes the script that builds the model of a system built in *motulator*:

```python
from motulator_export import simulink

simulink.write_model("ipmsm.slx", mdl, ctrl, t_stop=1.2)
```

This writes `build_ipmsm.m`, which adds the folder of the package to the MATLAB path, builds the same system with the MATLAB classes, and saves `ipmsm.slx` next to it. Run it in MATLAB. With `code="matlab"`, the model has MATLAB Function blocks.

With the MATLAB Engine API for Python, `simulink.simulate` builds the model and simulates it. It takes the path of the model or that of its build script, which `write_model` returns:

```python
import numpy as np

script = simulink.write_model("ipmsm.slx", mdl, ctrl, t_stop=1.2)
t_eval = np.linspace(0, 1.2, 1201)
mdl_signals, ctrl_signals = simulink.simulate(script, t_eval, mdl, ctrl)
```

The signals are those of the [PLECS models](plecs.md#simulating-from-python). In MATLAB, `res.mdl` and `res.ctrl` of `Simulation.simulate` have the same names, without the space vectors.

## Good to know

- The script of `write_model` refers to the package by a path relative to the folder of the model. If you move the files, write the model again.
- A model needs the library: a model opened in a new MATLAB session finds it if the folder `motulator_export/simulink` is on the MATLAB path.
- The models agree with *motulator* as the PLECS models do, see [](plecs.md#accuracy).
- An S-function keeps its state in static variables, so a model can contain only one instance of each control system with S-functions. The MATLAB Function blocks have no such limit.
- The Simulink models support fewer configurations than the PLECS models, e.g., no dead time, DC-bus dynamics, or gate blocking (the gate enable is 1), see [](supported.md). The control systems still have the input `enable` and run in standby while it is zero, e.g., when the control code is reused on hardware.
