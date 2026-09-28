# motulator-plecs

Export [motulator](https://github.com/Aalto-Electric-Drives/motulator) drive systems and grid converter systems to [PLECS](https://www.plexim.com/products/plecs) Standalone.
The control system is ported to C and runs in a C-Script block inside a masked subsystem, whose parameters are the same as in the motulator API (e.g., `SynchronousMachinePars`, `FluxVectorControllerCfg`, and `SpeedController`), including the defaults: an empty parameter (`[]`) corresponds to `None` in motulator.
The derived quantities, such as the gains and the lookup tables, are computed by the C code at the start of the simulation, so the PLECS model is self-contained: the parameters can be changed in the mask without Python.
The system model (the machine, the mechanics, the converter, the DC bus, the filters, and the grid) is built from PLECS blocks as far as possible.

## Installation

```bash
pip install git+https://github.com/mhinkkan/motulator-plecs
```

The package requires motulator 0.8.0 or later.
For development, clone the repository and install it in editable mode with the development tools, `pip install -e .[dev]`.

## Usage

```python
from motulator_plecs import sm

sm.write_model("ipmsm.plecs", mdl, ctrl, w_M_ref, tau_L, t_stop, speed_ctrl_args)
```

The modules `sm` (synchronous machine drives), `im` (induction machine drives), and `grid` (grid converter systems) provide `write_model` and `simulate`.
The [examples](examples/) build systems in motulator, write the PLECS models, and, if PLECS Standalone is running with the RPC interface enabled (Preferences > General > RPC interface, port 1080), simulate them in both and print the maximum differences:

```bash
python examples/ipmsm_2kw_fvc.py
```

The written models can also be opened and simulated directly in PLECS:

- The parameters of the control system are in the mask of the control-system subsystem, named after the control method (e.g., `Flux-vector control`), on tabs corresponding to the motulator API. The controller signals are available as probe signals of the subsystem.
- The parameters of the system model are in the initialization commands of the model (Simulation > Simulation Parameters > Initialization), e.g., `machine`, `mechanics`, and `converter`.
- The C-Script blocks include the C files of the package via a path relative to the model file, so a model must be regenerated if it is moved relative to the package.

See [examples/README.md](examples/README.md) for the model structure and the agreement with motulator, and [motulator_plecs/c/README.md](motulator_plecs/c/README.md) for the C port.

## Limitations

- Drives: flux-vector control of synchronous machines (constant inductances or GradNet models, offline reference generation) and current-vector control of induction machines (constant parameters, default observer gain), in the sensorless or sensored mode with speed control
- Grid converters: grid-following current-vector control or disturbance-observer-based grid-forming control in the power-control mode, with an LCL filter (without resistances and grid impedance) or an L filter (with its series resistance and the grid inductance), and a balanced grid with constant voltage and frequency
- Converter: carrier comparison (`pwm=True`) with a stiff, capacitive, or diode-bridge-fed DC bus

Other configurations raise `NotImplementedError` in the writer modules.
