# Getting started

## Requirements

- Python 3.12 or later. The package installs *motulator* 0.15.2 or a later 0.15 release.
- **PLECS**: PLECS Standalone. To simulate from Python, enable the RPC interface (Preferences > General > RPC interface, port 1080).
- **Simulink**: MATLAB with Simulink, and a C compiler that supports `complex.h`: gcc (Linux), clang (macOS), or MinGW-w64 (Windows, selected with `mex -setup C`). MSVC does not work. The models with MATLAB Function blocks need no compiler. To simulate from Python, install the MATLAB Engine API for Python matching your MATLAB release (e.g., `pip install matlabengine==26.1.*` for R2026a).
- **Native simulation**: a C compiler, by default gcc (or the one given by the environment variable `CC`).

## Installation

Clone the repository, which contains the examples, and install the package in a virtual environment:

```bash
git clone https://github.com/Aalto-Electric-Drives/motulator-export
cd motulator-export
python -m venv .venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate
python -m pip install -e .
```

## A first model

Each example script builds a system of the *motulator* examples. Run the examples from the root of the repository.

### PLECS

```bash
python examples/ipmsm_2kw_fvc.py
```

Open `examples/ipmsm_2kw_fvc.plecs` in PLECS and start the simulation. If PLECS is running with the RPC interface enabled, the script also simulates the system in PLECS and in *motulator* and prints the differences.

### Simulink

```bash
python examples/ipmsm_2kw_fvc_simulink.py
```

In MATLAB, run `examples/simulink/build_ipmsm_2kw_fvc.m`, which compiles the blocks of the control system and saves `ipmsm_2kw_fvc.slx`. Without a C compiler, use MATLAB Function blocks instead:

```bash
python examples/ipmsm_2kw_fvc_simulink.py --matlab
```

and run `examples/simulink/build_ipmsm_2kw_fvc_matlab.m`, which saves `ipmsm_2kw_fvc_matlab.slx`. With the MATLAB Engine API for Python, the script also builds and simulates the model and prints the differences from *motulator*.

### Native simulation

```bash
python examples/native/plot_2kw_ipmsm_fvc.py
```

The script simulates the drive natively and plots the results. After you close the figure, it simulates the same system in *motulator* and prints the run times and the differences. The [examples](../examples/index.rst) include, e.g., a parameter sweep.

### Example scripts

The scripts for PLECS and Simulink are:

| System                                                    | PLECS                     | Simulink                        |
| --------------------------------------------------------- | ------------------------- | ------------------------------- |
| 2.2-kW IPMSM, sensorless flux-vector control              | `ipmsm_2kw_fvc.py`        | `ipmsm_2kw_fvc_simulink.py`     |
| 5.6-kW PM-SyRM, GradNet models, flux-vector control       | `pmsyrm_6kw_gn_fvc.py`    | `pmsyrm_6kw_gn_fvc_simulink.py` |
| 2.2-kW induction machine, current-vector control          | `im_2kw_cvc.py`           | `im_2kw_cvc_simulink.py`        |
| 2.2-kW induction machine, dead time and its compensation  | `im_2kw_dead_time_cvc.py` |                                 |
| 2.2-kW induction machine, observer-based V/Hz control     | `im_2kw_vhz.py`           | `im_2kw_vhz_simulink.py`        |
| 10-kVA grid converter, LCL filter, grid-following control | `gfl_10kva_lcl.py`        | `gfl_10kva_lcl_simulink.py`     |
| 12.5-kVA grid converter, weak grid, grid-forming control  | `gfm_13kva_do.py`         | `gfm_13kva_do_simulink.py`      |

## Exporting your own system

Build the system in *motulator*, set its references, and write the models:

```python
import motulator.drive.control.sm as control
from motulator.drive import model

from motulator_export import StepSignal, plecs, simulink

par = model.SynchronousMachinePars(n_p=3, R_s=3.6, L_d=0.036, L_q=0.051, psi_f=0.545)
mdl = model.Drive(
    model.SynchronousMachine(par),
    model.MechanicalSystem(J=0.015),
    model.VoltageSourceConverter(u_dc=540),
    pwm=True,
)
cfg = control.FluxVectorControllerCfg(i_s_max=6.5)
ctrl = control.VectorControlSystem(
    control.FluxVectorController(par, cfg),
    control.SpeedController(J=0.015, alpha_s=25.0),
)
ctrl.set_speed_ref(StepSignal(time=0.1, after=50.0))
mdl.mechanics.set_external_load_torque(StepSignal(time=0.7, after=10.0))

plecs.write_model("ipmsm.plecs", mdl, ctrl, t_stop=1.2)
simulink.write_model("ipmsm.slx", mdl, ctrl, t_stop=1.2)
```

- The references, the load torque, and the gate enable (`ctrl.set_enable`) can be constants or `StepSignal` objects. A function of time cannot be exported.
- The models start from the state of the system model, e.g., a given initial speed (`mdl.mechanics.state.w_M = 50.0`). Write the models before simulating the system in *motulator*.
- A system that is not supported raises `NotImplementedError` before any file is written.
