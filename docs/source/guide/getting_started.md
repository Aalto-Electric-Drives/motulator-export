# Getting started

## Requirements

- Python 3.12 or later. The package installs *motulator* 0.22.0 or a later 0.22 release.
- **PLECS**: PLECS Standalone. To simulate from Python, enable the RPC interface (File > PLECS Preferences > General > RPC interface port, 1080).
- **Simulink**: MATLAB R2021a or later with Simulink, and a C compiler that supports `complex.h`: gcc (Linux), clang (macOS), or MinGW-w64 (Windows, selected with `mex -setup C`). MSVC does not work. The models with MATLAB Function blocks need no compiler. To simulate from Python, install the MATLAB Engine API for Python matching your MATLAB release (e.g., `pip install matlabengine==26.1.*` for R2026a).
- **Native simulation**: a C compiler, by default gcc (or the one given by the environment variable `CC`).

## Installation

Install the package in a [virtual environment](https://packaging.python.org/en/latest/guides/installing-using-pip-and-virtual-environments/). On Linux and macOS:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install motulator-export
```

On Windows:

```powershell
py -m venv .venv
.venv\Scripts\activate
pip install motulator-export
```

If PowerShell does not allow running the activation script, allow it with `Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser`.

The libraries for PLECS and Simulink and the MATLAB classes are in the folder of the installed package, which `python -c "import motulator_export; print(motulator_export.__path__[0])"` prints.

The plots need a GUI library for *matplotlib*. On some Linux systems, it must be installed separately; see the [installation of *motulator*](https://aalto-electric-drives.github.io/motulator/installation.html).

The examples are in the repository. To run them, clone the repository and install the package from it. On Linux and macOS:

```bash
git clone https://github.com/Aalto-Electric-Drives/motulator-export
cd motulator-export
python3 -m venv .venv
source .venv/bin/activate
pip install -e .
```

On Windows, create and activate the virtual environment as above. In [VS Code](https://code.visualstudio.com/docs/python/environments), the virtual environment can also be created with the Python extension. For development, see [](development.md).

## A first model

Each example script builds a system of the *motulator* examples. Run the examples from the root of the repository.

### PLECS

```bash
python examples/ipmsm_2kw_fvc.py
```

Open `examples/ipmsm_2kw_fvc.plecs` in PLECS and start the simulation. If PLECS is running with the RPC interface enabled, the script also simulates the system in PLECS and in *motulator* and prints the differences.

### Simulink

In MATLAB, run `examples/simulink/build_ipmsm_2kw_fvc.m`. It builds the system with the MATLAB classes of *motulator* and the model from the blocks of the library, and saves `examples/simulink/ipmsm_2kw_fvc.slx`. Simulate it in Simulink, or run `res = simulation.simulate(); res.plot();`. The first run builds the library, which takes a few minutes (see [](simulink.md#setup)). Without a C compiler, run `build_ipmsm_2kw_fvc_matlab.m` instead, which builds the model with MATLAB Function blocks.

The scripts are written by the Python scripts from the systems built in *motulator*:

```bash
python examples/ipmsm_2kw_fvc_simulink.py  # With --matlab, build_ipmsm_2kw_fvc_matlab.m
```

With the MATLAB Engine API for Python, the Python script also builds and simulates the model and prints the differences from *motulator*.

### Native simulation

```bash
python examples/native/plot_2kw_ipmsm_fvc.py
```

The script simulates the drive natively and plots the results. After you close the figure, it simulates the same system in *motulator* and prints the run times and the differences. The [examples](../examples/index.rst) include, e.g., a parameter sweep.

### Example scripts

The scripts for PLECS and Simulink are:

| System                                                                    | PLECS                     | Simulink                          |
| ------------------------------------------------------------------------- | ------------------------- | --------------------------------- |
| 2.2-kW IPMSM, sensorless flux-vector control                              | `ipmsm_2kw_fvc.py`        | `ipmsm_2kw_fvc_simulink.py`       |
| 2.2-kW IPMSM, sensorless current-vector control                           | `ipmsm_2kw_cvc.py`        | `ipmsm_2kw_cvc_simulink.py`       |
| 2.2-kW IPMSM, observer-based V/Hz control                                 | `ipmsm_2kw_vhz.py`        | `ipmsm_2kw_vhz_simulink.py`       |
| 5.6-kW PM-SyRM, GradNet models, flux-vector control                       | `pmsyrm_6kw_gn_fvc.py`    | `pmsyrm_6kw_gn_fvc_simulink.py`   |
| 2.2-kW induction machine, current-vector control                          | `im_2kw_cvc.py`           | `im_2kw_cvc_simulink.py`          |
| 2.2-kW induction machine, discrete-time current control in torque control | `im_2kw_cvc_discrete.py`  | `im_2kw_cvc_discrete_simulink.py` |
| 2.2-kW induction machine, flux-vector control                             | `im_2kw_fvc.py`           | `im_2kw_fvc_simulink.py`          |
| 2.2-kW induction machine, dead time and its compensation                  | `im_2kw_dead_time_cvc.py` |                                   |
| 2.2-kW induction machine, observer-based V/Hz control                     | `im_2kw_vhz.py`           | `im_2kw_vhz_simulink.py`          |
| 10-kVA grid converter, LCL filter, grid-following control                 | `gfl_10kva_lcl.py`        | `gfl_10kva_lcl_simulink.py`       |
| 12.5-kVA grid converter, weak grid, grid-forming control                  | `gfm_13kva_do.py`         | `gfm_13kva_do_simulink.py`        |

The MATLAB scripts that build the Simulink models are `examples/simulink/build_<name>.m`, and `build_<name>_matlab.m` with MATLAB Function blocks.

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
    model.VoltageSourceConverter(u_dc=540, modulator=model.CarrierComparison()),
)
cfg = control.FluxVectorControllerCfg(i_s_max=6.5)
ctrl = control.VectorControlSystem(
    control.FluxVectorController(par, cfg),
    control.SpeedController(J=0.015, alpha_s=25.0),
)
ctrl.set_speed_ref(StepSignal(time=0.1, after=50.0))
mdl.mechanics.set_external_load_torque(StepSignal(time=0.7, after=10.0))

plecs.write_model("ipmsm.plecs", mdl, ctrl, t_stop=1.2)
simulink.write_model("ipmsm.slx", mdl, ctrl, t_stop=1.2)  # Writes build_ipmsm.m
```

- The references (e.g., `ctrl.set_speed_ref`, or `ctrl.set_torque_ref` in the torque-control mode without the speed controller), the load torque (or the external rotor speed of `ExternalRotorSpeed`, `set_external_rotor_speed`), and the gate enable (`ctrl.set_enable`) can be constants or `StepSignal` objects. A function of time cannot be exported.
- The models start from the state of the system model, e.g., a given initial speed (`mdl.mechanics.state.w_M = 50.0`). Write the models before simulating the system in *motulator*.
- A system that is not supported raises `NotImplementedError` before any file is written, see [](supported.md).

## Troubleshooting

- **The C compiler is not found** (native simulation): install gcc and put it on the `PATH`, or give the compiler in the environment variable `CC`. On Windows, gcc is included, e.g., in [WinLibs](https://winlibs.com/), [MSYS2](https://www.msys2.org/), and the MinGW-w64 add-on of MATLAB. The compiled libraries are cached in the folder `motulator_export` of the temporary directory.
- **PLECS does not respond** (`PLECS RPC interface not available`): start PLECS Standalone and enable its RPC interface on port 1080.
- **The MATLAB Engine API cannot be imported or started**: install the version of `matlabengine` that matches your MATLAB release.
- **The S-functions of the library cannot be rebuilt** (Windows): run `clear mex` in MATLAB, which unloads the compiled S-functions of a model that was open.
