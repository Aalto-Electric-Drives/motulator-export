# motulator-export

**Export of [motulator](https://github.com/Aalto-Electric-Drives/motulator) drive and grid converter systems to PLECS and Simulink, and their native simulation in C**

[![PyPI version](https://img.shields.io/pypi/v/motulator-export.svg)](https://pypi.org/project/motulator-export/)

Documentation: https://aalto-electric-drives.github.io/motulator-export/

Build the system in *motulator* as usual, then:

```python
from motulator_export import native, plecs, simulink

plecs.write_model("drive.plecs", mdl, ctrl, t_stop=1.2)  # PLECS model
simulink.write_model("drive.slx", mdl, ctrl, t_stop=1.2)  # Simulink model
res = native.Simulation(mdl, ctrl).simulate(t_stop=1.2)  # Native simulation
```

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="https://raw.githubusercontent.com/Aalto-Electric-Drives/motulator-export/main/docs/source/images/plecs_drive_black.png">
  <img alt="PLECS model of a 2.2-kW IPMSM drive" src="https://raw.githubusercontent.com/Aalto-Electric-Drives/motulator-export/main/docs/source/images/plecs_drive.png">
</picture>

*PLECS model of a 2.2-kW IPMSM drive with sensorless flux-vector control, written by [examples/ipmsm_2kw_fvc.py](https://github.com/Aalto-Electric-Drives/motulator-export/blob/main/examples/ipmsm_2kw_fvc.py).*

- **One control code everywhere.** The control algorithms of *motulator* are ported to C. The same code runs in PLECS, in Simulink, and in the native simulation.
- **The structure of *motulator*.** Each class of the control system is a masked block with the arguments of its class, nested as in *motulator*. The parameters are set as in a *motulator* script, and they can be changed without Python.
- **Verified.** The PLECS and Simulink models agree with *motulator* to about 1e-6 relative to the signal magnitudes, and the native simulation to the rounding errors.
- **Fast.** The native simulation is 30 to 40 times faster than *motulator*, which is useful, e.g., for parameter sweeps.

## Installation

Install the package from PyPI in a [virtual environment](https://packaging.python.org/en/latest/guides/installing-using-pip-and-virtual-environments/) (*motulator* is installed as a dependency). On Linux and macOS:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install motulator-export
```

On Windows, the environment is created and activated with `py -m venv .venv` and `.venv\Scripts\activate`. The requirements of PLECS, Simulink, and the native simulation, e.g., a C compiler, and the examples of the repository are described in [Getting started](https://aalto-electric-drives.github.io/motulator-export/guide/getting_started.html).

For development, clone the repository and install the package with `pip install -e ".[dev]"`; see [Development](https://aalto-electric-drives.github.io/motulator-export/guide/development.html).
