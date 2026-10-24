# motulator-export

*motulator-export* turns drive systems and grid converter systems built in [*motulator*](https://aalto-electric-drives.github.io/motulator/) into ready-to-run [PLECS](https://www.plexim.com/products/plecs) and [Simulink](https://www.mathworks.com/products/simulink.html) models, and simulates them natively in C. Build the system in *motulator* as usual, then:

```python
from motulator_export import native, plecs, simulink

plecs.write_model("drive.plecs", mdl, ctrl, t_stop=1.2)  # PLECS model
simulink.write_model("drive.slx", mdl, ctrl, t_stop=1.2)  # Script of the Simulink model
res = native.Simulation(mdl, ctrl).simulate(t_stop=1.2)  # Native simulation
```

```{image} images/plecs_drive.png
---
target: _images/plecs_drive.png
alt: PLECS model of a 2.2-kW IPMSM drive
class: only-light
---
```

```{image} images/plecs_drive_black.png
---
target: _images/plecs_drive_black.png
alt: PLECS model of a 2.2-kW IPMSM drive
class: only-dark
---
```

*PLECS model of a 2.2-kW IPMSM drive with sensorless flux-vector control.*

- **One control code everywhere.** The control algorithms of *motulator* are ported to C. The same code runs in PLECS, in Simulink, and in the native simulation.
- **The structure of *motulator*.** Each class of the control system is a masked block with the arguments of its class, nested as in *motulator*. The parameters are set as in a *motulator* script, and they can be changed without Python. The blocks are also in libraries for PLECS and Simulink, and the Simulink models are built from the library by MATLAB scripts that read as *motulator* scripts.
- **Verified.** In the comparisons of the examples, the PLECS and Simulink models agree with *motulator* to about 1e-6 relative to the signal magnitudes, and the native simulation to the rounding errors.
- **Fast.** The native simulation is 30 to 40 times faster than *motulator*, which is useful, e.g., for parameter sweeps.

To start, see [Getting started](guide/getting_started.md). The user guide then covers the [PLECS models](guide/plecs.md), the [Simulink models](guide/simulink.md), the [native simulation](guide/native.md), and the [supported systems](guide/supported.md).

```{note}
The package supports *motulator* 0.17. A new release of *motulator* is supported after it has been tested.
```

```{toctree}
---
hidden:
titlesonly:
caption: User guide
maxdepth: 1
---
guide/getting_started
guide/plecs
guide/simulink
guide/native
guide/supported
```

```{toctree}
---
hidden:
titlesonly:
caption: Examples
maxdepth: 1
---
examples/index
```

```{toctree}
---
hidden:
titlesonly:
caption: Reference
maxdepth: 1
---
API <autoapi/motulator_export/index>
guide/development
motulator <https://aalto-electric-drives.github.io/motulator/>
```
