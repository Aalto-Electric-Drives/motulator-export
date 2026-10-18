# *motulator-export:* PLECS, Simulink, and Native Simulation of *motulator* Systems

This package exports drive systems and grid converter systems built in [*motulator*](https://aalto-electric-drives.github.io/motulator/) to [PLECS](https://www.plexim.com/products/plecs) and [Simulink](https://www.mathworks.com/products/simulink.html), and simulates them natively in C. You build a system in *motulator*, as usual, and get a ready-to-run simulation model of it. The control algorithms of *motulator* are ported to C, and the same C code runs in the C-Script blocks of PLECS, in the S-functions of Simulink, and in the native simulation; a MATLAB port of the same code runs in MATLAB Function blocks. Each class of the control system is a block with a mask of the arguments of its class, in the hierarchy of *motulator*, and the parameters are defined as in a *motulator* example. The models and the native simulation are verified against *motulator*.

The native simulation (`native.Simulation`) has the interface and the results of the simulation of *motulator*, but it is 30 to 40 times faster, which is useful, e.g., for parameter sweeps. See the [examples](examples/index.rst).

```{image} ../../examples/pmsyrm_6kw_gn_fvc.png
---
alt: PLECS model of the 5.6-kW PM-SyRM drive with flux-vector control
class: only-light
---
```

```{image} ../../examples/pmsyrm_6kw_gn_fvc_black.png
---
alt: PLECS model of the 5.6-kW PM-SyRM drive with flux-vector control
class: only-dark
---
```

*PLECS model of a 5.6-kW PM-SyRM drive: the flux-vector control subsystem, the computational delay, the PWM, the converter, the machine with a GradNet model, the mechanics, and the scope.*

```{note}
The package supports *motulator* 0.15. A new release of *motulator* is supported after it has been tested.
```

```{toctree}
---
titlesonly:
caption: Getting Started
name: getting_started
maxdepth: 1
---
pages/installation
pages/export
pages/native
pages/supported
```

```{toctree}
---
titlesonly:
caption: Examples
name: examples
maxdepth: 1
---
examples/index
pages/plecs
```

```{toctree}
---
titlesonly:
caption: Details
name: details
maxdepth: 1
---
pages/simulink
pages/c_port
pages/matlab_port
pages/development
```

```{toctree}
---
titlesonly:
caption: Reference
name: reference
maxdepth: 1
---
API <autoapi/motulator_export/index>
motulator <https://aalto-electric-drives.github.io/motulator/>
```
