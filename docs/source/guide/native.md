# Native simulation

`native.Simulation` simulates a system built in *motulator* in C. It has the interface and the results of the simulation of *motulator*, so a script only replaces the simulation object:

```python
from motulator.drive import utils

from motulator_export import native

sim = native.Simulation(mdl, ctrl)  # Instead of model.Simulation(mdl, ctrl)
res = sim.simulate(t_stop=1.75)
utils.plot(res, base)
```

The control system runs the C port of its algorithms, and the system model runs as C models of its subsystems, integrated with the same solver steps as in *motulator*. The C code is compiled at the first use and cached. The [examples](../examples/index.rst) show the native simulation of systems of the *motulator* examples, including a parameter sweep.

## Run times

Run times of `simulate` on one core of a laptop, with *motulator* 0.15.0:

| System                                       | PWM model          | *motulator* | Native | Ratio |
| -------------------------------------------- | ------------------ | ----------: | -----: | ----: |
| 5.6-kW PM-SyRM, GradNet models, 1.75 s       | zero-order hold    |       9.9 s | 0.31 s |    32 |
| 5.6-kW PM-SyRM, GradNet models, 1.75 s       | carrier comparison |      27.9 s | 0.87 s |    32 |
| 2.2-kW IPMSM, sensorless FVC, 1.6 s          | zero-order hold    |       3.3 s | 0.09 s |    36 |
| 2.2-kW IPMSM, sensorless FVC, 1.6 s          | carrier comparison |       9.1 s | 0.24 s |    38 |

## Accuracy

The results agree with *motulator* to the rounding errors, about 1e-12 relative to the signal magnitudes. With carrier comparison (`pwm=True`) or a diode bridge (`DiodeBridgeDCBus`), a rounding error can change a switching instant or a step size, after which the results differ more. The simulations of *motulator* itself are equally sensitive to the rounding errors.

## Good to know

- The references and the load torque can be constants, `StepSignal` objects (evaluated in C, fastest), or functions of time (called from C).
- As in *motulator*, the results are also stored in `mdl` and `ctrl`, and a subsequent call of `simulate` continues the simulation.
- Gate blocking (`ctrl.set_enable`) is not supported. Other unsupported systems raise `NotImplementedError` when the simulation is created, see [](supported.md).
