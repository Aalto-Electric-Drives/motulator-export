# Supported systems

| Feature                                                                    | PLECS  | Simulink | Native |
| -------------------------------------------------------------------------- | :----: | :------: | :----: |
| Synchronous machine drives: flux-vector control (also with GradNet models) |  yes   |   yes    |  yes   |
| Induction machine drives: current-vector control                           |  yes   |   yes    |  yes   |
| Induction machine drives: observer-based and open-loop V/Hz control        |  yes   |   yes    |  yes   |
| Grid-following control                                                     |  yes   |   yes    |  yes   |
| Grid-forming control (disturbance observer)                                |  yes   |   yes    |  yes   |
| Stiff DC bus                                                               |  yes   |   yes    |  yes   |
| Capacitive or diode-bridge-fed DC bus (`DCBus`, `DiodeBridgeDCBus`)        | drives |    no    |  yes   |
| Dead time and threshold voltage of the converter                           | drives |    no    |  yes   |
| Their compensation in the control system (drives)                          |  yes   |   yes    |  yes   |
| Single update in the PWM of the control system (`PWM(single_update=True)`) |  yes   |   yes    |  yes   |
| Single update in the system model (`single_update`)                        |  yes   |    no    |  yes   |
| Gate blocking (`set_enable`)                                               |  yes   |    no    |   no   |

The drives can be sensorless or sensored, and the induction machine models have constant parameters. Other configurations raise `NotImplementedError` when the model is written or the native simulation is created.

**PLECS and Simulink.** The converter uses carrier comparison (`pwm=True`) with the computational delay of one sampling period. Grid-following control is supported with an LCL filter without resistances and grid impedance, and grid-forming control with an L filter and the grid inductance, without the grid resistance. The grid voltage is balanced, with constant magnitude and frequency. The Simulink control systems support the standby while the gates are blocked, but the Simulink system model does not block the gates, so a system with gate blocking cannot be exported to Simulink. In PLECS, gate blocking is not supported with the GradNet machine model.

**Native simulation.** It is more general: it also supports the zero-order hold (the default of *motulator*), other computational delays, both filters with their resistances and the grid impedance with either control, grid voltages that vary in time (e.g., voltage dips and unbalance), and the friction of the mechanics as a function of the speed.

**MATLAB classes.** The Simulink models are built with these MATLAB classes of *motulator* (see [](simulink.md)). `Drive` and `GridConverterSystem` model the carrier comparison (`pwm=True` of *motulator*), and `StepSignal` is that of *motulator-export*:

| Package                                         | Classes                                                                                                                                                                                                                                                                      |
| ----------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `motulator.drive.model`                         | `SynchronousMachinePars`, `SaturatedSynchronousMachinePars`, `SpatialSaturatedSynchronousMachinePars`, `InductionMachinePars`, `InductionMachineInvGammaPars`, `SynchronousMachine`, `InductionMachine`, `MechanicalSystem`, `VoltageSourceConverter`, `Drive`, `Simulation` |
| `motulator.drive.control.sm`                    | `FluxVectorControllerCfg`, `FluxVectorController`, `SpeedController`, `VectorControlSystem`, `PWM`, `InverterNonlinearity`                                                                                                                                                   |
| `motulator.drive.control.im`                    | `CurrentVectorControllerCfg`, `CurrentVectorController`, `ObserverBasedVHzControllerCfg`, `ObserverBasedVHzController`, `SpeedController`, `VectorControlSystem`, `VHzControlSystem`, `PWM`, `InverterNonlinearity`                                                          |
| `motulator.grid.model`                          | `LCLFilter`, `LFilter`, `ThreePhaseSource`, `VoltageSourceConverter`, `GridConverterSystem`, `Simulation`                                                                                                                                                                    |
| `motulator.grid.control`                        | `CurrentVectorControllerCfg`, `CurrentVectorController`, `ObserverBasedGridFormingControllerCfg`, `ObserverBasedGridFormingController`, `GridConverterControlSystem`, `PWM`, `InverterNonlinearity`                                                                          |
| `motulator.drive.utils`, `motulator.grid.utils` | `BaseValues`, `NominalValues`, `StepSignal`                                                                                                                                                                                                                                  |
