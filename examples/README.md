# Examples

Each script builds a system of a *motulator* example, writes the PLECS model (`*.plecs` in this directory), and, if PLECS Standalone is running with the RPC interface enabled, simulates the system in both *motulator* and PLECS and prints the maximum differences.
The scripts `*_simulink.py` do the same for Simulink, writing the build scripts of the models in `simulink/`; see [the Simulink export](../motulator_export/simulink/README.md).

| Script                    | System                                                                                                                                                                                                                                                           |
| ------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `ipmsm_2kw_fvc.py`        | 2.2-kW IPMSM, sensorless flux-vector control (the README example of *motulator*); with `--diode`, a diode bridge with a DC-bus inductor and capacitor                                                                                                            |
| `pmsyrm_6kw_gn_fvc.py`    | 5.6-kW PM-SyRM, GradNet models from FEM data (in `trained_models/`), sensored flux-vector control (`plot_6kw_pmsyrm_gn_fvc_fem_harm.py`)                                                                                                                         |
| `im_2kw_cvc.py`           | 2.2-kW induction machine, sensorless current-vector control (`plot_2kw_im_sat_cvc.py` with the constant-parameter machine model)                                                                                                                                 |
| `im_2kw_dead_time_cvc.py` | 2.2-kW induction machine, dead time and its compensation, sensorless current-vector control at low speeds (`plot_2kw_im_dead_time_cvc.py` with the constant-parameter machine model and `sign=np.sign`); with `--tanh`, the compensation uses `sign=tanh(i/i_0)` |
| `gfl_10kva_lcl.py`        | 10-kVA grid converter with an LCL filter, grid-following control (`plot_10kva_lcl_gfl.py`)                                                                                                                                                                       |
| `gfm_13kva_do.py`         | 12.5-kVA grid converter with an L filter in a weak grid, disturbance-observer-based grid-forming control (`plot_13kva_do_gfm.py`)                                                                                                                                |

Run the scripts from the repository root, e.g., `python examples/ipmsm_2kw_fvc.py`.
For the comparison, a script writes a temporary copy of the model (`*_tmp.plecs`, removed at exit) with output ports, through which the RPC interface returns the signals; the models in this directory have no output ports.
The scripts close the model in PLECS before simulating it, since an open model is not reloaded from the file.

## Structure of the PLECS models

- The references (e.g., the speed reference or the power references) and the load torque are Step blocks; several steps are given as vector parameters of one Step block and summed by a Gain block. A constant reference (the converter voltage reference of grid-forming control) is a Constant block.
- The control system samples the references and the measurements (e.g., the phase currents, the DC-bus voltage, and the rotor angle or speed in the sensored mode) with the sampling period `T_s`. The duty ratios pass through a Delay block, which models the computational delay of one sampling period, as in *motulator*.
- The Symmetrical PWM block of PLECS (regular sampling with double update, carrier frequency `1/(2*T_s)`) generates the gate signals of the ideal two-level converter of PLECS. This corresponds to carrier comparison in *motulator* (`pwm=True`), except that the counter quantization of the duty ratios is not modeled. The scripts therefore use a fine quantization in *motulator* (`CarrierComparison(N=2**24)`); with the default quantization of 4096 levels, the differences are about 1e-2.
- With the dead time of the converter (`t_d`, induction machine drives only), the Blanking Time block of PLECS delays the turn-on of the switches by `t_d`, and the converter is the IGBT converter of PLECS with ideal IGBTs and diodes: during blanking, both switches of the leg are off, and the current direction determines the conducting diode, as in *motulator* with `sign=np.sign`. The duty-ratio error model of the PWM of the control system (`d_err = dead_time_error(i_abc, d_abc, t_d, T_s, sign)`) is given by the dead time `t_d` and the current scale `i_0` in the mask, and the compensation can be disabled (`feedforward`). The current direction in the control system is `sign = np.sign` (`i_0 = 0`, by default in the examples) or the smooth `sign = tanh(i/i_0)`, which avoids the chattering of the compensation due to the current ripple and the measurement noise near the zero crossings of the currents; `i_0` can be chosen to be about half the peak-to-peak current ripple.
- The synchronous machine is the PLECS permanent-magnet synchronous machine (`SynchronousMachinePars`) or a subsystem that looks like it (`SpatialSaturatedSynchronousMachinePars` with a GradNet current map with spatial harmonics): inside, a C-Script block computes the currents and the torque from the flux linkage, and they are injected with controlled current and torque sources.
- The induction machine is the PLECS squirrel-cage induction machine, parametrized as the T model with zero rotor leakage inductance, which is identical to the inverse-Γ model of *motulator*.
- The DC bus is a DC voltage source, a capacitor, or a diode bridge fed by the grid with a DC-bus inductor and capacitor (`FrequencyConverter`, zero grid inductance).
- The AC filter of the grid converter is an LCL filter or an L filter with its series resistance and the grid inductance (`LFilter`), with the grid as three AC voltage sources.
- The circuits are not grounded: as in the space-vector models of *motulator*, there is no zero-sequence path (grounding both the DC bus and the grid neutral would let the common-mode voltage of the converter drive a zero-sequence current).
- Voltmeters and ammeters are used only for the signals fed back to the control system. The AC voltages are measured line to line (u_ab and u_bc), as in practice. The other signals, e.g., the inductor currents shown in the scope, are measured with PLECS probes.
- A GradNet flux map of the control system is given as a struct in the model workspace (`est_flux_map`).

## Agreement of the PLECS models with *motulator*

Maximum differences (PLECS − *motulator*) printed by the scripts, with *motulator* 0.8.2 and PLECS 5.0 (2026-10-05).
*motulator* uses the tolerances of 1e-9 (1e-8 in `pmsyrm_6kw_gn_fvc.py`), and PLECS the variable-step Dormand–Prince solver with the relative tolerance of 1e-6 and the maximum step `T_s`.
After a change in the writers or the C port, regenerate the models by running the scripts and check that the differences stay at this level.

| Script                           | System model                               | Control system                                               |
| -------------------------------- | ------------------------------------------ | ------------------------------------------------------------ |
| `ipmsm_2kw_fvc.py`               | w_M 7.5e-6, tau_M 5.4e-6, i_s_ab 2.2e-6    | w_M 6.8e-6, tau_M 5.4e-6, tau_M_ref 5.4e-6, psi_s_ref 1.3e-8 |
| `ipmsm_2kw_fvc.py --diode`       | w_M 1.6e-6, tau_M 2.8e-6, i_s_ab 1.3e-6    | w_M 4.0e-6, tau_M 2.8e-6, tau_M_ref 3.0e-6, psi_s_ref 9.1e-9 |
| `pmsyrm_6kw_gn_fvc.py`           | w_M 3.4e-4, tau_M 2.1e-3, i_s_ab 7.0e-4    |                                                              |
| `im_2kw_cvc.py`                  | w_M 7.1e-7, tau_M 2.3e-6, i_s_ab 8.3e-7    | w_M 3.9e-6, tau_M 2.6e-6, tau_M_ref 2.9e-6, psi_R 3.2e-8     |
| `im_2kw_dead_time_cvc.py`        | w_M 0.23, tau_M 0.75, i_s_ab 0.50          | w_M 1.4, tau_M 0.90, tau_M_ref 1.1, psi_R 7.1e-3             |
| `im_2kw_dead_time_cvc.py --tanh` | w_M 0.28, tau_M 0.48, i_s_ab 0.18          | w_M 0.66, tau_M 0.38, tau_M_ref 0.44, psi_R 6.1e-3           |
| `gfl_10kva_lcl.py`               | i_c_ab 3.1e-6, i_g_ab 2.2e-6, i_c_a 3.1e-6 | p_g 1.3e-3, q_g 7.4e-4, u_g 1.7e-13, w_g 1.1e-12             |
| `gfm_13kva_do.py`                | i_c_ab 9.4e-7, i_c_a 8.0e-7                | p_g 3.9e-4, q_g 4.0e-4, v_c 2.7e-6, theta_c 0                |

The differences are in SI units; relative to the signal magnitudes, they are about 1e-6 or below (e.g., 1e-3 W of 10 kW).
In `pmsyrm_6kw_gn_fvc.py`, the differences are larger (about 1e-4 relative), since *motulator* evaluates the GradNets in single precision, see [the C port](../motulator_export/c/README.md).
In `im_2kw_dead_time_cvc.py`, the differences are due to the switching transients during blanking, which *motulator* does not model in detail.
*motulator* determines the state of a blanked leg from the current direction in the beginning of the blanking interval, whereas in PLECS, the current of a blanked leg cannot cross zero, since its diode turns off.
At low speeds, this occurs often due to the current ripple near the zero crossings of the phase currents, and the sensorless low-speed operation amplifies the differences (e.g., 1.4 rad/s in the speed estimate during transients, compared with the speed reference of 7.9 rad/s).
Furthermore, at the first switching from zero currents, the blanked leg is open in PLECS.
With `T_D = 1e-9` in the script, the differences are at the level of the other examples (e.g., w_M 1.4e-6, i_s_ab 8.5e-6, ctrl.w_M 3.7e-6), which verifies the export of the dead time and its compensation.
With `--tanh` (`I_0 = 0.1` A, about half the peak-to-peak current ripple of 0.15–0.28 A), the compensation is less sensitive to these differences near the zero crossings: the differences in the currents, the torques, and the speed estimate are about a half or less of those with the signum function, whereas those in the speed and in the flux estimate are similar.
