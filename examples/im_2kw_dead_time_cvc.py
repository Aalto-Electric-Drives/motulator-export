"""
2.2-kW IM, dead time, sensorless CVC: motulator vs. PLECS
=========================================================

This script builds a 2.2-kW induction machine (IM) drive with sensorless current-
vector control (CVC) at low speeds, corresponding to the example
examples/drive/current_vector/plot_2kw_im_dead_time_cvc.py with the constant-
parameter machine model and the signum function as the current-direction function.
The dead time and the threshold voltage of the converter are modeled, and their
effects are compensated for in the control system. The duty ratios are updated once
per carrier period (single update), so the sampling period equals the carrier
period. The script exports the system to a PLECS model (im_2kw_dead_time_cvc.plecs),
where the dead time is modeled with the Blanking Time block and the threshold voltage
with the forward voltages of the IGBT converter with parasitics of PLECS, and, if
PLECS Standalone is running with the RPC interface enabled, simulates both and
compares the results.

With --tanh, the current direction in the compensation of the control system is the
smooth function tanh(i/i_0) instead of the signum function, as on hardware, where the
current ripple and the measurement noise near the zero crossings would make the
signum function chatter (im_2kw_dead_time_cvc_tanh.plecs). The converter model still
uses the signum function.

Run from the repository root:

    python examples/im_2kw_dead_time_cvc.py [--tanh]

"""

# %%
import sys
from math import pi
from pathlib import Path

import motulator.drive.control.im as control
import numpy as np
from _compare import (
    Comparison,
    drive_signals,
    plot_drive,
    simulate_motulator,
    simulate_plecs,
    write_plecs_model,
)
from motulator.drive import model, utils

from motulator_export import StepSignal

T_STOP = 2.4
nom = utils.NominalValues(U=400, I=5, f=50, P=2.2e3, tau=14.6)
base = utils.BaseValues.from_nominal(nom, n_p=2)
# Speed reference (rad/s), reversed under the load torque (Nm), leading to the
# regenerating mode
W_M_REF = StepSignal(time=[0.2, 1.4], after=[0.05 * base.w_M, -0.05 * base.w_M])
TAU_L = StepSignal(time=0.8, after=0.7 * nom.tau)
T_D = 2e-6  # Dead time (s)
U_TH = 1.0  # Threshold voltage (V)
# Sampling period (s), the default value in CurrentVectorControllerCfg, equal to the
# carrier period with single update
T_S = 125e-6
I_0 = 0.05  # Current scale (A) of tanh(i/i_0), about half the peak-to-peak ripple


# %%
def build_system(tanh: bool = False) -> tuple[model.Drive, control.VectorControlSystem]:
    """
    Build the drive system of the motulator example.

    With `tanh`, the compensation uses the current-direction function `tanh(i/I_0)`
    instead of `np.sign`.

    """
    par = model.InductionMachineInvGammaPars(
        n_p=2, R_s=3.7, R_R=2.1, L_sgm=0.021, L_M=0.224
    )
    # Single update with the computational delay of one sampling period, i.e., two
    # half carrier periods, and fine quantization (not modeled in PLECS)
    modulator = model.CarrierComparison(t_d=T_D, single_update=True, N=2**24)
    mdl = model.Drive(
        model.InductionMachine(par),
        model.MechanicalSystem(J=0.015),
        model.VoltageSourceConverter(u_dc=540, modulator=modulator, u_th=U_TH),
    )
    est_par = control.InductionMachineInvGammaPars(
        n_p=2, R_s=3.7, R_R=2.1, L_sgm=0.021, L_M=0.224
    )
    cfg = control.CurrentVectorControllerCfg(
        psi_s_nom=0.95 * base.psi, i_s_max=1.5 * base.i, sensorless=True, T_s=T_S
    )
    # The duty-ratio error model of the system model, compensated for in the PWM
    # configured for single update, optionally with a smooth current-direction
    # function
    sign = (lambda i: np.tanh(i / I_0)) if tanh else np.sign
    d_err = control.InverterNonlinearity(T_D, U_TH, sign)
    pwm = control.PWM(d_err=d_err, single_update=True)
    ctrl = control.VectorControlSystem(
        control.CurrentVectorController(est_par, cfg),
        control.SpeedController(J=0.015, alpha_s=2 * pi * 4),
        pwm,
    )
    return mdl, ctrl


# %%
if __name__ == "__main__":
    tanh = "--tanh" in sys.argv  # Smooth current direction in the compensation
    mdl, ctrl = build_system(tanh)
    ctrl.set_speed_ref(W_M_REF)
    mdl.mechanics.set_external_load_torque(TAU_L)
    name = "im_2kw_dead_time_cvc_tanh" if tanh else "im_2kw_dead_time_cvc"
    tmp = write_plecs_model(
        Path(__file__).with_name(f"{name}.plecs"), mdl, ctrl, T_STOP
    )

    # Simulate in motulator and in PLECS, and compare
    res = simulate_motulator(mdl, ctrl, T_STOP)
    cmp = Comparison(res, T_STOP)
    plecs_signals = simulate_plecs(tmp, cmp.t_eval, mdl, ctrl)
    fbk, ref = res.ctrl.fbk, res.ctrl.ref
    ctrl_signals = {
        "w_M": fbk.w_M,
        "tau_M": fbk.tau_M,
        "tau_M_ref": ref.tau_M,
        "psi_R": np.abs(fbk.psi_R),
    }
    cmp.print_differences("PLECS", plecs_signals, drive_signals(res), ctrl_signals)
    plot_drive(cmp, res, plecs_signals[0], "PLECS")
