"""
2.2-kW IM, discrete-time current control: motulator vs. PLECS
=============================================================

This script builds the drive system of the example
examples/drive/current_vector/plot_2kw_im_cvc_discrete.py with sensorless
current-vector control of a 2.2-kW induction machine (IM) and the direct
discrete-time current controller (discrete=True), exports it to a PLECS model
(im_2kw_cvc_discrete.plecs), and, if PLECS Standalone is running with the RPC
interface enabled, simulates both and compares the results. The drive is in the
torque-control mode (without the speed controller) at the constant external rotor
speed of 0.5 p.u. (ExternalRotorSpeed), and the observer starts from this speed
(vector_ctrl.observer.w_M). The sampling frequency is 5 kHz and the current-control
bandwidth 2*pi*700 rad/s.

Run from the repository root:

    python examples/im_2kw_cvc_discrete.py

"""

# %%
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

T_STOP = 0.37
nom = utils.NominalValues(U=400, I=5, f=50, P=2.2e3, tau=14.6)
base = utils.BaseValues.from_nominal(nom, n_p=2)
# Torque reference (Nm), stepping as in the example, and the external rotor speed
TAU_M_REF = StepSignal(
    time=[0.3, 0.32, 0.34], after=[0.3 * nom.tau, 0.5 * nom.tau, -0.3 * nom.tau]
)
W_M_EXT = 0.5 * base.w_M  # (rad/s)


# %%
def build_system() -> tuple[model.Drive, control.VectorControlSystem]:
    """
    Build the drive system of the motulator example with the discrete-time design of
    the current controller, the observer starting from the external rotor speed.
    """
    par = model.InductionMachineInvGammaPars(
        n_p=2, R_s=3.7, R_R=2.1, L_sgm=0.021, L_M=0.224
    )
    mdl = model.Drive(
        model.InductionMachine(par),
        model.ExternalRotorSpeed(),
        # Fine quantization (not modeled in PLECS)
        model.VoltageSourceConverter(
            u_dc=540, modulator=model.CarrierComparison(N=2**24)
        ),
    )
    cfg = control.CurrentVectorControllerCfg(
        psi_s_nom=base.psi,
        i_s_max=2 * base.i,
        alpha_c=2 * pi * 700,
        T_s=200e-6,
        discrete=True,
    )
    vector_ctrl = control.CurrentVectorController(par, cfg)
    vector_ctrl.observer.w_M = W_M_EXT  # Initial speed estimate
    ctrl = control.VectorControlSystem(vector_ctrl)
    return mdl, ctrl


# %%
if __name__ == "__main__":
    mdl, ctrl = build_system()
    ctrl.set_torque_ref(TAU_M_REF)
    mdl.mechanics.set_external_rotor_speed(W_M_EXT)
    path = Path(__file__).with_name("im_2kw_cvc_discrete.plecs")
    tmp = write_plecs_model(path, mdl, ctrl, T_STOP)

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
