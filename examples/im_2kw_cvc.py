"""
2.2-kW IM, sensorless CVC: motulator vs. PLECS
==============================================

This script builds a 2.2-kW induction machine (IM) drive with sensorless current-
vector control (CVC), corresponding to the example
examples/drive/current_vector/plot_2kw_im_sat_cvc.py with the constant-parameter
machine model, exports it to a PLECS model (im_2kw_cvc.plecs), and, if PLECS
Standalone is running with the RPC interface enabled, simulates both and compares
the results.

Run from the repository root:

    python examples/im_2kw_cvc.py

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

T_STOP = 1.5
nom = utils.NominalValues(U=400, I=5, f=50, P=2.2e3, tau=14.6)
base = utils.BaseValues.from_nominal(nom, n_p=2)
W_M_REF = StepSignal(time=0.2, after=0.5 * base.w_M)  # Speed reference (rad/s)
TAU_L = StepSignal(time=0.75, after=nom.tau)  # Load torque (Nm)


# %%
def build_system() -> tuple[model.Drive, control.VectorControlSystem]:
    """Build the drive system of the motulator example."""
    par = model.InductionMachineInvGammaPars(
        n_p=2, R_s=3.7, R_R=2.1, L_sgm=0.021, L_M=0.224
    )
    mdl = model.Drive(
        model.InductionMachine(par),
        model.MechanicalSystem(J=0.015),
        # Fine quantization (not modeled in PLECS)
        model.VoltageSourceConverter(
            u_dc=540, modulator=model.CarrierComparison(N=2**24)
        ),
    )
    est_par = control.InductionMachineInvGammaPars(
        n_p=2, R_s=3.7, R_R=2.1, L_sgm=0.021, L_M=0.224
    )
    cfg = control.CurrentVectorControllerCfg(
        psi_s_nom=base.psi, i_s_max=1.5 * base.i, sensorless=True
    )
    ctrl = control.VectorControlSystem(
        control.CurrentVectorController(est_par, cfg),
        control.SpeedController(J=0.015, alpha_s=2 * pi * 4),
    )
    return mdl, ctrl


# %%
if __name__ == "__main__":
    mdl, ctrl = build_system()
    ctrl.set_speed_ref(W_M_REF)
    mdl.mechanics.set_external_load_torque(TAU_L)
    path = Path(__file__).with_name("im_2kw_cvc.plecs")
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
