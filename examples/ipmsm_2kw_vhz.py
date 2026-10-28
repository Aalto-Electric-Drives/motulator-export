"""
2.2-kW IPMSM, observer-based V/Hz control: motulator vs. PLECS
==============================================================

This script builds a 2.2-kW interior permanent-magnet synchronous machine (IPMSM)
drive with observer-based V/Hz control, corresponding to the example
examples/drive/vhz/plot_2kw_ipmsm_ovhz.py (with the speed reference stepped and
rate-limited instead of ramped), exports it to a PLECS model (ipmsm_2kw_vhz.plecs),
and, if PLECS Standalone is running with the RPC interface enabled, simulates both
and compares the results.

Run from the repository root:

    python examples/ipmsm_2kw_vhz.py

"""

# %%
from math import pi
from pathlib import Path

import motulator.drive.control.sm as control
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

T_STOP = 1.6
nom = utils.NominalValues(U=370, I=4.3, f=75, P=2.2e3, tau=14)
base = utils.BaseValues.from_nominal(nom, n_p=3)
W_M_REF = StepSignal(time=0.2, after=base.w_M)  # Speed reference (rad/s)
TAU_L = StepSignal(time=0.8, after=nom.tau)  # Load torque (Nm)


# %%
def build_system() -> tuple[model.Drive, control.VHzControlSystem]:
    """Build the drive system of the motulator example."""
    par = model.SynchronousMachinePars(
        n_p=3, R_s=3.6, L_d=0.036, L_q=0.051, psi_f=0.545
    )
    mdl = model.Drive(
        model.SynchronousMachine(par),
        model.MechanicalSystem(J=0.015),
        # Fine quantization (not modeled in PLECS)
        model.VoltageSourceConverter(
            u_dc=540, modulator=model.CarrierComparison(N=2**24)
        ),
    )
    cfg = control.ObserverBasedVHzControllerCfg(i_s_max=1.5 * base.i)
    vhz_ctrl = control.ObserverBasedVHzController(par, cfg)
    ctrl = control.VHzControlSystem(vhz_ctrl, slew_rate=2 * pi * 120)
    return mdl, ctrl


# %%
if __name__ == "__main__":
    mdl, ctrl = build_system()
    ctrl.set_speed_ref(W_M_REF)
    mdl.mechanics.set_external_load_torque(TAU_L)
    path = Path(__file__).with_name("ipmsm_2kw_vhz.plecs")
    tmp = write_plecs_model(path, mdl, ctrl, T_STOP)

    # Simulate in motulator and in PLECS, and compare
    res = simulate_motulator(mdl, ctrl, T_STOP)
    cmp = Comparison(res, T_STOP)
    plecs_signals = simulate_plecs(tmp, cmp.t_eval, mdl, ctrl)
    fbk, ref = res.ctrl.fbk, res.ctrl.ref
    ctrl_signals = {
        "w_c": fbk.w_c,
        "tau_M": fbk.tau_M,
        "tau_M_ref": ref.tau_M,
        "psi_s": np.abs(fbk.psi_s),
    }
    cmp.print_differences("PLECS", plecs_signals, drive_signals(res), ctrl_signals)
    plot_drive(cmp, res, plecs_signals[0], "PLECS")
