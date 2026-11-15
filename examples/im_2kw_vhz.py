"""
2.2-kW IM, observer-based or open-loop V/Hz control: motulator vs. PLECS
========================================================================

This script builds a 2.2-kW induction machine (IM) drive with observer-based V/Hz
control, corresponding to the example examples/drive/vhz/plot_2kw_im_ovhz.py, exports
it to a PLECS model (im_2kw_vhz.plecs), and, if PLECS Standalone is running with the
RPC interface enabled, simulates both and compares the results.

With --open-loop, the control system is open-loop V/Hz control with the linear V/Hz
curve, as in examples/drive/grid_connection/plot_2kw_im_diode_vhz.py
(im_2kw_vhz_open_loop.plecs).

Run from the repository root:

    python examples/im_2kw_vhz.py [--open-loop]

"""

# %%
import sys
from math import pi
from pathlib import Path
from typing import Any

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

T_STOP = 1.6
nom = utils.NominalValues(U=400, I=5, f=50, P=2.2e3, tau=14.6)
base = utils.BaseValues.from_nominal(nom, n_p=2)
W_M_REF = StepSignal(time=0.2, after=base.w_M)  # Speed reference (rad/s)
TAU_L = StepSignal(time=0.8, after=nom.tau)  # Load torque (Nm)


# %%
def build_system(
    open_loop: bool = False,
) -> tuple[model.Drive, control.VHzControlSystem]:
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
    if open_loop:
        vhz_ctrl = control.OpenLoopVHzController(
            n_p=2, u_s_ref=lambda w_s: 1j * w_s * base.psi
        )
        return mdl, control.VHzControlSystem(vhz_ctrl, slew_rate=2 * pi * 60)
    est_par = control.InductionMachineInvGammaPars(
        n_p=2, R_s=3.7, R_R=2.1, L_sgm=0.021, L_M=0.224
    )
    cfg = control.ObserverBasedVHzControllerCfg(
        psi_s_nom=base.psi, i_s_max=1.5 * base.i
    )
    vhz_ctrl = control.ObserverBasedVHzController(est_par, cfg)
    return mdl, control.VHzControlSystem(vhz_ctrl, slew_rate=2 * pi * 120)


def control_signals(res: Any, open_loop: bool) -> dict[str, np.ndarray]:
    """Compared signals of the control system, the monitored signals of its block."""
    fbk, ref = res.ctrl.fbk, res.ctrl.ref
    if open_loop:
        return {
            "w_s": fbk.w_s,
            "u_s_ref": np.abs(ref.u_s),
            "i_sd": np.real(fbk.i_s),
            "i_sq": np.imag(fbk.i_s),
        }
    return {
        "w_s": fbk.w_s,
        "tau_M": fbk.tau_M,
        "tau_M_ref": ref.tau_M,
        "psi_s": np.abs(fbk.psi_s),
    }


# %%
if __name__ == "__main__":
    open_loop = "--open-loop" in sys.argv  # Open-loop V/Hz control
    mdl, ctrl = build_system(open_loop)
    ctrl.set_speed_ref(W_M_REF)
    mdl.mechanics.set_external_load_torque(TAU_L)
    name = "im_2kw_vhz_open_loop" if open_loop else "im_2kw_vhz"
    tmp = write_plecs_model(
        Path(__file__).with_name(f"{name}.plecs"), mdl, ctrl, T_STOP
    )

    # Simulate in motulator and in PLECS, and compare
    res = simulate_motulator(mdl, ctrl, T_STOP)
    cmp = Comparison(res, T_STOP)
    plecs_signals = simulate_plecs(tmp, cmp.t_eval, mdl, ctrl)
    ctrl_signals = control_signals(res, open_loop)
    cmp.print_differences("PLECS", plecs_signals, drive_signals(res), ctrl_signals)
    plot_drive(cmp, res, plecs_signals[0], "PLECS")
