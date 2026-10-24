"""
2.2-kW IPMSM, sensorless CVC: motulator vs. PLECS
=================================================

This script builds the drive system of the example
examples/drive/current_vector/plot_2kw_ipmsm_blocked_cvc.py with sensorless
current-vector control, exports it to a PLECS model (ipmsm_2kw_cvc.plecs), and, if
PLECS Standalone is running with the RPC interface enabled, simulates both and
compares the results.

Run from the repository root:

    python examples/ipmsm_2kw_cvc.py [--blocked]

With --blocked, the gates of the converter are blocked before the start and during
the operation, as in the example (ipmsm_2kw_cvc_blocked.plecs): while the gates are
blocked, the control system runs in standby, and when they are released at 0.8 s,
it catches the rotating machine with a flying start.

"""

# %%
import sys
from math import pi
from pathlib import Path

import motulator.drive.control.sm as control
from _compare import (
    Comparison,
    drive_signals,
    plot_drive,
    simulate_motulator,
    simulate_plecs,
    write_plecs_model,
)
from motulator.common.model import CarrierComparison
from motulator.drive import model, utils

from motulator_export import StepSignal

T_STOP = 1.2
nom = utils.NominalValues(U=370, I=4.3, f=75, P=2.2e3, tau=14)
base = utils.BaseValues.from_nominal(nom, n_p=3)
W_M_REF = StepSignal(time=0.05, after=0.8 * base.w_M)  # Speed reference (rad/s)
TAU_L = StepSignal(time=0.3, after=0.5 * nom.tau)  # Load torque (Nm)
# Gate enable with --blocked: released at 0.05 s, blocked at 0.6 s, and released at
# 0.8 s, as in the example
ENABLE = StepSignal(time=[0.05, 0.6, 0.8], after=[1.0, 0.0, 1.0], before=0.0)


# %%
def build_system() -> tuple[model.Drive, control.VectorControlSystem]:
    """Build the drive system of the motulator example."""
    par = model.SynchronousMachinePars(
        n_p=3, R_s=3.6, L_d=0.036, L_q=0.051, psi_f=0.545
    )
    mdl = model.Drive(
        model.SynchronousMachine(par),
        model.MechanicalSystem(J=0.015),
        model.VoltageSourceConverter(u_dc=540),
        pwm=True,
    )
    mdl.pwm = CarrierComparison(N=2**24)  # Fine quantization (not modeled in PLECS)
    cfg = control.CurrentVectorControllerCfg(i_s_max=1.5 * base.i)
    ctrl = control.VectorControlSystem(
        control.CurrentVectorController(par, cfg),
        control.SpeedController(J=0.015, alpha_s=2 * pi * 4),
    )
    return mdl, ctrl


# %%
if __name__ == "__main__":
    blocked = "--blocked" in sys.argv  # Gates blocked at the start and later
    mdl, ctrl = build_system()
    ctrl.set_speed_ref(W_M_REF)
    mdl.mechanics.set_external_load_torque(TAU_L)
    suffix = ""
    if blocked:
        # The gate enable of the control system, exported as its input enable
        ctrl.set_enable(ENABLE)
        suffix = "_blocked"
    path = Path(__file__).with_name(f"ipmsm_2kw_cvc{suffix}.plecs")
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
        "i_q_ref": ref.i_s.imag,
    }
    cmp.print_differences("PLECS", plecs_signals, drive_signals(res), ctrl_signals)
    plot_drive(cmp, res, plecs_signals[0], "PLECS")
