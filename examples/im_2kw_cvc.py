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
import atexit
import sys
from math import pi
from pathlib import Path
from typing import cast

import matplotlib.pyplot as plt
import motulator.drive.control.im as control
import numpy as np
from motulator.common.model import CarrierComparison, SolverCfg
from motulator.common.utils import abc2complex
from motulator.drive import model, utils

from motulator_export import StepSignal, sampled_step
from motulator_export.plecs import im

T_STOP = 1.5
nom = utils.NominalValues(U=400, I=5, f=50, P=2.2e3, tau=14.6)
base = utils.BaseValues.from_nominal(nom, n_p=2)
W_M_REF = StepSignal(time=0.2, after=0.5 * base.w_M)  # Speed reference (rad/s)
TAU_L = StepSignal(time=0.75, after=nom.tau)  # Load torque (Nm)
SPEED_CTRL = {"J": 0.015, "alpha_s": 2 * pi * 4}  # Arguments of SpeedController


# %%
def build_system() -> tuple[model.Drive, control.VectorControlSystem]:
    """Build the drive system of the motulator example."""
    par = model.InductionMachineInvGammaPars(
        n_p=2, R_s=3.7, R_R=2.1, L_sgm=0.021, L_M=0.224
    )
    mdl = model.Drive(
        model.InductionMachine(par),
        model.MechanicalSystem(J=0.015),
        model.VoltageSourceConverter(u_dc=540),
        pwm=True,
    )
    mdl.pwm = CarrierComparison(N=2**24)  # Fine quantization (not modeled in PLECS)
    est_par = control.InductionMachineInvGammaPars(
        n_p=2, R_s=3.7, R_R=2.1, L_sgm=0.021, L_M=0.224
    )
    cfg = control.CurrentVectorControllerCfg(
        psi_s_nom=base.psi, i_s_max=1.5 * base.i, sensorless=True
    )
    ctrl = control.VectorControlSystem(
        control.CurrentVectorController(est_par, cfg),
        control.SpeedController(**SPEED_CTRL),
    )
    return mdl, ctrl


# %%
if __name__ == "__main__":
    mdl, ctrl = build_system()
    # Speed reference switching at the same sample as in motulator
    T_s = cast(control.CurrentVectorController, ctrl.vector_ctrl).cfg.T_s
    w_M_ref = sampled_step(W_M_REF, T_s)
    path = im.write_model(
        Path(__file__).with_name("im_2kw_cvc.plecs"),
        mdl,
        ctrl,
        w_M_ref,
        TAU_L,
        T_STOP,
        SPEED_CTRL,
    )
    print(f"Wrote {path}")
    # Temporary copy with the output ports for the comparison, removed at exit
    tmp = im.write_model(
        path.with_stem(path.stem + "_tmp"),
        mdl,
        ctrl,
        w_M_ref,
        TAU_L,
        T_STOP,
        SPEED_CTRL,
        outputs=True,
    )
    atexit.register(tmp.unlink, missing_ok=True)

    # Simulate in motulator with tight tolerances
    ctrl.set_speed_ref(W_M_REF)
    mdl.mechanics.set_external_load_torque(TAU_L)
    sim = model.Simulation(mdl, ctrl, cfg=SolverCfg(rtol=1e-9, atol=1e-9))
    res = sim.simulate(t_stop=T_STOP)
    in_mdl = res.mdl.t <= T_STOP
    in_ctrl = res.ctrl.t + 0.5 * T_s <= T_STOP

    # Simulate in PLECS at the motulator solver steps and just after the sampling
    # instants
    t_mdl = res.mdl.t[in_mdl]
    t_ctrl = res.ctrl.t[in_ctrl] + 0.5 * T_s
    t_eval = np.unique(np.concatenate((t_mdl, t_ctrl)))
    try:
        plecs_mdl, plecs_ctrl = im.simulate(tmp, t_eval)
    except ConnectionRefusedError:
        print("PLECS RPC interface not available, skipping the comparison.")
        sys.exit()

    k_mdl = np.searchsorted(t_eval, t_mdl)
    i_s_ab = abc2complex(np.array([plecs_mdl[f"i_{p}"][k_mdl] for p in "abc"]))
    print("Maximum differences (PLECS - motulator):")
    for name, plecs_value, value in [
        ("w_M", plecs_mdl["w_M"][k_mdl], res.mdl.mechanics.w_M[in_mdl]),
        ("tau_M", plecs_mdl["tau_M"][k_mdl], res.mdl.machine.tau_M[in_mdl]),
        ("i_s_ab", i_s_ab, res.mdl.machine.i_s_ab[in_mdl]),
    ]:
        print(f"  mdl.{name}: {np.max(np.abs(plecs_value - value)):.3g}")
    k_ctrl = np.searchsorted(t_eval, t_ctrl)
    fbk, ref = res.ctrl.fbk, res.ctrl.ref
    for name, value in [
        ("w_M", fbk.w_M[in_ctrl]),
        ("tau_M", fbk.tau_M[in_ctrl]),
        ("tau_M_ref", ref.tau_M[in_ctrl]),
        ("psi_R", np.abs(fbk.psi_R[in_ctrl])),
    ]:
        err = np.max(np.abs(plecs_ctrl[name][k_ctrl] - value))
        print(f"  ctrl.{name}: {err:.3g}")

    # Plot
    fig, axs = plt.subplots(3, 1, sharex=True, figsize=(8, 7))
    t = t_mdl
    axs[0].plot(
        t, res.mdl.mechanics.w_M[in_mdl], label=r"$\omega_\mathrm{M}$ (motulator)"
    )
    axs[0].plot(t, plecs_mdl["w_M"][k_mdl], "--", label=r"$\omega_\mathrm{M}$ (PLECS)")
    axs[0].set_ylabel("Speed (rad/s)")
    axs[1].plot(
        t, res.mdl.machine.tau_M[in_mdl], label=r"$\tau_\mathrm{M}$ (motulator)"
    )
    axs[1].plot(t, plecs_mdl["tau_M"][k_mdl], "--", label=r"$\tau_\mathrm{M}$ (PLECS)")
    axs[1].set_ylabel("Torque (Nm)")
    axs[2].plot(t, res.mdl.machine.i_s_ab[in_mdl].real, label=r"$i_\alpha$ (motulator)")
    axs[2].plot(t, i_s_ab.real, "--", label=r"$i_\alpha$ (PLECS)")
    axs[2].set_ylabel("Current (A)")
    axs[2].set_xlabel("Time (s)")
    for ax in axs:
        ax.legend(loc="upper right")
        ax.grid(True)
    fig.tight_layout()
    plt.show()
