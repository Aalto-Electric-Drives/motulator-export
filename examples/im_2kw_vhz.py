"""
2.2-kW IM, observer-based or open-loop V/Hz control: motulator vs. PLECS
========================================================================

This script builds a 2.2-kW induction machine (IM) drive with observer-based V/Hz
control, corresponding to the example examples/drive/vhz/plot_2kw_im_ovhz.py, exports
it to a PLECS model (im_2kw_vhz.plecs), and, if PLECS Standalone is running with the
RPC interface enabled, simulates both and compares the results.

With --open-loop, the control system is pure open-loop V/Hz control, i.e., the
observer-based V/Hz controller with the machine model L_M = inf (and R_s = R_R =
L_sgm = 0) and zero gains, as in examples/drive/vhz/plot_2kw_im_diode_vhz.py
(im_2kw_vhz_open_loop.plecs).

Run from the repository root:

    python examples/im_2kw_vhz.py [--open-loop]

"""

# %%
import atexit
import sys
from math import inf, pi
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
        model.VoltageSourceConverter(u_dc=540),
        pwm=True,
    )
    mdl.pwm = CarrierComparison(N=2**24)  # Fine quantization (not modeled in PLECS)
    if open_loop:
        est_par = control.InductionMachineInvGammaPars(
            n_p=2, R_s=0, R_R=0, L_sgm=0, L_M=inf
        )
        cfg = control.ObserverBasedVHzControllerCfg(
            psi_s_nom=base.psi, i_s_max=inf, alpha_f=0, alpha_tau=0, alpha_psi=0
        )
        slew_rate = 2 * pi * 60
    else:
        est_par = control.InductionMachineInvGammaPars(
            n_p=2, R_s=3.7, R_R=2.1, L_sgm=0.021, L_M=0.224
        )
        cfg = control.ObserverBasedVHzControllerCfg(
            psi_s_nom=base.psi, i_s_max=1.5 * base.i
        )
        slew_rate = 2 * pi * 120
    vhz_ctrl = control.ObserverBasedVHzController(est_par, cfg)
    ctrl = control.VHzControlSystem(vhz_ctrl, slew_rate=slew_rate)
    return mdl, ctrl


# %%
if __name__ == "__main__":
    open_loop = "--open-loop" in sys.argv  # Pure open-loop V/Hz control
    mdl, ctrl = build_system(open_loop)
    # Speed reference switching at the same sample as in motulator
    T_s = cast(control.ObserverBasedVHzController, ctrl.vhz_ctrl).cfg.T_s
    w_M_ref = sampled_step(W_M_REF, T_s)
    name = "im_2kw_vhz_open_loop" if open_loop else "im_2kw_vhz"
    path = im.write_model(
        Path(__file__).with_name(f"{name}.plecs"), mdl, ctrl, w_M_ref, TAU_L, T_STOP
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
        plecs_mdl, plecs_ctrl = im.simulate(tmp, t_eval, ctrl)
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
        ("w_s", fbk.w_s[in_ctrl]),
        ("tau_M", fbk.tau_M[in_ctrl]),
        ("tau_M_ref", ref.tau_M[in_ctrl]),
        ("psi_s", np.abs(fbk.psi_s[in_ctrl])),
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
