"""
2.2-kW IM, dead time, sensorless CVC: motulator vs. PLECS
=========================================================

This script builds a 2.2-kW induction machine (IM) drive with sensorless current-
vector control (CVC) at low speeds, corresponding to the example
examples/drive/current_vector/plot_2kw_im_dead_time_cvc.py with the constant-
parameter machine model and the signum function as the current-direction function.
The dead time of the converter is modeled, and its effect is compensated for in the
control system. The script exports the system to a PLECS model
(im_2kw_dead_time_cvc.plecs), where the dead time is modeled with the Blanking Time
block and the IGBT converter of PLECS, and, if PLECS Standalone is running with the
RPC interface enabled, simulates both and compares the results.

With --tanh, the current direction in the dead-time compensation of the control
system is the smooth function tanh(i/i_0) instead of the signum function, as on
hardware, where the current ripple and the measurement noise near the zero
crossings would make the signum function chatter (im_2kw_dead_time_cvc_tanh.plecs).
The converter model still uses the signum function.

Run from the repository root:

    python examples/im_2kw_dead_time_cvc.py [--tanh]

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
from motulator.common.utils import abc2complex, dead_time_error
from motulator.drive import model, utils

from motulator_export import StepSignal, sampled_step
from motulator_export.plecs import im

T_STOP = 2.4
nom = utils.NominalValues(U=400, I=5, f=50, P=2.2e3, tau=14.6)
base = utils.BaseValues.from_nominal(nom, n_p=2)
# Speed reference (rad/s), reversed under the load torque (Nm), leading to the
# regenerating mode
W_M_REF = StepSignal(time=[0.2, 1.4], after=[0.05 * base.w_M, -0.05 * base.w_M])
TAU_L = StepSignal(time=0.8, after=0.7 * nom.tau)
SPEED_CTRL = {"J": 0.015, "alpha_s": 2 * pi * 4}  # Arguments of SpeedController
T_D = 2e-6  # Dead time (s)
T_S = 125e-6  # Sampling period (s), the default value in CurrentVectorControllerCfg
I_0 = 0.1  # Current scale (A) of tanh(i/i_0), about half the peak-to-peak ripple


# %%
def build_system(tanh: bool = False) -> tuple[model.Drive, control.VectorControlSystem]:
    """
    Build the drive system of the motulator example.

    With `tanh`, the dead-time compensation uses the current-direction function
    `tanh(i/I_0)` instead of `np.sign`.

    """
    par = model.InductionMachineInvGammaPars(
        n_p=2, R_s=3.7, R_R=2.1, L_sgm=0.021, L_M=0.224
    )
    mdl = model.Drive(
        model.InductionMachine(par),
        model.MechanicalSystem(J=0.015),
        model.VoltageSourceConverter(u_dc=540, t_d=T_D),
        pwm=True,
    )
    # Fine quantization (not modeled in PLECS), the dead time as in the converter
    mdl.pwm = CarrierComparison(N=2**24, t_d=T_D)
    est_par = control.InductionMachineInvGammaPars(
        n_p=2, R_s=3.7, R_R=2.1, L_sgm=0.021, L_M=0.224
    )
    cfg = control.CurrentVectorControllerCfg(
        psi_s_nom=0.95 * base.psi, i_s_max=1.5 * base.i, sensorless=True, T_s=T_S
    )
    # The duty-ratio error model of the system model, compensated for in the PWM,
    # optionally with a smooth current-direction function
    sign = (lambda i: np.tanh(i / I_0)) if tanh else np.sign
    pwm = control.PWM(d_err=lambda i, d: dead_time_error(i, d, T_D, T_S, sign))
    ctrl = control.VectorControlSystem(
        control.CurrentVectorController(est_par, cfg),
        control.SpeedController(**SPEED_CTRL),
        pwm,
    )
    return mdl, ctrl


# %%
if __name__ == "__main__":
    tanh = "--tanh" in sys.argv  # Smooth current direction in the compensation
    mdl, ctrl = build_system(tanh)
    # Speed reference switching at the same sample as in motulator
    T_s = cast(control.CurrentVectorController, ctrl.vector_ctrl).cfg.T_s
    w_M_ref = sampled_step(W_M_REF, T_s)
    path = im.write_model(
        Path(__file__).with_name(
            f"im_2kw_dead_time_cvc{'_tanh' if tanh else ''}.plecs"
        ),
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
