"""
5.6-kW PM-SyRM, GradNet from FEM data, FVC: motulator vs. PLECS
===============================================================

This script builds the drive system of the example
examples/drive/gradnet/plot_6kw_pmsyrm_gn_fvc_fem_harm.py in motulator, exports it to
a PLECS model (pmsyrm_6kw_gn_fvc.plecs), and, if PLECS Standalone is running with the
RPC interface enabled, simulates both and compares the results. The machine model is
a GradNet current map with spatial harmonics, and the control system uses a GradNet
flux map (sensored flux-vector control).

Run from the repository root:

    python examples/pmsyrm_6kw_gn_fvc.py

"""

# %%
import atexit
import sys
import time
from math import pi
from pathlib import Path
from typing import cast

import matplotlib.pyplot as plt
import motulator.drive.control.sm as control
import motulator.drive.gradnet as gn
import numpy as np
from motulator.common.model import CarrierComparison, SolverCfg
from motulator.drive import model, utils

from motulator_export import StepSignal, sampled_step
from motulator_export.plecs import sm

MODEL_DIR = Path(__file__).parent / "trained_models"
nom = utils.NominalValues(U=460, I=8.8, f=60, P=5.6e3, tau=29.7)
base = utils.BaseValues.from_nominal(nom, n_p=2)

T_STOP = 1.75
W_M_REF = StepSignal(time=0.25, after=2 * base.w_M)  # Speed reference (rad/s)
TAU_L = StepSignal(time=1.25, after=0.5 * base.tau)  # Load torque (Nm)
SPEED_CTRL = {"J": 0.05, "alpha_s": 2 * pi * 4}  # Arguments of SpeedController


# %%
def build_system() -> tuple[model.Drive, control.VectorControlSystem]:
    """Build the drive system of the motulator example."""
    path = MODEL_DIR / "baldor_fem_curr_map_harm_softmax_d48_sub10.pth"
    magnetic_map = gn.CurrentMapWithHarmonics(
        gn.load_gradnet(path, activation=gn.Softmax)
    )
    par = model.SpatialSaturatedSynchronousMachinePars(
        n_p=2, R_s=0.63, magnetic_map_fcn=magnetic_map
    )
    mdl = model.Drive(
        model.SynchronousMachine(par),
        model.MechanicalSystem(J=0.05),
        model.VoltageSourceConverter(u_dc=540),
        pwm=True,
    )
    mdl.pwm = CarrierComparison(N=2**24)  # Fine quantization (not modeled in PLECS)

    path = MODEL_DIR / "baldor_fem_flux_map_pnorm_d12_sub20.pth"
    est_flux_map = gn.FluxMap(gn.load_gradnet(path, activation=gn.PNormGradient))
    est_par = control.SaturatedSynchronousMachinePars(
        n_p=2, R_s=0.63, psi_s_dq_fcn=est_flux_map
    )
    cfg = control.FluxVectorControllerCfg(
        i_s_max=2 * base.i, alpha_i=0, alpha_o=2 * pi * 8, J=0.05, sensorless=False
    )
    ctrl = control.VectorControlSystem(
        control.FluxVectorController(est_par, cfg),
        control.SpeedController(**SPEED_CTRL),
    )
    return mdl, ctrl


# %%
if __name__ == "__main__":
    mdl, ctrl = build_system()
    # Speed reference switching at the same sample as in motulator
    T_s = cast(control.FluxVectorController, ctrl.vector_ctrl).cfg.T_s
    w_M_ref = sampled_step(W_M_REF, T_s)
    path = sm.write_model(
        Path(__file__).with_name("pmsyrm_6kw_gn_fvc.plecs"),
        mdl,
        ctrl,
        w_M_ref,
        TAU_L,
        T_STOP,
        SPEED_CTRL,
    )
    print(f"Wrote {path}")
    # Temporary copy with the output ports for the comparison, removed at exit
    tmp = sm.write_model(
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

    # Simulate in PLECS first (fast), at a uniform grid
    t_grid = np.arange(0, T_STOP, T_s / 4)
    start = time.time()
    try:
        plecs_mdl, _ = sm.simulate(tmp, t_grid)
    except ConnectionRefusedError:
        print("PLECS RPC interface not available, skipping the comparison.")
        sys.exit()
    print(f"PLECS: {time.time() - start:.1f} s")

    # Simulate in motulator
    ctrl.set_speed_ref(W_M_REF)
    mdl.mechanics.set_external_load_torque(TAU_L)
    sim = model.Simulation(mdl, ctrl, cfg=SolverCfg(rtol=1e-8, atol=1e-8))
    start = time.time()
    res = sim.simulate(t_stop=T_STOP)
    print(f"motulator: {time.time() - start:.1f} s")

    # Compare at the motulator solver steps (motulator runs to the end of the last
    # sampling period, drop the extra samples)
    keep = res.mdl.t <= T_STOP
    t, idx = np.unique(res.mdl.t[keep], return_index=True)
    plecs_mdl, plecs_ctrl = sm.simulate(tmp, t)
    i_s_abc = np.array([plecs_mdl[k] for k in ("i_a", "i_b", "i_c")])
    i_s_ab = (2 / 3) * (i_s_abc[0] - 0.5 * (i_s_abc[1] + i_s_abc[2])) + 1j * (
        i_s_abc[1] - i_s_abc[2]
    ) / np.sqrt(3)
    ref = {
        "w_M": res.mdl.mechanics.w_M[keep][idx],
        "tau_M": res.mdl.machine.tau_M[keep][idx],
        "i_s_ab": res.mdl.machine.i_s_ab[keep][idx],
    }
    print("Maximum differences (PLECS - motulator):")
    print(f"  w_M:    {np.max(np.abs(plecs_mdl['w_M'] - ref['w_M'])):.3g} rad/s")
    print(f"  tau_M:  {np.max(np.abs(plecs_mdl['tau_M'] - ref['tau_M'])):.3g} Nm")
    print(f"  i_s_ab: {np.max(np.abs(i_s_ab - ref['i_s_ab'])):.3g} A")

    # Plot
    fig, axs = plt.subplots(3, 1, sharex=True, figsize=(8, 7))
    axs[0].plot(t, ref["w_M"], label=r"$\omega_\mathrm{M}$ (motulator)")
    axs[0].plot(t, plecs_mdl["w_M"], "--", label=r"$\omega_\mathrm{M}$ (PLECS)")
    axs[0].set_ylabel("Speed (rad/s)")
    axs[1].plot(t, ref["tau_M"], label=r"$\tau_\mathrm{M}$ (motulator)")
    axs[1].plot(t, plecs_mdl["tau_M"], "--", label=r"$\tau_\mathrm{M}$ (PLECS)")
    axs[1].set_ylabel("Torque (Nm)")
    axs[2].plot(t, ref["i_s_ab"].real, label=r"$i_\alpha$ (motulator)")
    axs[2].plot(t, i_s_ab.real, "--", label=r"$i_\alpha$ (PLECS)")
    axs[2].set_ylabel("Current (A)")
    axs[2].set_xlabel("Time (s)")
    for ax in axs:
        ax.legend(loc="upper right")
        ax.grid(True)
    fig.tight_layout()
    plt.show()
