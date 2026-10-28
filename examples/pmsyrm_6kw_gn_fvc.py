"""
5.6-kW PM-SyRM, GradNet from FEM data, FVC: motulator vs. PLECS
===============================================================

This script builds the drive system of the example
examples/drive/flux_vector/plot_6kw_pmsyrm_gn_fvc_fem_harm.py in motulator, exports it
to a PLECS model (pmsyrm_6kw_gn_fvc.plecs), and, if PLECS Standalone is running with
the RPC interface enabled, simulates both and compares the results. The machine model is
a GradNet current map with spatial harmonics, and the control system uses a GradNet
flux map (sensored flux-vector control).

Run from the repository root:

    python examples/pmsyrm_6kw_gn_fvc.py

"""

# %%
import time
from math import pi
from pathlib import Path
from typing import cast

import motulator.drive.control.sm as control
import motulator.drive.gradnet as gn
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

MODEL_DIR = Path(__file__).parent / "trained_models"
nom = utils.NominalValues(U=460, I=8.8, f=60, P=5.6e3, tau=29.7)
base = utils.BaseValues.from_nominal(nom, n_p=2)

T_STOP = 1.75
W_M_REF = StepSignal(time=0.25, after=2 * base.w_M)  # Speed reference (rad/s)
TAU_L = StepSignal(time=1.25, after=0.5 * base.tau)  # Load torque (Nm)


# %%
def build_system() -> tuple[model.Drive, control.VectorControlSystem]:
    """Build the drive system of the motulator example."""
    path = MODEL_DIR / "baldor_fem_curr_map_harm_softmax_d48_sub10.npz"
    magnetic_map = gn.CurrentMapWithHarmonics(gn.load_gradnet(path))
    par = model.SpatialSaturatedSynchronousMachinePars(
        n_p=2, R_s=0.63, magnetic_map_fcn=magnetic_map
    )
    mdl = model.Drive(
        model.SynchronousMachine(par),
        model.MechanicalSystem(J=0.05),
        # Fine quantization (not modeled in PLECS)
        model.VoltageSourceConverter(
            u_dc=540, modulator=model.CarrierComparison(N=2**24)
        ),
    )

    path = MODEL_DIR / "baldor_fem_flux_map_pnorm_d12_sub20.npz"
    est_flux_map = gn.FluxMap(gn.load_gradnet(path))
    est_par = control.SaturatedSynchronousMachinePars(
        n_p=2, R_s=0.63, psi_s_dq_fcn=est_flux_map
    )
    cfg = control.FluxVectorControllerCfg(
        i_s_max=2 * base.i, alpha_i=0, alpha_o=2 * pi * 8, J=0.05, sensorless=False
    )
    ctrl = control.VectorControlSystem(
        control.FluxVectorController(est_par, cfg),
        control.SpeedController(J=0.05, alpha_s=2 * pi * 4),
    )
    return mdl, ctrl


# %%
if __name__ == "__main__":
    mdl, ctrl = build_system()
    ctrl.set_speed_ref(W_M_REF)
    mdl.mechanics.set_external_load_torque(TAU_L)
    path = Path(__file__).with_name("pmsyrm_6kw_gn_fvc.plecs")
    tmp = write_plecs_model(path, mdl, ctrl, T_STOP)

    # Simulate in PLECS first (fast), at a uniform grid
    T_s = cast(control.FluxVectorController, ctrl.vector_ctrl).cfg.T_s
    start = time.time()
    simulate_plecs(tmp, np.arange(0, T_STOP, T_s / 4), mdl, ctrl)
    print(f"PLECS: {time.time() - start:.1f} s")

    # Simulate in motulator
    start = time.time()
    res = simulate_motulator(mdl, ctrl, T_STOP)
    print(f"motulator: {time.time() - start:.1f} s")

    # Compare at the motulator solver steps
    cmp = Comparison(res, T_STOP, control=False)
    plecs_signals = simulate_plecs(tmp, cmp.t_eval, mdl, ctrl)
    cmp.print_differences("PLECS", plecs_signals, drive_signals(res))
    plot_drive(cmp, res, plecs_signals[0], "PLECS")
