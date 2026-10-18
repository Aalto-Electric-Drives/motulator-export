"""
5.6-kW PM-SyRM, GradNet from FEM data, FVC: native simulation
=============================================================

This script simulates the drive system of the example
examples/drive/gradnet/plot_6kw_pmsyrm_gn_fvc_fem_harm.py of motulator natively, i.e.,
in C. The system is built in motulator, as usual, and only the simulation object is
replaced by `motulator_export.native.Simulation`, whose interface and results are
those of motulator. The machine model is a GradNet current map with spatial
harmonics, and the control system uses a GradNet flux map (sensored flux-vector
control).

With --compare, the system is also simulated in motulator, and the run times and the
differences are printed. With --pwm, the carrier comparison is used instead of the
zero-order hold, with a fine quantization of the duty ratios, so that the
comparison is not affected by the quantization.

Run from the repository root:

    python examples/pmsyrm_6kw_gn_fvc_native.py
    python examples/pmsyrm_6kw_gn_fvc_native.py --compare --pwm

"""

# %%
import argparse
import time
from math import pi
from pathlib import Path

import motulator.drive.control.sm as control
import motulator.drive.gradnet as gn
import numpy as np
from motulator.common.model import CarrierComparison
from motulator.drive import model, utils

from motulator_export import native

MODEL_DIR = Path(__file__).parent / "trained_models"
nom = utils.NominalValues(U=460, I=8.8, f=60, P=5.6e3, tau=29.7)
base = utils.BaseValues.from_nominal(nom, n_p=2)
T_STOP = 1.75


# %%
def build_system(pwm: bool = False) -> tuple[model.Drive, control.VectorControlSystem]:
    """Build the drive system of the motulator example."""
    path = MODEL_DIR / "baldor_fem_curr_map_harm_softmax_d48_sub10.npz"
    magnetic_map = gn.CurrentMapWithHarmonics(gn.load_gradnet(path))
    par = model.SpatialSaturatedSynchronousMachinePars(
        n_p=2, R_s=0.63, magnetic_map_fcn=magnetic_map
    )
    mdl = model.Drive(
        model.SynchronousMachine(par),
        model.MechanicalSystem(J=0.05),
        model.VoltageSourceConverter(u_dc=540),
        pwm=pwm,
    )
    if pwm:
        mdl.pwm = CarrierComparison(N=2**24)

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
    ctrl.set_speed_ref(lambda t: (t > 0.25) * 2 * base.w_M)
    mdl.mechanics.set_external_load_torque(lambda t: (t > 1.25) * 0.5 * base.tau)
    return mdl, ctrl


# %%
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Native simulation of the example")
    parser.add_argument("--pwm", action="store_true", help="carrier comparison")
    parser.add_argument("--compare", action="store_true", help="also in motulator")
    args = parser.parse_args()

    # Simulate natively
    sim = native.Simulation(*build_system(args.pwm))
    start = time.time()
    res = sim.simulate(t_stop=T_STOP)
    print(f"Native: {time.time() - start:.2f} s")

    if args.compare:
        sim_py = model.Simulation(*build_system(args.pwm))
        start = time.time()
        res_py = sim_py.simulate(t_stop=T_STOP)
        print(f"motulator: {time.time() - start:.1f} s")

        # The control systems are sampled at the same instants
        print("Maximum differences (native - motulator) at the sampling instants:")
        fbk, fbk_py = res.ctrl.fbk, res_py.ctrl.fbk
        print(f"  w_M:   {np.max(np.abs(fbk.w_M - fbk_py.w_M)):.3g} rad/s")
        print(f"  tau_M: {np.max(np.abs(fbk.tau_M - fbk_py.tau_M)):.3g} Nm")
        print(f"  i_s:   {np.max(np.abs(fbk.i_s - fbk_py.i_s)):.3g} A")

    # Plot the results in per-unit values, as in motulator
    utils.plot(res, base)
