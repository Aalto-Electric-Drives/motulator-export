"""
2.2-kW IPMSM, FVC
=================

This example simulates sensorless flux-vector control (FVC) of a 2.2-kW interior
permanent-magnet synchronous machine (IPMSM) drive natively. The system is that of the
*motulator* example :doc:`motulator:drive_examples/flux_vector/plot_2kw_ipmsm_fvc`.
The same system is also simulated in *motulator*, and the run times and the
differences are printed.

"""

# %%
import time
from math import pi

import motulator.drive.control.sm as control
import numpy as np
from motulator.drive import model, utils

from motulator_export import native

# %%
# Compute base values based on the nominal values.

nom = utils.NominalValues(U=370, I=4.3, f=75, P=2.2e3, tau=14)
base = utils.BaseValues.from_nominal(nom, n_p=3)


# %%
# Configure the system model and the control system, as in the *motulator* example.
# The function builds a new system for each simulation.


def build_system() -> tuple[model.Drive, control.VectorControlSystem]:
    """Build the system model and the control system."""
    par = model.SynchronousMachinePars(
        n_p=3, R_s=3.6, L_d=0.036, L_q=0.051, psi_f=0.545
    )
    machine = model.SynchronousMachine(par)
    mechanics = model.MechanicalSystem(J=0.015)
    converter = model.VoltageSourceConverter(u_dc=540)
    mdl = model.Drive(machine, mechanics, converter)

    est_par = par  # Assume accurate model parameter estimates
    cfg = control.FluxVectorControllerCfg(i_s_max=1.5 * base.i, sensorless=True)
    vector_ctrl = control.FluxVectorController(est_par, cfg)
    speed_ctrl = control.SpeedController(J=0.015, alpha_s=2 * pi * 4)
    ctrl = control.VectorControlSystem(vector_ctrl, speed_ctrl)

    # Speed reference and external load torque
    ctrl.set_speed_ref(lambda t: (t > 0.2) * 2 * base.w_M)
    mdl.mechanics.set_external_load_torque(lambda t: (t > 0.8) * 0.7 * nom.tau)
    return mdl, ctrl


# %%
# Simulate the system natively. Only the simulation object differs from the
# *motulator* example: `native.Simulation` replaces `model.Simulation`, and the
# results are plotted with the plotting functions of *motulator*.

mdl, ctrl = build_system()
sim = native.Simulation(mdl, ctrl)
start = time.perf_counter()
res = sim.simulate(t_stop=1.6)
t_native = time.perf_counter() - start
utils.plot(res, base)

# %%
# Simulate the same system in *motulator*. The native simulation takes the same solver
# steps, so the results agree to the rounding errors.

mdl, ctrl = build_system()
start = time.perf_counter()
res_py = model.Simulation(mdl, ctrl).simulate(t_stop=1.6)
t_py = time.perf_counter() - start

print(f"Run time: native {t_native:.2f} s, motulator {t_py:.1f} s")
diff = {
    "rotor speed (rad/s)": res.mdl.mechanics.w_M - res_py.mdl.mechanics.w_M,
    "stator current (A)": res.mdl.machine.i_s_ab - res_py.mdl.machine.i_s_ab,
    "torque reference (Nm)": res.ctrl.ref.tau_M - res_py.ctrl.ref.tau_M,
}
for name, d in diff.items():
    print(f"Maximum difference in the {name}: {np.max(np.abs(d)):.1e}")
