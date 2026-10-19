"""
5.6-kW PM-SyRM, GradNet models, FVC
===================================

This example simulates flux-vector control (FVC) of a 5.6-kW PM synchronous reluctance
machine drive natively. The system is that of the *motulator* example
:doc:`motulator:drive_examples/flux_vector/plot_6kw_pmsyrm_gn_fvc_fem_harm`: the machine
model uses a GradNet current map with spatial harmonics, and the control system uses a
GradNet flux map, both trained on FEM data. The GradNets are evaluated in C, which
makes the native simulation about 30 times faster than *motulator*.

"""

# %%
import time
from math import pi
from pathlib import Path

import motulator.drive.control.sm as control
import motulator.drive.gradnet as gn
from motulator.drive import model, utils

from motulator_export import native

# %%
# Compute base values based on the nominal values.

nom = utils.NominalValues(U=460, I=8.8, f=60, P=5.6e3, tau=29.7)
base = utils.BaseValues.from_nominal(nom, n_p=2)

# %%
# Configure the system model with the GradNet current map with spatial harmonics.

try:
    MODEL_DIR = Path(__file__).parents[1] / "trained_models"
except NameError:  # The gallery of the documentation runs the examples in their folder
    MODEL_DIR = Path("..") / "trained_models"
path = MODEL_DIR / "baldor_fem_curr_map_harm_softmax_d48_sub10.npz"
magnetic_map = gn.CurrentMapWithHarmonics(gn.load_gradnet(path))
par = model.SpatialSaturatedSynchronousMachinePars(
    n_p=2, R_s=0.63, magnetic_map_fcn=magnetic_map
)
machine = model.SynchronousMachine(par)
mechanics = model.MechanicalSystem(J=0.05)
converter = model.VoltageSourceConverter(u_dc=540)
mdl = model.Drive(machine, mechanics, converter)

# %%
# Configure the control system with the GradNet flux map.

path = MODEL_DIR / "baldor_fem_flux_map_pnorm_d12_sub20.npz"
est_flux_map = gn.FluxMap(gn.load_gradnet(path))
est_par = control.SaturatedSynchronousMachinePars(
    n_p=2, R_s=0.63, psi_s_dq_fcn=est_flux_map
)
cfg = control.FluxVectorControllerCfg(
    i_s_max=2 * base.i, alpha_i=0, alpha_o=2 * pi * 8, J=0.05, sensorless=False
)
vector_ctrl = control.FluxVectorController(est_par, cfg)
speed_ctrl = control.SpeedController(J=0.05, alpha_s=2 * pi * 4)
ctrl = control.VectorControlSystem(vector_ctrl, speed_ctrl)

# %%
# Set the speed reference and the external load torque.

ctrl.set_speed_ref(lambda t: (t > 0.25) * 2 * base.w_M)
mdl.mechanics.set_external_load_torque(lambda t: (t > 1.25) * 0.5 * base.tau)

# %%
# Simulate natively and plot the results in per-unit values.

sim = native.Simulation(mdl, ctrl)
start = time.perf_counter()
res = sim.simulate(t_stop=1.75)
print(f"Run time: {time.perf_counter() - start:.2f} s")
utils.plot(res, base)
