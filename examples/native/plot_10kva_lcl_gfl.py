"""
10-kVA, LCL filter, GFL
=======================

This example simulates grid-following (GFL) current-vector control of a 10-kVA grid
converter, connected to the grid through an LCL filter, natively. The system is that
of the *motulator* example :doc:`motulator:grid_examples/grid_following/plot_10kva_lcl_gfl`.

"""

# %%
from motulator.grid import control, model, utils

from motulator_export import native

# %%
# Compute base values based on the nominal values.

nom = utils.NominalValues(U=400, I=14.5, f=50, P=10e3)
base = utils.BaseValues.from_nominal(nom)

# %%
# Configure the system model.

ac_filter = model.LCLFilter(
    L_fc=0.073 * base.L, L_fg=0.073 * base.L, C_f=0.043 * base.C, u_f0_ab=base.u
)
ac_source = model.ThreePhaseSource(w_g=base.w, e_g=base.u)
converter = model.VoltageSourceConverter(u_dc=650)
mdl = model.GridConverterSystem(converter, ac_filter, ac_source)

# %%
# Configure the control system.

cfg = control.CurrentVectorControllerCfg(
    i_max=1.5 * base.i, L=0.073 * base.L, T_s=100e-6
)
inner_ctrl = control.CurrentVectorController(cfg)
ctrl = control.GridConverterControlSystem(inner_ctrl)

# %%
# Set the references for the converter output power.

ctrl.set_power_ref(lambda t: (t > 0.02) * 5e3)
ctrl.set_reactive_power_ref(lambda t: (t > 0.04) * 4e3)

# %%
# Simulate natively and plot the results.

sim = native.Simulation(mdl, ctrl)
res = sim.simulate(t_stop=0.08)
utils.plot_control_signals(res, base)
utils.plot_grid_waveforms(res, base)
