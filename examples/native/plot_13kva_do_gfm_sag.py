"""
12.5-kVA, DO-GFM, grid-voltage sag
==================================

This example simulates a 12.5-kVA converter with disturbance-observer-based
grid-forming (DO-GFM) control in a weak grid during a grid-voltage sag natively. The
system is that of the *motulator* example
:doc:`motulator:grid_examples/grid_forming/plot_13kva_do_gfm_sag`: the grid voltage
drops to 0.5 p.u. while the active-power reference is kept at 1 p.u., and the
active-power reference is limited to a realizable level. The magnitude of the grid
voltage is a function of time, which the native simulation calls from C.

"""

# %%
from motulator.grid import control, model, utils

from motulator_export import native

# %%
# Compute base values based on the nominal values.

nom = utils.NominalValues(U=400, I=18, f=50, P=12.5e3)
base = utils.BaseValues.from_nominal(nom)

# %%
# Configure the system model, with the grid-voltage sag.

ac_filter = model.LFilter(L_f=0.15 * base.L, R_f=0.05 * base.Z, L_g=0.74 * base.L)
ac_source = model.ThreePhaseSource(
    w_g=base.w, e_g=lambda t: (1 - (t > 0.4) * 0.5 + (t > 1) * 0.5) * base.u
)
converter = model.VoltageSourceConverter(u_dc=650)
mdl = model.GridConverterSystem(converter, ac_filter, ac_source)

# %%
# Configure the control system, with the limitation of the active current.

cfg = control.ObserverBasedGridFormingControllerCfg(
    i_max=1.3 * base.i,
    L=0.35 * base.L,
    R=0.05 * base.Z,
    R_a=0.2 * base.Z,
    u_nom=base.u,
    w_nom=base.w,
    i_d_max=1.1 * base.i,
)
inner_ctrl = control.ObserverBasedGridFormingController(cfg)
ctrl = control.GridConverterControlSystem(inner_ctrl)

# %%
# Set the references for the converter output voltage magnitude and the active power.

ctrl.set_ac_voltage_ref(base.u)
ctrl.set_power_ref(lambda t: (t > 0.1) * nom.P)

# %%
# Simulate natively and plot the results.

sim = native.Simulation(mdl, ctrl)
res = sim.simulate(t_stop=1.4)
utils.plot_control_signals(res, base)
utils.plot_grid_waveforms(res, base)
