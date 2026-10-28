"""
2.2-kW IM, diode bridge, V/Hz control
=====================================

This example simulates pure open-loop V/Hz control of a 2.2-kW induction machine (IM)
drive natively. The system is that of the *motulator* example
:doc:`motulator:drive_examples/grid_connection/plot_2kw_im_diode_vhz`: the drive is fed
from a six-pulse diode bridge, which is supplied from a stiff grid through an AC choke,
the load torque is quadratic, and the converter uses carrier comparison.

"""

# %%
from math import inf, pi, sqrt

import matplotlib.pyplot as plt
import motulator.drive.control.im as control
from motulator.common.model import CompositeModel, DCBus, ThreePhaseSource
from motulator.common.utils import complex2abc
from motulator.drive import model as drive_model
from motulator.drive import utils as drive_utils
from motulator.grid import model as grid_model

from motulator_export import native

# %%
# Compute base values based on the nominal values.

nom = drive_utils.NominalValues(U=400, I=5, f=50, P=2.2e3, tau=14.6)
base = drive_utils.BaseValues.from_nominal(nom, n_p=2)

# %%
# Configure the system model. The diode bridge and the inverter share the DC bus, and
# their models are combined into a composite model, the diode bridge first.

dc_bus = DCBus(C_dc=235e-6, u_dc=sqrt(2) * nom.U)
grid_sys = grid_model.GridConverterSystem(
    grid_model.DiodeBridge(),
    grid_model.LFilter(L_f=1e-3),
    ThreePhaseSource(w_g=base.w, e_g=base.u),
    dc_bus=dc_bus,
)

par = drive_model.InductionMachineInvGammaPars(
    n_p=2, R_s=3.7, R_R=2.1, L_sgm=0.021, L_M=0.224
)
machine = drive_model.InductionMachine(par)
k = 1.1 * nom.tau / base.w_M**2  # Quadratic load torque profile
mechanics = drive_model.MechanicalSystem(J=0.015, B_L=lambda w_M: k * abs(w_M))
converter = drive_model.VoltageSourceConverter(
    modulator=drive_model.CarrierComparison()
)
drive_sys = drive_model.Drive(machine, mechanics, converter, dc_bus=dc_bus)

mdl = CompositeModel({"grid": grid_sys, "drive": drive_sys})

# %%
# Configure the control system. The open-loop V/Hz control is the observer-based V/Hz
# control with the machine model `L_M = inf` and zero gains.

est_par = control.InductionMachineInvGammaPars(n_p=2, R_s=0, R_R=0, L_sgm=0, L_M=inf)
cfg = control.ObserverBasedVHzControllerCfg(
    psi_s_nom=base.psi, i_s_max=inf, alpha_f=0, alpha_tau=0, alpha_psi=0
)
vhz_ctrl = control.ObserverBasedVHzController(est_par, cfg)
ctrl = control.VHzControlSystem(vhz_ctrl, slew_rate=2 * pi * 60)

# %%
# Set the speed reference and the external load torque.

ctrl.set_speed_ref(lambda t: (t > 0.2) * base.w_M)
mechanics.set_external_load_torque(lambda t: (t > 1) * 0.2 * nom.tau)

# %%
# Simulate natively, giving the control system of the drive by its name, and plot the
# drive results in per-unit values.

sim = native.Simulation(mdl, {"drive": ctrl})
res = sim.simulate(t_stop=1.4)
res_drive = drive_model.SimulationResults(res.mdl.drive, res.ctrl.drive)
drive_utils.plot(res_drive, base)
drive_utils.plot_stator_waveforms(res_drive, base, t_lims=(1.1, 1.125))

# %%
# Plot the grid-side waveforms: the phase-a voltage of the grid, the DC-bus voltage,
# and the phase-a current of the diode bridge, which conducts in pulses.

t, grid_ts = res.mdl.t, res.mdl.grid
fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
ax1.plot(t, complex2abc(grid_ts.ac_source.e_g_ab)[0] / base.u, label=r"$e_\mathrm{ga}$")
ax1.plot(t, grid_ts.dc_bus.u_dc / base.u, label=r"$u_\mathrm{dc}$")
ax1.set_ylabel("Voltage (p.u.)")
ax1.legend(loc="lower right")
ax2.plot(t, complex2abc(grid_ts.ac_filter.i_g_ab)[0] / base.i, label=r"$i_\mathrm{ga}$")
ax2.set_ylabel("Current (p.u.)")
ax2.set_xlabel("Time (s)")
ax2.set_xlim(1.1, 1.125)
ax2.legend(loc="lower right")
plt.show()
