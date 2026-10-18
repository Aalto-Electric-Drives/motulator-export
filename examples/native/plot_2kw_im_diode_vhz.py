"""
2.2-kW IM, diode bridge, V/Hz control
=====================================

This example simulates pure open-loop V/Hz control of a 2.2-kW induction machine (IM)
drive natively. The system is that of the *motulator* example
:doc:`motulator:drive_examples/vhz/plot_2kw_im_diode_vhz`: a diode bridge, a stiff
three-phase grid, and a DC link are modeled, the load torque is quadratic, and the
converter uses carrier comparison.

"""

# %%
from math import inf, pi

import motulator.drive.control.im as control
from motulator.drive import model, utils

from motulator_export import native

# %%
# Compute base values based on the nominal values.

nom = utils.NominalValues(U=400, I=5, f=50, P=2.2e3, tau=14.6)
base = utils.BaseValues.from_nominal(nom, n_p=2)

# %%
# Configure the system model.

par = model.InductionMachineInvGammaPars(
    n_p=2, R_s=3.7, R_R=2.1, L_sgm=0.021, L_M=0.224
)
machine = model.InductionMachine(par)
k = 1.1 * nom.tau / base.w_M**2  # Quadratic load torque profile
mechanics = model.MechanicalSystem(J=0.015, B_L=lambda w_M: k * abs(w_M))
converter = model.VoltageSourceConverter()
dc_bus = model.DiodeBridgeDCBus(
    C_dc=235e-6, L_dc=2e-3, U_g=nom.U, f_g=nom.f, converters=[converter]
)
mdl = model.Drive(machine, mechanics, converter, pwm=True, dc_bus=dc_bus)

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
mdl.mechanics.set_external_load_torque(lambda t: (t > 1) * 0.2 * nom.tau)

# %%
# Simulate natively and plot the results.

sim = native.Simulation(mdl, ctrl)
res = sim.simulate(t_stop=1.4)
utils.plot(res, base)
utils.plot_stator_waveforms(res, base, t_lims=(1.1, 1.125))
utils.plot_dc_bus_waveforms(res, base, t_lims=(1.1, 1.125))
