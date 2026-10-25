% Build the Simulink model im_2kw_vhz_open_loop.slx in the folder of this script.
% Written by motulator_export.simulink from motulator: the system is built
% with the MATLAB classes of motulator, as in a motulator script, and the
% model from the blocks of the library of motulator.

here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', '..', 'motulator_export', 'simulink'));
import motulator.drive.model.*
import motulator.drive.control.im.*
import motulator.drive.utils.*

%% System model
par = InductionMachinePars(n_p=2, R_s=3.7, R_r=2.5122070312499996, L_ell=0.02296875, L_s=0.245);
machine = InductionMachine(par);
mechanics = MechanicalSystem(J=0.015);
converter = VoltageSourceConverter(u_dc=540);
mdl = Drive(machine, mechanics, converter);

%% Control system ([] = None, the default)
est_par = InductionMachineInvGammaPars(n_p=2, R_s=0, R_R=0, L_sgm=0, L_M=inf);
cfg = ObserverBasedVHzControllerCfg(psi_s_nom=1.0395957349782348, i_s_max=inf, alpha_psi=0, alpha_tau=0, alpha_f=0);
vhz_ctrl = ObserverBasedVHzController(est_par, cfg);
ctrl = VHzControlSystem(vhz_ctrl, slew_rate=2*pi*60);
ctrl.set_speed_ref(StepSignal(time=0.2, after=2*pi*25));
mdl.mechanics.set_external_load_torque(StepSignal(time=0.8, after=14.6));

%% Simulink model (the blocks of the control system are S-functions)
simulation = Simulation(mdl, ctrl);
simulation.build('im_2kw_vhz_open_loop', t_stop=1.6, folder=here);
% Simulate the model and plot the results:
%   res = simulation.simulate();
%   res.plot();
