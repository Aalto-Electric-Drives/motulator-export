% Build the Simulink model im_2kw_cvc_discrete.slx in the folder of this script.
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
mechanics = ExternalRotorSpeed();
converter = VoltageSourceConverter(u_dc=540);
mdl = Drive(machine, mechanics, converter);

%% Control system ([] = None, the default)
est_par = InductionMachineInvGammaPars(n_p=2, R_s=3.7, R_R=2.1, L_sgm=0.021, L_M=0.224);
cfg = CurrentVectorControllerCfg(psi_s_nom=1.0395957349782348, i_s_max=14.142135623730951, alpha_c=2*pi*700, T_s=0.0002, discrete=1);
vector_ctrl = CurrentVectorController(est_par, cfg);
vector_ctrl.observer.w_M = 2*pi*12.5;
ctrl = VectorControlSystem(vector_ctrl);
ctrl.set_torque_ref(StepSignal(time=[0.3 0.32 0.34], after=[4.38 7.3 -4.38]));
mdl.mechanics.set_external_rotor_speed(2*pi*12.5);

%% Simulink model (the blocks of the control system are S-functions)
simulation = Simulation(mdl, ctrl);
simulation.build('im_2kw_cvc_discrete', t_stop=0.37, folder=here);
% Simulate the model and plot the results:
%   res = simulation.simulate();
%   res.plot();
