% Build the Simulink model im_2kw_fvc_matlab.slx in the folder of this script.
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
est_par = InductionMachineInvGammaPars(n_p=2, R_s=3.7, R_R=2.1, L_sgm=0.021, L_M=0.224);
cfg = FluxVectorControllerCfg(psi_s_nom=0.987615948229323, i_s_max=10.606601717798213, alpha_i=0, J=0.015);
vector_ctrl = FluxVectorController(est_par, cfg);
speed_ctrl = SpeedController(J=0.015, alpha_s=2*pi*4);
ctrl = VectorControlSystem(vector_ctrl, speed_ctrl);
ctrl.set_speed_ref(StepSignal(time=0.2, after=2*pi*50));
mdl.mechanics.set_external_load_torque(StepSignal(time=0.8, after=7.3));

%% Simulink model (the blocks of the control system are MATLAB Function blocks)
simulation = Simulation(mdl, ctrl);
simulation.build('im_2kw_fvc_matlab', t_stop=1.5, folder=here, code='matlab');
% Simulate the model and plot the results:
%   res = simulation.simulate();
%   res.plot();
