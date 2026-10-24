% Build the Simulink model ipmsm_2kw_fvc.slx in the folder of this script.
% Written by motulator_export.simulink from motulator: the system is built
% with the MATLAB classes of motulator, as in a motulator script, and the
% model from the blocks of the library of motulator.

here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', '..', 'motulator_export', 'simulink'));
import motulator.drive.model.*
import motulator.drive.control.sm.*
import motulator.drive.utils.*

%% System model
par = SynchronousMachinePars(n_p=3, R_s=3.6, L_d=0.036, L_q=0.051, psi_f=0.545);
machine = SynchronousMachine(par);
mechanics = MechanicalSystem(J=0.015);
converter = VoltageSourceConverter(u_dc=540);
mdl = Drive(machine, mechanics, converter);

%% Control system ([] = None, the default)
cfg = FluxVectorControllerCfg(i_s_max=6.5);
vector_ctrl = FluxVectorController(par, cfg);
speed_ctrl = SpeedController(J=0.015, alpha_s=25);
ctrl = VectorControlSystem(vector_ctrl, speed_ctrl);
ctrl.set_speed_ref(StepSignal(time=0.1, after=50));
mdl.mechanics.set_external_load_torque(StepSignal(time=0.7, after=10));

%% Simulink model (the blocks of the control system are S-functions)
simulation = Simulation(mdl, ctrl);
simulation.build('ipmsm_2kw_fvc', t_stop=1.2, folder=here);
% Simulate the model and plot the results:
%   res = simulation.simulate();
%   res.plot();
