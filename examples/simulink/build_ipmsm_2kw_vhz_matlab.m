% Build the Simulink model ipmsm_2kw_vhz_matlab.slx in the folder of this script.
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
cfg = ObserverBasedVHzControllerCfg(i_s_max=9.121677477306463);
vhz_ctrl = ObserverBasedVHzController(par, cfg);
ctrl = VHzControlSystem(vhz_ctrl, slew_rate=2*pi*120);
ctrl.set_speed_ref(StepSignal(time=0.2, after=2*pi*25));
mdl.mechanics.set_external_load_torque(StepSignal(time=0.8, after=14));

%% Simulink model (the blocks of the control system are MATLAB Function blocks)
simulation = Simulation(mdl, ctrl);
simulation.build('ipmsm_2kw_vhz_matlab', t_stop=1.6, folder=here, code='matlab');
% Simulate the model and plot the results:
%   res = simulation.simulate();
%   res.plot();
