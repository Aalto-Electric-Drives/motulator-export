% Build the Simulink model gfl_10kva_lcl_matlab.slx in the folder of this script.
% Written by motulator_export.simulink from motulator: the system is built
% with the MATLAB classes of motulator, as in a motulator script, and the
% model from the blocks of the library of motulator.

here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', '..', 'motulator_export', 'simulink'));
import motulator.grid.model.*
import motulator.grid.control.*
import motulator.grid.utils.*

%% System model
converter = VoltageSourceConverter(u_dc=650);
ac_filter = LCLFilter(L_fc=0.003700874424440534, L_fg=0.003700874424440534, C_f=8.593839157387184e-06, u_f0_ab=326.5986323710904);
ac_source = ThreePhaseSource();
mdl = GridConverterSystem(converter, ac_filter, ac_source);

%% Control system ([] = None, the default)
cfg = CurrentVectorControllerCfg(i_max=30.75914498161482, L=0.003700874424440534, T_s=0.0001);
inner_ctrl = CurrentVectorController(cfg);
ctrl = GridConverterControlSystem(inner_ctrl);
ctrl.set_power_ref(StepSignal(time=0.02, after=5000));
ctrl.set_reactive_power_ref(StepSignal(time=0.04, after=4000));

%% Simulink model (the blocks of the control system are MATLAB Function blocks)
simulation = Simulation(mdl, ctrl);
simulation.build('gfl_10kva_lcl_matlab', t_stop=0.08, folder=here, code='matlab');
% Simulate the model and plot the results:
%   res = simulation.simulate();
%   res.plot();
