% Build the Simulink model gfm_13kva_do.slx in the folder of this script.
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
ac_filter = LFilter(L_f=0.006125876615797688, R_f=0.6415002990995841, L_g=0.030220991304601926);
ac_source = ThreePhaseSource();
mdl = GridConverterSystem(converter, ac_filter, ac_source);

%% Control system ([] = None, the default)
cfg = ObserverBasedGridFormingControllerCfg(i_max=33.09259735953043, L=0.014293712103527936, R=0.6415002990995841, R_a=2.5660011963983362);
inner_ctrl = ObserverBasedGridFormingController(cfg);
ctrl = GridConverterControlSystem(inner_ctrl);
ctrl.set_power_ref(StepSignal(time=[0.2 0.5 0.8 1.2], after=[4166.666666666667 8333.333333333334 12500 -12500]));
ctrl.set_ac_voltage_ref(326.5986323710904);

%% Simulink model (the blocks of the control system are S-functions)
simulation = Simulation(mdl, ctrl);
simulation.build('gfm_13kva_do', t_stop=1.4, folder=here);
% Simulate the model and plot the results:
%   res = simulation.simulate();
%   res.plot();
