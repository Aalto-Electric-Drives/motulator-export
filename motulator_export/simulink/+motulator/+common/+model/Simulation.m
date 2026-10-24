classdef Simulation < handle
    %SIMULATION Simulation of a system model and a control system in Simulink.
    %   SIM = SIMULATION(MDL, CTRL) as in motulator, with the system model MDL
    %   (Drive or GridConverterSystem) and the control system CTRL.
    %
    %   SIM.BUILD(NAME, t_stop=..., folder=..., code=...) builds the Simulink model
    %   NAME from the blocks of the library of motulator and saves it as NAME.slx
    %   in the folder FOLDER (by default the current folder), with the stop time
    %   T_STOP (s, by default 1). The blocks of the control system are S-functions
    %   (CODE 'c', the default) or MATLAB Function blocks (CODE 'matlab'), see
    %   motulator.build_library. The parameters of the model, i.e., the arguments
    %   of the parts of MDL and CTRL as structs, are in its model workspace, and
    %   the masks of the blocks refer to them.
    %
    %   RES = SIM.SIMULATE(t_stop=...) simulates the built model, by default to its
    %   stop time, and returns the results (SimulationResults).

    properties
        mdl          % System model (Drive or GridConverterSystem)
        ctrl         % Control system
        name = ''    % Name of the built model
        file = ''    % File of the built model
    end

    properties (Access = private)
        mdl_signals = {}   % Names of the signals of the output port mdl
        ctrl_signals = {}  % Names of the signals of the output port ctrl
        scope = {}         % Groups of the signals {name, indices in [mdl; ctrl]}
    end

    methods
        function obj = Simulation(mdl, ctrl)
            obj.mdl = mdl;
            obj.ctrl = ctrl;
        end

        function build(obj, name, varargin)
            %BUILD Build the Simulink model and save it.
            opts = motulator.internal.options(varargin, ...
                struct('t_stop', 1, 'folder', pwd, 'code', 'c'));
            code = char(opts.code);
            [variables, builder, obj.mdl_signals] = ...
                motulator.internal.system_model(obj.mdl);
            c = motulator.internal.control_system(obj.ctrl, obj.mdl);
            lib = motulator.internal.library(code);
            fields = fieldnames(c.variables);
            for k = 1:numel(fields)
                variables.(fields{k}) = c.variables.(fields{k});
            end
            name = char(name);
            folder = char(opts.folder);
            motulator.internal.new_model(name, folder, variables, c.T_s, opts.t_stop);
            info = motulator.internal.block_info(code, c.name);
            obj.scope = builder(name, obj.mdl, c, lib, info);
            obj.ctrl_signals = info.signals;
            obj.file = fullfile(folder, [name '.slx']);
            save_system(name, obj.file);
            obj.name = name;
            fprintf('Saved %s\n', obj.file);
        end

        function res = simulate(obj, varargin)
            %SIMULATE Simulate the built model.
            opts = motulator.internal.options(varargin, struct('t_stop', []));
            if isempty(obj.name)
                error('motulator:simulation', 'Build the model first (build).');
            end
            args = {};
            if ~isempty(opts.t_stop)
                args = {'StopTime', motulator.internal.num(opts.t_stop)};
            end
            load_system(obj.file);
            out = sim(obj.name, args{:});
            res = motulator.common.model.SimulationResults(out.tout, out.yout, ...
                obj.mdl_signals, obj.ctrl_signals, obj.scope);
        end
    end
end
