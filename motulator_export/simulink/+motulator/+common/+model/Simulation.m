classdef Simulation < handle
    %SIMULATION Simulation of a system model and a control system in Simulink.
    %   SIM = SIMULATION(MDL, CTRL) as in motulator, with the system model MDL
    %   (Drive or GridConverterSystem) and the control system CTRL.
    %
    %   SIM.BUILD(NAME, t_stop=..., folder=..., code=...) builds the Simulink model
    %   NAME and saves it as NAME.slx in the folder FOLDER (by default the current
    %   folder), with the stop time T_STOP (s, by default 1).
    %
    %   With CODE 'c' (the default), the blocks are links to the library of
    %   motulator (see motulator.build_library), whose control systems run
    %   S-functions. The parameters of the model, i.e., the arguments of the parts
    %   of MDL and CTRL as structs, are in its model workspace, and the masks of the
    %   blocks refer to them.
    %
    %   With CODE 'matlab', the control system runs MATLAB Function blocks, and the
    %   model needs neither the library nor this package: the blocks are in the
    %   model, and the MATLAB Function blocks contain their code. The blocks of the
    %   control system have no masks: each refers to its struct in the struct ctrl
    %   of the parameters of the blocks (e.g., ctrl.SpeedController), which is
    %   computed from the arguments of CTRL by the function of the parameters of
    %   the control system (e.g., fvc_params.m in the folder library of the
    %   package). The parameters of the model, i.e., ctrl and the arguments of the
    %   parts of MDL as structs, are in the script init_NAME.m, saved next to the
    %   model, which runs it at the start of a simulation (its callback InitFcn). A
    %   block copied to another model needs only the variables of the script.
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
            if ~any(strcmp(code, {'c', 'matlab'}))
                error('motulator:arguments', 'The code must be ''c'' or ''matlab''.');
            end
            [variables, builder, obj.mdl_signals] = ...
                motulator.internal.system_model(obj.mdl);
            c = motulator.internal.control_system(obj.ctrl, obj.mdl);
            name = char(name);
            folder = char(opts.folder);
            motulator.setup();
            info = motulator.internal.block_info(code, c.name);
            if strcmp(code, 'matlab')
                % The parameters in the initialization script, which the model
                % runs at each start, and the blocks in the model
                lib = '';
                ctrl_params = motulator.internal.control_params(info, c.variables);
                c.T_s_expr = 'ctrl.T_s';
                init = ['init_' name];
                motulator.internal.write_init(fullfile(folder, [init '.m']), name, ...
                    {'System model', variables
                    ['Blocks of the control system (' info.params ')'], ...
                    struct('ctrl', ctrl_params)});
                init_fcn = sprintf( ...
                    'run(fullfile(fileparts(get_param(bdroot, ''FileName'')), ''%s.m''));', ...
                    init);
                variables = struct();
                % The parameters for the build
                evalin('base', sprintf('run(''%s'');', ...
                    strrep(fullfile(folder, [init '.m']), '''', '''''')));
            else
                lib = motulator.internal.library();
                fields = fieldnames(c.variables);
                for k = 1:numel(fields)
                    variables.(fields{k}) = c.variables.(fields{k});
                end
                init_fcn = '';
            end
            motulator.internal.new_model(name, folder, variables, c.T_s, ...
                opts.t_stop, init_fcn);
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
