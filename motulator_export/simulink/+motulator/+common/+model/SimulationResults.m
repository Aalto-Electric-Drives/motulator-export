classdef SimulationResults < handle
    %SIMULATIONRESULTS Results of a simulation in Simulink.
    %   The signals of the system model (mdl, e.g., the phase currents i_a, i_b,
    %   and i_c, the rotor speed w_M, and the torque tau_M of a drive) and the
    %   monitored signals of the control system (ctrl, e.g., the speed estimate w_M
    %   and the torque reference tau_M_ref) are the fields of the structs MDL and
    %   CTRL, column vectors with the time t (s). The names are those of
    %   motulator_export.simulink.simulate. RES.PLOT shows the signals as the scope
    %   of the model, and RES.PLOT(BASE) in per-unit values of the base values BASE
    %   (BaseValues).

    properties
        mdl   % Signals of the system model, with the time t
        ctrl  % Monitored signals of the control system, with the time t
    end

    properties (Access = private)
        scope  % Groups of the signals {name, indices in [mdl; ctrl]}
    end

    methods
        function obj = SimulationResults(t, y, mdl_signals, ctrl_signals, scope)
            obj.mdl = struct('t', t);
            obj.ctrl = struct('t', t);
            n = numel(mdl_signals);
            for k = 1:n
                obj.mdl.(mdl_signals{k}) = y(:, k);
            end
            for k = 1:numel(ctrl_signals)
                obj.ctrl.(ctrl_signals{k}) = y(:, n + k);
            end
            obj.scope = scope;
        end

        function plot(obj, base)
            %PLOT Plot the signals in the groups of the scope of the model.
            if nargin < 2
                base = [];
            end
            names = [fieldnames(obj.mdl); fieldnames(obj.ctrl)];
            values = [struct2cell(obj.mdl); struct2cell(obj.ctrl)];
            % The signals without the times
            keep = ~strcmp(names, 't');
            names = names(keep);
            values = values(keep);
            n_mdl = numel(fieldnames(obj.mdl)) - 1;
            figure;
            n = size(obj.scope, 1);
            for k = 1:n
                ax = subplot(n, 1, k);
                hold(ax, 'on');
                labels = cell(1, 0);
                for j = obj.scope{k, 2}
                    plot(ax, obj.mdl.t, values{j}/scale(names{j}, base));
                    if j <= n_mdl
                        labels{end + 1} = [names{j} ' (mdl)']; %#ok<AGROW>
                    else
                        labels{end + 1} = names{j}; %#ok<AGROW>
                    end
                end
                legend(ax, labels, 'Interpreter', 'none', 'Location', 'eastoutside');
                if isempty(base)
                    ylabel(ax, obj.scope{k, 1});
                else
                    ylabel(ax, [obj.scope{k, 1} ' (p.u.)']);
                end
                grid(ax, 'on');
                xlim(ax, [obj.mdl.t(1) obj.mdl.t(end)]);
            end
            xlabel(ax, 'Time (s)');
        end
    end
end

function s = scale(name, base)
% Base value of a signal by its name, 1 without the base values
s = 1;
if isempty(base)
    return
end
if strncmp(name, 'w_M', 3)
    s = base.w_M;
elseif strncmp(name, 'w_', 2)
    s = base.w;
elseif strncmp(name, 'tau', 3)
    s = base.tau;
elseif strncmp(name, 'psi', 3)
    s = base.psi;
elseif strncmp(name, 'i_', 2)
    s = base.i;
elseif strncmp(name, 'p_', 2) || strncmp(name, 'q_', 2)
    s = base.p;
elseif any(strncmp(name, {'u_', 'v_', 'e_'}, 2))
    s = base.u;
end
end
