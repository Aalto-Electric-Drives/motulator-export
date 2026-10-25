classdef StepSignal < motulator.internal.Object
    %STEPSIGNAL Step signal: before at first and after at t > time.
    %   SIG = STEPSIGNAL(TIME, AFTER, BEFORE) or STEPSIGNAL(time=..., after=...,
    %   before=...) as StepSignal of motulator_export. Several steps are given as
    %   vectors: AFTER(k) is the value after the step at TIME(k), the times being
    %   increasing. The references and the load torque are constants or step
    %   signals, which the Simulink model has as Constant or Step blocks.

    properties
        time        % Time of the step (s), or the times of the steps
        after       % Value after the step, or the values after the steps
        before = 0  % Value before the first step
    end

    methods
        function obj = StepSignal(varargin)
            obj.set_args(varargin, {'time', 'after', 'before'}, {'time', 'after'});
            times = obj.time(:);
            values = [obj.before; obj.after(:)];
            if numel(times) ~= numel(obj.after)
                error('motulator:arguments', ...
                    'StepSignal: time and after must have the same number of steps.');
            end
            if ~all(isfinite([times; values])) || any(diff(times) < 0)
                error('motulator:arguments', ['StepSignal: the times and the ' ...
                    'values must be finite, and the times increasing.']);
            end
        end

        function y = value(obj, t)
            %VALUE Value at the time T (a scalar or an array).
            levels = [obj.before; obj.after(:)];
            y = obj.before + zeros(size(t));
            for k = 1:numel(obj.time)
                y = y + (t > obj.time(k))*(levels(k + 1) - levels(k));
            end
        end

        function sig = sampled(obj, T_s)
            %SAMPLED Step signal switching at the same sample as in motulator.
            %   The controller of motulator evaluates a reference (t > time) at its
            %   time t, accumulated by adding the sampling period T_S, so the step
            %   is moved halfway between the samples (sampled_step of
            %   motulator_export).
            if ~(isfinite(T_s) && T_s > 0) || any(obj.time >= 1e9)
                error('motulator:arguments', ['StepSignal: the sampling period ' ...
                    'must be positive and the step times below 1e9 s.']);
            end
            times = zeros(size(obj.time));
            for k = 1:numel(obj.time)
                t = 0;
                n = 0;
                while ~(t > obj.time(k))
                    t = mod(t + T_s, 1e9); % As in ControlSystem.update of motulator
                    n = n + 1;
                end
                times(k) = (n - 0.5)*T_s;
            end
            sig = motulator.common.utils.StepSignal(times, obj.after, obj.before);
        end

        function rows = steps(obj)
            %STEPS Rows [time before after] of the Step blocks of the signal.
            %   The first row steps from before to the first value, and the others
            %   from zero to the increments, so that the sum of the steps is the
            %   signal.
            levels = [obj.before; obj.after(:)];
            rows = [obj.time(:), [obj.before; zeros(numel(obj.time) - 1, 1)], ...
                [levels(2); diff(levels(2:end))]];
        end
    end
end
