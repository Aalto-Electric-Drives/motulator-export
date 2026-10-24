classdef PWM < motulator.internal.Object
    %PWM Pulse-width modulator of the control system.
    %   PWM = PWM(overmodulation=..., d_err=..., feedforward=..., d_min=...,
    %   single_update=...) as in motulator: space-vector PWM with the
    %   minimum-phase-error ('MPE') or the minimum-magnitude-error ('MME')
    %   overmodulation, the duty-ratio error model D_ERR (InverterNonlinearity, []
    %   = None) and its compensation (feedforward), the minimum duty ratio d_min of
    %   a switching leg, and the single update of the duty ratios once per carrier
    %   period (or twice).

    properties
        overmodulation = 'MPE'  % Overmodulation, 'MPE' or 'MME'
        d_err = []              % Duty-ratio error model (InverterNonlinearity)
        feedforward = true      % Compensate for the duty-ratio error
        k_pred = 1.5            % Prediction factor of the angle (only 1.5)
        d_min = 0               % Minimum duty ratio of a switching leg
        single_update = false   % Update the duty ratios once per carrier period
    end

    methods
        function obj = PWM(varargin)
            obj.set_args(varargin, {'overmodulation', 'd_err', 'feedforward', ...
                'k_pred', 'd_min', 'single_update'}, {});
            if ~any(strcmp(obj.overmodulation, {'MPE', 'MME'}))
                error('motulator:unsupported', ...
                    'Only the overmodulation ''MPE'' or ''MME'' supported.');
            end
            if obj.k_pred ~= 1.5
                error('motulator:unsupported', 'Only k_pred = 1.5 supported.');
            end
        end

        function s = params(obj)
            %PARAMS Arguments as the struct of the mask of the PWM, with the fields
            %   t_d, u_th, and i_0 of the duty-ratio error model (zeros if none).
            d_err = obj.d_err;
            if isempty(d_err)
                d_err = motulator.common.control.InverterNonlinearity();
            end
            s = struct('t_d', d_err.t_d, 'u_th', d_err.u_th, 'i_0', d_err.i_0, ...
                'feedforward', double(obj.feedforward), 'd_min', obj.d_min, ...
                'single_update', double(obj.single_update), ...
                'overmodulation', char(obj.overmodulation));
        end
    end
end
