classdef ExternalRotorSpeed < motulator.internal.Object
    %EXTERNALROTORSPEED Rotor angle integrated from an external rotor speed.
    %   MECHANICS = EXTERNALROTORSPEED() as in motulator. The state (state.theta_M)
    %   is the initial rotor angle, and SET_EXTERNAL_ROTOR_SPEED(W_M) sets the rotor
    %   speed: a constant or a StepSignal.

    properties
        state = struct('theta_M', 0)  % Initial rotor angle
        w_M = 0        % External rotor speed (rad/s), a constant or a StepSignal
    end

    methods
        function obj = ExternalRotorSpeed(varargin)
            obj.set_args(varargin, {}, {});
        end

        function set_external_rotor_speed(obj, w_M)
            %SET_EXTERNAL_ROTOR_SPEED Set the external rotor speed (rad/s).
            obj.w_M = w_M;
        end
    end
end
