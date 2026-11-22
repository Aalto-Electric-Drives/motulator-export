classdef ExternalRotorSpeed < motulator.internal.Object
    %EXTERNALROTORSPEED Rotor angle integrated from an external rotor speed.
    %   MECHANICS = EXTERNALROTORSPEED(W_M) as in motulator. The rotor speed W_M is a
    %   constant or a StepSignal, defaults to 0, and it can be changed later through
    %   the property w_M. The state (state.theta_M) is the initial rotor angle.

    properties
        state = struct('theta_M', 0)  % Initial rotor angle
        w_M = 0        % External rotor speed (rad/s), a constant or a StepSignal
    end

    methods
        function obj = ExternalRotorSpeed(varargin)
            obj.set_args(varargin, {'w_M'}, {});
        end
    end
end
