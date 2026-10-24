classdef SpeedController < motulator.internal.Object
    %SPEEDCONTROLLER 2DOF PI speed controller.
    %   SPEED_CTRL = SPEEDCONTROLLER(J=..., alpha_s=..., alpha_i=..., tau_M_max=...)
    %   as in motulator.

    properties
        J                % Total moment of inertia (kgm^2)
        alpha_s          % Reference-tracking bandwidth (rad/s)
        alpha_i = []     % Integral-action bandwidth (rad/s), [] = alpha_s
        tau_M_max = inf  % Maximum motor torque (Nm)
    end

    methods
        function obj = SpeedController(varargin)
            obj.set_args(varargin, {'J', 'alpha_s', 'alpha_i', 'tau_M_max'}, ...
                {'J', 'alpha_s'});
        end
    end
end
