classdef VectorControlSystem < motulator.internal.Object
    %VECTORCONTROLSYSTEM Vector control system of a machine drive.
    %   CTRL = VECTORCONTROLSYSTEM(VECTOR_CTRL, SPEED_CTRL, PWM) as in motulator,
    %   with the vector controller VECTOR_CTRL (FluxVectorController of a
    %   synchronous machine or CurrentVectorController of an induction machine), the
    %   speed controller SPEED_CTRL (SpeedController), and the PWM (by default
    %   PWM()). SET_SPEED_REF(W_M_REF) sets the speed reference (mechanical rad/s):
    %   a constant or a StepSignal.

    properties
        vector_ctrl      % Vector controller
        speed_ctrl = []  % Speed controller (SpeedController)
        pwm = []         % Pulse-width modulator (PWM)
        w_M_ref = []     % Speed reference (rad/s), a constant or a StepSignal
    end

    methods
        function obj = VectorControlSystem(varargin)
            obj.set_args(varargin, {'vector_ctrl', 'speed_ctrl', 'pwm'}, ...
                {'vector_ctrl', 'speed_ctrl'});
            if isempty(obj.pwm)
                obj.pwm = motulator.common.control.PWM();
            end
        end

        function set_speed_ref(obj, w_M_ref)
            %SET_SPEED_REF Set the speed reference (mechanical rad/s).
            obj.w_M_ref = w_M_ref;
        end
    end
end
