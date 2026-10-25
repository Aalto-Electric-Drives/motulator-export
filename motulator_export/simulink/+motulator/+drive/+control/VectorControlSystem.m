classdef VectorControlSystem < motulator.internal.Object
    %VECTORCONTROLSYSTEM Vector control system of a machine drive.
    %   CTRL = VECTORCONTROLSYSTEM(VECTOR_CTRL, SPEED_CTRL, PWM) as in motulator,
    %   with the vector controller VECTOR_CTRL (FluxVectorController or
    %   CurrentVectorController of a synchronous or an induction machine), the speed
    %   controller SPEED_CTRL (SpeedController; by default [], i.e., the
    %   torque-control mode), and the PWM (by default PWM()). SET_SPEED_REF(W_M_REF)
    %   sets the speed reference (mechanical rad/s) and, in the torque-control mode,
    %   SET_TORQUE_REF(TAU_M_REF) the torque reference (Nm): a constant or a
    %   StepSignal.

    properties
        vector_ctrl      % Vector controller
        speed_ctrl = []  % Speed controller (SpeedController), [] in torque control
        pwm = []         % Pulse-width modulator (PWM)
        w_M_ref = []     % Speed reference (rad/s), a constant or a StepSignal
        tau_M_ref = []   % Torque reference (Nm), a constant or a StepSignal
    end

    methods
        function obj = VectorControlSystem(varargin)
            obj.set_args(varargin, {'vector_ctrl', 'speed_ctrl', 'pwm'}, ...
                {'vector_ctrl'});
            if isempty(obj.pwm)
                obj.pwm = motulator.common.control.PWM();
            end
        end

        function set_speed_ref(obj, w_M_ref)
            %SET_SPEED_REF Set the speed reference (mechanical rad/s).
            if isempty(obj.speed_ctrl)
                error('motulator:arguments', ...
                    'Speed-control mode requires speed_ctrl.');
            end
            obj.w_M_ref = w_M_ref;
        end

        function set_torque_ref(obj, tau_M_ref)
            %SET_TORQUE_REF Set the torque reference (Nm) of the torque-control mode.
            if ~isempty(obj.speed_ctrl)
                error('motulator:arguments', ...
                    'The speed controller gives the torque reference.');
            end
            obj.tau_M_ref = tau_M_ref;
        end
    end
end
