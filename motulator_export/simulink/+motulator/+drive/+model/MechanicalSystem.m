classdef MechanicalSystem < motulator.internal.Object
    %MECHANICALSYSTEM Stiff mechanical system.
    %   MECHANICS = MECHANICALSYSTEM(J=...) as in motulator, without friction (B_L
    %   = 0). The state (state.w_M and state.theta_M) is the initial rotor speed and
    %   angle, and SET_EXTERNAL_LOAD_TORQUE(TAU_L) sets the load torque: a constant
    %   or a StepSignal.

    properties
        J              % Total moment of inertia (kgm^2)
        B_L = 0        % Friction coefficient (Nm/(rad/s)), only 0
        state = struct('w_M', 0, 'theta_M', 0)  % Initial rotor speed and angle
        tau_L = 0      % External load torque (Nm), a constant or a StepSignal
    end

    methods
        function obj = MechanicalSystem(varargin)
            obj.set_args(varargin, {'J', 'B_L'}, {'J'});
            motulator.internal.check_zero(obj, 'B_L', 'Friction');
        end

        function set_external_load_torque(obj, tau_L)
            %SET_EXTERNAL_LOAD_TORQUE Set the external load torque (Nm).
            obj.tau_L = tau_L;
        end
    end
end
