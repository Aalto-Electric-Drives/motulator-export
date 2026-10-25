classdef SaturatedSynchronousMachinePars < motulator.internal.Object
    %SATURATEDSYNCHRONOUSMACHINEPARS Parameters of a saturated synchronous machine
    %   with a GradNet flux map.
    %   PAR = SATURATEDSYNCHRONOUSMACHINEPARS(n_p=..., R_s=..., psi_s_dq_fcn=...) as
    %   in motulator, for the machine model of the control system. The flux map
    %   PSI_S_DQ_FCN is a struct of the parameters of a GradNet (in_dim, mu_dim, W,
    %   b, mu_log, bias, activation, beta_log, p, in_base, and out_base), e.g.,
    %   written by motulator_export.simulink.write_model.

    properties
        n_p                % Number of pole pairs
        R_s                % Stator resistance (Ohm)
        psi_s_dq_fcn = []  % GradNet flux map psi_s_dq(i_s_dq) (struct)
        G_c = 0            % Core-loss conductance (S), only 0
    end

    methods
        function obj = SaturatedSynchronousMachinePars(varargin)
            obj.set_args(varargin, {'n_p', 'R_s'}, {'n_p', 'R_s', 'psi_s_dq_fcn'});
            motulator.internal.check_zero(obj, 'G_c', 'Core losses');
        end
    end
end
