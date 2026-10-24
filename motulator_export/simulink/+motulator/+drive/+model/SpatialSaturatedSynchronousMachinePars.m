classdef SpatialSaturatedSynchronousMachinePars < motulator.internal.Object
    %SPATIALSATURATEDSYNCHRONOUSMACHINEPARS Parameters of a saturated synchronous
    %   machine with spatial harmonics, given by a GradNet current map.
    %   PAR = SPATIALSATURATEDSYNCHRONOUSMACHINEPARS(n_p=..., R_s=...,
    %   magnetic_map_fcn=...) as in motulator, for the machine model of the system.
    %   The current map MAGNETIC_MAP_FCN is a struct of the parameters of a GradNet
    %   (see SaturatedSynchronousMachinePars) with the order k of the spatial
    %   harmonics and the PM-flux linkage psi_f (the initial flux linkage), e.g.,
    %   written by motulator_export.simulink.write_model.

    properties
        n_p                    % Number of pole pairs
        R_s                    % Stator resistance (Ohm)
        magnetic_map_fcn = []  % GradNet current map i_s_dq(psi_s_dq, theta_m)
        G_c = 0                % Core-loss conductance (S), only 0
    end

    methods
        function obj = SpatialSaturatedSynchronousMachinePars(varargin)
            args = {'n_p', 'R_s', 'magnetic_map_fcn'};
            obj.set_args(varargin, [args, {'G_c'}], args);
            motulator.internal.check_zero(obj, 'G_c', 'Core losses');
        end
    end
end
