classdef ThreePhaseSource < motulator.internal.Object
    %THREEPHASESOURCE Balanced three-phase voltage source of the grid.
    %   AC_SOURCE = THREEPHASESOURCE(w_g=..., e_g=...) as in motulator, with constant
    %   magnitude and frequency (phi = e_g_neg = phi_neg = 0).

    properties
        w_g = 2*pi*50         % Angular frequency (rad/s)
        e_g = sqrt(2/3)*400   % Magnitude (V, peak, line-neutral)
        phi = 0               % Phase angle (rad), only 0
        e_g_neg = 0           % Magnitude of the negative sequence (V), only 0
        phi_neg = 0           % Phase angle of the negative sequence (rad), only 0
    end

    methods
        function obj = ThreePhaseSource(varargin)
            obj.set_args(varargin, {'w_g', 'e_g', 'phi', 'e_g_neg', 'phi_neg'}, {});
            for name = {'phi', 'e_g_neg', 'phi_neg'}
                motulator.internal.check_zero(obj, name{1}, ...
                    'Phase or negative sequence');
            end
        end
    end
end
