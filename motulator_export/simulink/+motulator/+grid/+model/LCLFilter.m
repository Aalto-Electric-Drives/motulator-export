classdef LCLFilter < motulator.internal.Object
    %LCLFILTER LCL filter.
    %   AC_FILTER = LCLFILTER(L_fc=..., L_fg=..., C_f=..., u_f0_ab=...) as in
    %   motulator, without the resistances and the grid impedance, with the initial
    %   capacitor voltage u_f0_ab (V, a complex space vector).

    properties
        L_fc         % Converter-side inductance (H)
        L_fg         % Grid-side inductance (H)
        C_f          % Filter capacitance (F)
        R_fc = 0     % Converter-side resistance (Ohm), only 0
        R_fg = 0     % Grid-side resistance (Ohm), only 0
        L_g = 0      % Grid inductance (H), only 0
        R_g = 0      % Grid resistance (Ohm), only 0
        u_f0_ab = 0  % Initial capacitor voltage (V)
    end

    methods
        function obj = LCLFilter(varargin)
            obj.set_args(varargin, {'L_fc', 'L_fg', 'C_f', 'R_fc', 'R_fg', 'L_g', ...
                'R_g', 'u_f0_ab'}, {'L_fc', 'L_fg', 'C_f'});
            for name = {'R_fc', 'R_fg', 'L_g', 'R_g'}
                motulator.internal.check_zero(obj, name{1}, ...
                    'Grid impedance or resistance');
            end
        end
    end
end
