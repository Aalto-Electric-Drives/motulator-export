classdef LFilter < motulator.internal.Object
    %LFILTER L filter.
    %   AC_FILTER = LFILTER(L_f=..., R_f=..., L_g=...) as in motulator, with the grid
    %   inductance L_g, without the grid resistance.

    properties
        L_f       % Filter inductance (H)
        R_f = 0   % Filter resistance (Ohm)
        L_g = 0   % Grid inductance (H)
        R_g = 0   % Grid resistance (Ohm), only 0
    end

    methods
        function obj = LFilter(varargin)
            obj.set_args(varargin, {'L_f', 'R_f', 'L_g', 'R_g'}, {'L_f'});
            motulator.internal.check_zero(obj, 'R_g', 'Grid resistance');
        end
    end
end
