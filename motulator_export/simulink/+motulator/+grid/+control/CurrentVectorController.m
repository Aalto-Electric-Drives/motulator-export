classdef CurrentVectorController < motulator.internal.Object
    %CURRENTVECTORCONTROLLER Grid-following current-vector controller with a PLL.
    %   INNER_CTRL = CURRENTVECTORCONTROLLER(CFG) as in motulator, with the
    %   configuration CFG (CurrentVectorControllerCfg).

    properties
        cfg  % Configuration (CurrentVectorControllerCfg)
    end

    methods
        function obj = CurrentVectorController(varargin)
            obj.set_args(varargin, {'cfg'}, {'cfg'});
        end
    end
end
