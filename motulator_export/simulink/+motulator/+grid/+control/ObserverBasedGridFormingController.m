classdef ObserverBasedGridFormingController < motulator.internal.Object
    %OBSERVERBASEDGRIDFORMINGCONTROLLER Disturbance-observer-based grid-forming
    %   controller with the transparent current limitation.
    %   INNER_CTRL = OBSERVERBASEDGRIDFORMINGCONTROLLER(CFG) as in motulator, with
    %   the configuration CFG (ObserverBasedGridFormingControllerCfg).

    properties
        cfg  % Configuration (ObserverBasedGridFormingControllerCfg)
    end

    methods
        function obj = ObserverBasedGridFormingController(varargin)
            obj.set_args(varargin, {'cfg'}, {'cfg'});
        end
    end
end
