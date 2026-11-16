classdef OpenLoopVHzController < motulator.internal.Object
    %OPENLOOPVHZCONTROLLER Open-loop V/Hz controller of an induction machine.
    %   VHZ_CTRL = OPENLOOPVHZCONTROLLER(N_P, U_S_REF, T_s=...) as in motulator, with
    %   the number of pole pairs N_P and the V/Hz curve U_S_REF, a function of the
    %   stator angular frequency (rad/s) giving the stator voltage reference (V).
    %   Only affine curves are supported, e.g., @(w_s) 1i*w_s*psi_s_nom.

    properties
        n_p           % Number of pole pairs
        u_s_ref       % V/Hz curve, a function of the stator angular frequency
        T_s = 250e-6  % Sampling period (s)
    end

    methods
        function obj = OpenLoopVHzController(varargin)
            obj.set_args(varargin, {'n_p', 'u_s_ref', 'T_s'}, {'n_p', 'u_s_ref'});
            [u_0, k] = obj.coefficients();
            w_s = 2*pi*linspace(-1000, 1000, 21);
            u = arrayfun(obj.u_s_ref, w_s);
            if any(abs(u - (u_0 + k*w_s)) > 1e-9*(abs(u_0) + abs(k)*abs(w_s)))
                error('motulator:unsupported', ...
                    'Only an affine V/Hz curve u_s_ref supported.');
            end
        end

        function [u_0, k] = coefficients(obj)
            %COEFFICIENTS Coefficients of the V/Hz curve u_0 + k*w_s: the voltage u_0
            %   at zero frequency (V) and the slope k (Vs).
            u_0 = obj.u_s_ref(0);
            k = obj.u_s_ref(1) - u_0;
        end

        function s = params(obj)
            %PARAMS Arguments as a struct, the V/Hz curve by its coefficients, each
            %   [real part, imaginary part].
            [u_0, k] = obj.coefficients();
            curve = struct('u_0', [real(u_0) imag(u_0)], 'k', [real(k) imag(k)]);
            s = struct('n_p', obj.n_p, 'u_s_ref', curve, 'T_s', obj.T_s);
        end
    end
end
