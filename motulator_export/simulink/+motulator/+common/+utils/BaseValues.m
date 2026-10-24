classdef BaseValues < motulator.internal.Object
    %BASEVALUES Base values.
    %   BASE = BASEVALUES.FROM_NOMINAL(NOM) computes the base values from the
    %   nominal values NOM (NominalValues), and BASEVALUES.FROM_NOMINAL(NOM,
    %   n_p=N_P) also those of a machine with N_P pole pairs, as in motulator.

    properties
        u    % Voltage (V, peak, line-neutral)
        i    % Current (A, peak)
        w    % Angular frequency (rad/s)
        psi  % Flux linkage (Vs)
        p    % Power (W)
        Z    % Impedance (Ohm)
        L    % Inductance (H)
        C    % Capacitance (F)
        tau = 0  % Torque (Nm)
        n_p = 0  % Number of pole pairs
        w_M = 0  % Mechanical angular speed (rad/s)
    end

    methods
        function obj = BaseValues(varargin)
            obj.set_args(varargin, {'u', 'i', 'w', 'psi', 'p', 'Z', 'L', 'C'}, ...
                {'u', 'i', 'w', 'psi', 'p', 'Z', 'L', 'C'});
        end
    end

    methods (Static)
        function base = from_nominal(nom, varargin)
            %FROM_NOMINAL Base values from the nominal values.
            n_p = [];
            if ~isempty(varargin)
                if numel(varargin) == 1
                    n_p = varargin{1};
                elseif numel(varargin) == 2 && strcmp(char(varargin{1}), 'n_p')
                    n_p = varargin{2};
                else
                    error('motulator:arguments', ...
                        'from_nominal(nom, n_p=...) expected.');
                end
            end
            u = sqrt(2/3)*nom.U;
            i = sqrt(2)*nom.I;
            w = 2*pi*nom.f;
            psi = u/w;
            p = 1.5*u*i;
            Z = u/i;
            L = Z/w;
            C = 1/(Z*w);
            base = motulator.common.utils.BaseValues(u, i, w, psi, p, Z, L, C);
            if ~isempty(n_p)
                base.tau = n_p*p/w;
                base.n_p = n_p;
                base.w_M = w/n_p;
            end
        end
    end
end
