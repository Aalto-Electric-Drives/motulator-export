% Current-vector control of synchronous machine drives, ported from motulator
% (MATLAB port of the classes of ../c/sm_current_vector.c used by the blocks of the
% control system). The functions are copied into the MATLAB functions of the blocks
% (mfun_*.m) by motulator_export.simulink.
%
% Python counterpart:
%   motulator/drive/control/_sm_current_vector.py  CurrentController
%
% The observer and the reference generator are those of flux-vector control
% (sm_flux_vector.m).

function self = sm_current_ctrl_init(par, alpha_c, alpha_i)
%SM_CURRENT_CTRL_INIT Current controller (CurrentController): a complex-vector PI
%   controller of the flux linkages; alpha_i = NaN means alpha_c.
if isnan(alpha_i)
    alpha_i = alpha_c;
end
k_t = alpha_c;
k_i = alpha_c*alpha_i;
k_p = alpha_c + alpha_i;
self = struct('par', par, 'pi', complex_pi_init(k_p, k_i, k_t));
end

function [self, u] = sm_current_ctrl_compute_output(self, i_ref, i)
%SM_CURRENT_CTRL_COMPUTE_OUTPUT Voltage reference, the currents mapped to the flux
%   linkages, which takes the saliency and the magnetic saturation into account.
psi_ref = psi_s_dq(self.par, i_ref) - self.par.psi_f;
psi = psi_s_dq(self.par, i) - self.par.psi_f;
[self.pi, u] = complex_pi_compute_output(self.pi, psi_ref, psi, 0);
end

function self = sm_current_ctrl_update(self, T_s, u, w_c)
%SM_CURRENT_CTRL_UPDATE Update the integral state with the realized voltage u.
self.pi = complex_pi_update(self.pi, T_s, u, w_c);
end

function self = sm_current_ctrl_preset(self, u)
%SM_CURRENT_CTRL_PRESET Preset the integral state to the steady-state output u.
self.pi.u_i = u;
end
