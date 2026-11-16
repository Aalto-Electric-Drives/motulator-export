% Open-loop V/Hz control of induction machine drives, ported from motulator (MATLAB
% port of the classes of ../c/im_open_loop_vhz.c used by the blocks of the control
% systems). The functions are copied into the MATLAB functions of the blocks
% (mfun_*.m) by motulator_export.simulink.
%
% Python counterparts:
%   motulator/drive/control/_im_open_loop_vhz.py  OpenLoopVHzController
%
% The V/Hz curve is affine, u_s_ref(w_s) = u_0 + k*w_s, given by the voltage u_0 at
% zero frequency and the slope k.

function self = im_open_loop_vhz_ctrl_init(n_p, u_0, k, T_s)
%IM_OPEN_LOOP_VHZ_CTRL_INIT Open-loop V/Hz controller (OpenLoopVHzController).
self = struct('n_p', n_p, 'u_0', complex(u_0), 'k', complex(k), 'T_s', T_s, ...
    'theta_c', 0);
end

function fbk = im_open_loop_vhz_ctrl_get_feedback(self, u_s_ab, i_s_ab, w_M_ref)
%IM_OPEN_LOOP_VHZ_CTRL_GET_FEEDBACK Feedback signals (OpenLoopVHzFeedbacks) without
%   u_dc and u_s, the voltage u_s_ab NaN if unknown (blocked gates).
theta_c = self.theta_c;
w_s = self.n_p*w_M_ref;
u_s_zoh = complex(0);
if ~isnan(real(u_s_ab))
    u_s_zoh = exp(-1i*theta_c)*u_s_ab;
end
fbk = struct('i_s', exp(-1i*theta_c)*i_s_ab, 'u_s', complex(0), ...
    'u_s_zoh', u_s_zoh, 'w_M', w_M_ref, 'w_s', w_s, 'w_c', w_s, ...
    'theta_c', theta_c);
end

function u_s_ref = im_open_loop_vhz_ctrl_compute_output(self, fbk)
%IM_OPEN_LOOP_VHZ_CTRL_COMPUTE_OUTPUT Stator voltage reference from the V/Hz curve.
u_s_ref = self.u_0 + self.k*fbk.w_s;
end

function self = im_open_loop_vhz_ctrl_update(self, T_s, fbk)
%IM_OPEN_LOOP_VHZ_CTRL_UPDATE Update the angle of the coordinate system.
self.theta_c = wrap(self.theta_c + T_s*fbk.w_c);
end
