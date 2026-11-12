function ctrl_par = ovhz_params(vhz_ctrl, slew_rate, pwm)
%OVHZ_PARAMS Parameters of the blocks of the control system
%   Observer-based VHz control.
%   CTRL_PAR = OVHZ_PARAMS(VHZ_CTRL, SLEW_RATE, PWM) gives the struct CTRL_PAR with
%   a struct of the parameters for each block of the control system, nested as the
%   blocks, and the sampling period T_s, from the arguments of the top-level class
%   of motulator as structs (see the method params of the MATLAB classes of
%   motulator). The MATLAB Function blocks of the control system refer to the
%   variable ctrl_par of the model, e.g., to ctrl_par.PWM.compute_output. An empty
%   parameter ([], None in motulator) is NaN in the structs of the blocks. Generated
%   by motulator_export from the masks of the blocks of the control system.
T_s = vhz_ctrl.cfg.T_s;
ctrl_par = struct();
% Sampling period
ctrl_par.T_s = T_s;
p = struct();
p.T_s = T_s;
p = numeric_fields(p);
ctrl_par.get_measurement = p;
ctrl_par.RateLimiter = RateLimiter(slew_rate, T_s);
ctrl_par.PWM = PWM(T_s, pwm.t_d, pwm.u_th, pwm.i_0, pwm.feedforward, pwm.d_min, ...
    pwm.single_update, pwm.overmodulation);
ctrl_par.ObserverBasedVHzController = ObserverBasedVHzController(vhz_ctrl.par, ...
    vhz_ctrl.cfg);
p = struct();
p.T_s = T_s;
p = numeric_fields(p);
if ~isscalar(p.T_s) || isnan(p.T_s)
    error('modulate: T_s must be a scalar.');
end
ctrl_par.modulate = p;
p = struct();
p.T_s = T_s;
p = numeric_fields(p);
ctrl_par.Monitored_signals = p;
end

function p = RateLimiter(rate_limit, T_s)
%RATELIMITER Parameters of the block RateLimiter (RateLimiter of motulator) from the
%   arguments of its class.
p = struct();
p.rate_limit = rate_limit;
p.T_s = T_s;
p = numeric_fields(p);
if ~isscalar(p.rate_limit) || isnan(p.rate_limit)
    error('RateLimiter: rate_limit must be a scalar.');
end
if ~isscalar(p.T_s) || isnan(p.T_s)
    error('RateLimiter: T_s must be a scalar.');
end
end

function s = PWM(T_s, t_d, u_th, i_0, feedforward, d_min, single_update, ...
    overmodulation)
%PWM Parameters of the block PWM (PWM of motulator) from the arguments of its class.
% MME overmodulation, or MPE
mme = strcmp(overmodulation, 'MME');
s = struct();
p = struct();
p.T_s = T_s;
p.t_d = t_d;
p.u_th = u_th;
p.i_0 = i_0;
p.feedforward = feedforward;
p.d_min = d_min;
p.single_update = single_update;
p.mme = mme;
p = numeric_fields(p);
if ~isscalar(p.T_s) || isnan(p.T_s)
    error('compute_output: T_s must be a scalar.');
end
if ~isscalar(p.t_d) || isnan(p.t_d)
    error('compute_output: t_d must be a scalar.');
end
if ~isscalar(p.u_th) || isnan(p.u_th)
    error('compute_output: u_th must be a scalar.');
end
if ~isscalar(p.i_0) || isnan(p.i_0)
    error('compute_output: i_0 must be a scalar.');
end
if ~isscalar(p.feedforward) || isnan(p.feedforward)
    error('compute_output: feedforward must be a scalar.');
end
if ~isscalar(p.d_min) || isnan(p.d_min)
    error('compute_output: d_min must be a scalar.');
end
if ~isscalar(p.single_update) || isnan(p.single_update)
    error('compute_output: single_update must be a scalar.');
end
if ~isscalar(p.mme) || isnan(p.mme)
    error('compute_output: mme must be a scalar.');
end
s.compute_output = p;
p = struct();
p.T_s = T_s;
p.t_d = t_d;
p.u_th = u_th;
p.i_0 = i_0;
p.feedforward = feedforward;
p.d_min = d_min;
p.single_update = single_update;
p.mme = mme;
p = numeric_fields(p);
if ~isscalar(p.T_s) || isnan(p.T_s)
    error('get_realized_voltage: T_s must be a scalar.');
end
if ~isscalar(p.t_d) || isnan(p.t_d)
    error('get_realized_voltage: t_d must be a scalar.');
end
if ~isscalar(p.u_th) || isnan(p.u_th)
    error('get_realized_voltage: u_th must be a scalar.');
end
if ~isscalar(p.i_0) || isnan(p.i_0)
    error('get_realized_voltage: i_0 must be a scalar.');
end
if ~isscalar(p.feedforward) || isnan(p.feedforward)
    error('get_realized_voltage: feedforward must be a scalar.');
end
if ~isscalar(p.d_min) || isnan(p.d_min)
    error('get_realized_voltage: d_min must be a scalar.');
end
if ~isscalar(p.single_update) || isnan(p.single_update)
    error('get_realized_voltage: single_update must be a scalar.');
end
if ~isscalar(p.mme) || isnan(p.mme)
    error('get_realized_voltage: mme must be a scalar.');
end
s.get_realized_voltage = p;
end

function s = ObserverBasedVHzController(par, cfg)
%OBSERVERBASEDVHZCONTROLLER Parameters of the block ObserverBasedVHzController
%   (ObserverBasedVHzController of motulator) from the arguments of its class.
% Pure open-loop V/Hz control with L_M = inf (ObserverBasedVHzController):
% the unit observer gain with the weight h = 1, which disables the model-based
% correction (create_vhz_observer), the initial flux estimate, and k_u = inf
if isinf(par.L_M)
    k_o = 1;
    h = 1;
    psi_s0 = cfg.psi_s_nom;
    k_u = inf;
else
    k_o = [];
    h = 0;
    psi_s0 = 0;
    k_u = cfg.k_u;
end
s = struct();
s.FluxObserver = ObserverBasedVHzController_FluxObserver(par, k_o, h, psi_s0, ...
    cfg.T_s);
p = struct();
p.alpha_f = cfg.alpha_f;
p.T_s = cfg.T_s;
p = numeric_fields(p);
if ~isscalar(p.alpha_f) || isnan(p.alpha_f)
    error('tau_M_lpf: alpha_f must be a scalar.');
end
if ~isscalar(p.T_s) || isnan(p.T_s)
    error('tau_M_lpf: T_s must be a scalar.');
end
s.tau_M_lpf = p;
s.ReferenceGenerator = ObserverBasedVHzController_ReferenceGenerator(par, ...
    cfg.psi_s_nom, cfg.i_s_max, inf, k_u, cfg.k_b);
p = struct();
p = numeric_fields(p);
s.compute_standby_output = p;
s.FluxTorqueController = ObserverBasedVHzController_FluxTorqueController(par, ...
    cfg.alpha_psi, cfg.alpha_tau);
end

function p = ObserverBasedVHzController_FluxObserver(par, k_o, h, psi_s0, T_s)
%OBSERVERBASEDVHZCONTROLLER_FLUXOBSERVER Parameters of the block FluxObserver
%   (FluxObserver of motulator) from the arguments of its class.
% Fields of the machine model par (InductionMachineInvGammaPars) for the
% C-Script
par_n_p = par.n_p;
par_R_s = par.R_s;
par_R_R = par.R_R;
par_L_sgm = par.L_sgm;
par_L_M = par.L_M;
p = struct();
p.k_o = k_o;
p.h = h;
p.psi_s0 = psi_s0;
p.T_s = T_s;
p.par_n_p = par_n_p;
p.par_R_s = par_R_s;
p.par_R_R = par_R_R;
p.par_L_sgm = par_L_sgm;
p.par_L_M = par_L_M;
p = numeric_fields(p);
if ~isscalar(p.h) || isnan(p.h)
    error('FluxObserver: h must be a scalar.');
end
if ~isscalar(p.psi_s0) || isnan(p.psi_s0)
    error('FluxObserver: psi_s0 must be a scalar.');
end
if ~isscalar(p.T_s) || isnan(p.T_s)
    error('FluxObserver: T_s must be a scalar.');
end
if ~isnan(p.k_o(1)) && (numel(p.k_o) ~= 1 || p.k_o(1) ~= 1)
    error('FluxObserver: Only k_o = [] (the default) or 1 supported.');
end
if ~isscalar(p.par_n_p) || isnan(p.par_n_p)
    error('FluxObserver: par_n_p must be a scalar.');
end
if ~isscalar(p.par_R_s) || isnan(p.par_R_s)
    error('FluxObserver: par_R_s must be a scalar.');
end
if ~isscalar(p.par_R_R) || isnan(p.par_R_R)
    error('FluxObserver: par_R_R must be a scalar.');
end
if ~isscalar(p.par_L_sgm) || isnan(p.par_L_sgm)
    error('FluxObserver: par_L_sgm must be a scalar.');
end
if ~isscalar(p.par_L_M) || isnan(p.par_L_M)
    error('FluxObserver: par_L_M must be a scalar.');
end
end

function p = ObserverBasedVHzController_ReferenceGenerator(par, psi_s_nom, ...
    i_s_max, tau_M_max, k_u, k_b)
%OBSERVERBASEDVHZCONTROLLER_REFERENCEGENERATOR Parameters of the block
%   ReferenceGenerator (ReferenceGenerator of motulator) from the arguments of its
%   class.
% Fields of the machine model par (InductionMachineInvGammaPars) for the
% C-Script
par_n_p = par.n_p;
par_R_s = par.R_s;
par_R_R = par.R_R;
par_L_sgm = par.L_sgm;
par_L_M = par.L_M;
p = struct();
p.psi_s_nom = psi_s_nom;
p.i_s_max = i_s_max;
p.tau_M_max = tau_M_max;
p.k_u = k_u;
p.k_b = k_b;
p.par_n_p = par_n_p;
p.par_R_s = par_R_s;
p.par_R_R = par_R_R;
p.par_L_sgm = par_L_sgm;
p.par_L_M = par_L_M;
p = numeric_fields(p);
if ~isscalar(p.psi_s_nom) || isnan(p.psi_s_nom)
    error('ReferenceGenerator: psi_s_nom must be a scalar.');
end
if ~isscalar(p.i_s_max) || isnan(p.i_s_max)
    error('ReferenceGenerator: i_s_max must be a scalar.');
end
if ~isscalar(p.tau_M_max) || isnan(p.tau_M_max)
    error('ReferenceGenerator: tau_M_max must be a scalar.');
end
if ~isscalar(p.k_u) || isnan(p.k_u)
    error('ReferenceGenerator: k_u must be a scalar.');
end
if ~isscalar(p.k_b) || isnan(p.k_b)
    error('ReferenceGenerator: k_b must be a scalar.');
end
if ~isscalar(p.par_n_p) || isnan(p.par_n_p)
    error('ReferenceGenerator: par_n_p must be a scalar.');
end
if ~isscalar(p.par_R_s) || isnan(p.par_R_s)
    error('ReferenceGenerator: par_R_s must be a scalar.');
end
if ~isscalar(p.par_R_R) || isnan(p.par_R_R)
    error('ReferenceGenerator: par_R_R must be a scalar.');
end
if ~isscalar(p.par_L_sgm) || isnan(p.par_L_sgm)
    error('ReferenceGenerator: par_L_sgm must be a scalar.');
end
if ~isscalar(p.par_L_M) || isnan(p.par_L_M)
    error('ReferenceGenerator: par_L_M must be a scalar.');
end
end

function p = ObserverBasedVHzController_FluxTorqueController(par, alpha_psi, ...
    alpha_tau)
%OBSERVERBASEDVHZCONTROLLER_FLUXTORQUECONTROLLER Parameters of the block
%   FluxTorqueController (FluxTorqueController of motulator) from the arguments of
%   its class.
% Fields of the machine model par (InductionMachineInvGammaPars) for the
% C-Script
par_n_p = par.n_p;
par_R_s = par.R_s;
par_R_R = par.R_R;
par_L_sgm = par.L_sgm;
par_L_M = par.L_M;
p = struct();
p.alpha_psi = alpha_psi;
p.alpha_tau = alpha_tau;
p.par_n_p = par_n_p;
p.par_R_s = par_R_s;
p.par_R_R = par_R_R;
p.par_L_sgm = par_L_sgm;
p.par_L_M = par_L_M;
p = numeric_fields(p);
if ~isscalar(p.alpha_psi) || isnan(p.alpha_psi)
    error('FluxTorqueController: alpha_psi must be a scalar.');
end
if ~isscalar(p.alpha_tau) || isnan(p.alpha_tau)
    error('FluxTorqueController: alpha_tau must be a scalar.');
end
if ~isscalar(p.par_n_p) || isnan(p.par_n_p)
    error('FluxTorqueController: par_n_p must be a scalar.');
end
if ~isscalar(p.par_R_s) || isnan(p.par_R_s)
    error('FluxTorqueController: par_R_s must be a scalar.');
end
if ~isscalar(p.par_R_R) || isnan(p.par_R_R)
    error('FluxTorqueController: par_R_R must be a scalar.');
end
if ~isscalar(p.par_L_sgm) || isnan(p.par_L_sgm)
    error('FluxTorqueController: par_L_sgm must be a scalar.');
end
if ~isscalar(p.par_L_M) || isnan(p.par_L_M)
    error('FluxTorqueController: par_L_M must be a scalar.');
end
end

function p = numeric_fields(p)
%NUMERIC_FIELDS Fields of the struct p as doubles, [] (None) as NaN.
names = fieldnames(p);
for k = 1:numel(names)
    if isempty(p.(names{k}))
        p.(names{k}) = NaN;
    else
        p.(names{k}) = double(p.(names{k}));
    end
end
end
