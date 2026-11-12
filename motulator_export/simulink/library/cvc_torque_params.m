function ctrl_par = cvc_torque_params(vector_ctrl, pwm)
%CVC_TORQUE_PARAMS Parameters of the blocks of the control system
%   Current-vector control (torque).
%   CTRL_PAR = CVC_TORQUE_PARAMS(VECTOR_CTRL, PWM) gives the struct CTRL_PAR with a
%   struct of the parameters for each block of the control system, nested as the
%   blocks, and the sampling period T_s, from the arguments of the top-level class
%   of motulator as structs (see the method params of the MATLAB classes of
%   motulator). The MATLAB Function blocks of the control system refer to the
%   variable ctrl_par of the model, e.g., to ctrl_par.PWM.compute_output. An empty
%   parameter ([], None in motulator) is NaN in the structs of the blocks. Generated
%   by motulator_export from the masks of the blocks of the control system.
T_s = vector_ctrl.cfg.T_s;
sensorless = vector_ctrl.cfg.sensorless;
% Angle-advance factor (k_comp of the vector controller), zero with the
% direct discrete-time current controller
k_comp = 1.5;
if isfield(vector_ctrl.cfg, 'discrete') && vector_ctrl.cfg.discrete
    k_comp = 0;
end
% Initial speed estimate of the observer, zero if not given (e.g., in a
% model written before the initial state of the observer was exported)
if ~isfield(vector_ctrl, 'observer') || ~isfield(vector_ctrl.observer, 'w_M')
    vector_ctrl.observer.w_M = 0;
end
ctrl_par = struct();
% Sampling period
ctrl_par.T_s = T_s;
p = struct();
p.T_s = T_s;
p = numeric_fields(p);
ctrl_par.get_measurement = p;
p = struct();
p.sensorless = sensorless;
p.T_s = T_s;
p = numeric_fields(p);
if ~isscalar(p.sensorless) || isnan(p.sensorless)
    error('standby: sensorless must be a scalar.');
end
if ~isscalar(p.T_s) || isnan(p.T_s)
    error('standby: T_s must be a scalar.');
end
ctrl_par.standby = p;
p = struct();
p.T_s = T_s;
p = numeric_fields(p);
if ~isscalar(p.T_s) || isnan(p.T_s)
    error('torque_reference: T_s must be a scalar.');
end
ctrl_par.torque_reference = p;
ctrl_par.PWM = PWM(T_s, pwm.t_d, pwm.u_th, pwm.i_0, pwm.feedforward, pwm.d_min, ...
    pwm.single_update, pwm.overmodulation);
ctrl_par.CurrentVectorController = CurrentVectorController(vector_ctrl.par, ...
    vector_ctrl.cfg, vector_ctrl.observer);
p = struct();
p.T_s = T_s;
p.sensorless = sensorless;
p.k_comp = k_comp;
p = numeric_fields(p);
if ~isscalar(p.T_s) || isnan(p.T_s)
    error('modulate: T_s must be a scalar.');
end
if ~isscalar(p.sensorless) || isnan(p.sensorless)
    error('modulate: sensorless must be a scalar.');
end
if ~isscalar(p.k_comp) || isnan(p.k_comp)
    error('modulate: k_comp must be a scalar.');
end
ctrl_par.modulate = p;
p = struct();
p.T_s = T_s;
p = numeric_fields(p);
ctrl_par.Monitored_signals = p;
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

function s = CurrentVectorController(par, cfg, observer)
%CURRENTVECTORCONTROLLER Parameters of the block CurrentVectorController
%   (CurrentVectorController of motulator) from the arguments of its class.
% Default of alpha_i (CurrentController.__init__)
alpha_i = cfg.alpha_i;
if isempty(alpha_i)
    alpha_i = cfg.alpha_c;
end
s = struct();
s.SpeedFluxObserver = CurrentVectorController_SpeedFluxObserver(par, ...
    cfg.alpha_o, cfg.sensorless, cfg.J, observer.w_M, cfg.T_s);
s.CurrentReferenceGenerator = ...
    CurrentVectorController_CurrentReferenceGenerator(par, cfg.psi_s_nom, ...
    cfg.i_s_max, cfg.w_s_nom, cfg.k_u, cfg.k_fw, cfg.T_s);
s.CurrentController = CurrentVectorController_CurrentController(par, ...
    cfg.alpha_c, alpha_i, cfg.discrete, cfg.T_s);
end

function s = CurrentVectorController_SpeedFluxObserver(par, alpha_o, sensorless, ...
    J, w_M, T_s)
%CURRENTVECTORCONTROLLER_SPEEDFLUXOBSERVER Parameters of the block SpeedFluxObserver
%   (SpeedFluxObserver of motulator) from the arguments of its class.
% Default speed estimation poles of create_speed_flux_observer. With J, the
% double pole at alpha_o/2 keeps the speed gain about the same.
alpha_o_obs = alpha_o;
if isempty(alpha_o_obs)
    if sensorless
        alpha_o_obs = 2*pi*60;
    else
        alpha_o_obs = 2*pi*400;
    end
    if ~isempty(J)
        alpha_o_obs = 0.5*alpha_o_obs;
    end
end
% Gains for critically damped dynamics (SpeedFluxObserver.__init__), the
% pole matched to the sampling period (matched_pole)
alpha_o_obs = -expm1(-alpha_o_obs*T_s)/T_s;
if isempty(J)
    k_w = alpha_o_obs;
    k_tau = 0;
else
    k_w = 2*alpha_o_obs;
    k_tau = J*alpha_o_obs^2;
end
% Default observer gain of create_speed_flux_observer, and the weight of the
% measured rotor speed in the FluxObserver
k_o = sensorless;
h = 1 - sensorless;
s = struct();
s.FluxObserver = CurrentVectorController_SpeedFluxObserver_FluxObserver(par, ...
    k_o, h, T_s);
s.SpeedObserver = CurrentVectorController_SpeedFluxObserver_SpeedObserver(k_w, ...
    k_tau, J, w_M, T_s);
end

function p = CurrentVectorController_SpeedFluxObserver_FluxObserver(par, k_o, h, ...
    T_s)
%CURRENTVECTORCONTROLLER_SPEEDFLUXOBSERVER_FLUXOBSERVER Parameters of the block
%   FluxObserver (FluxObserver of motulator) from the arguments of its class.
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
p.T_s = T_s;
p.par_n_p = par_n_p;
p.par_R_s = par_R_s;
p.par_R_R = par_R_R;
p.par_L_sgm = par_L_sgm;
p.par_L_M = par_L_M;
p = numeric_fields(p);
if ~isscalar(p.k_o) || isnan(p.k_o)
    error('FluxObserver: k_o must be a scalar.');
end
if ~isscalar(p.h) || isnan(p.h)
    error('FluxObserver: h must be a scalar.');
end
if ~isscalar(p.T_s) || isnan(p.T_s)
    error('FluxObserver: T_s must be a scalar.');
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

function p = CurrentVectorController_SpeedFluxObserver_SpeedObserver(k_w, k_tau, ...
    J, w_M, T_s)
%CURRENTVECTORCONTROLLER_SPEEDFLUXOBSERVER_SPEEDOBSERVER Parameters of the block
%   SpeedObserver (SpeedObserver of motulator) from the arguments of its class.
p = struct();
p.k_w = k_w;
p.k_tau = k_tau;
p.J = J;
p.w_M = w_M;
p.T_s = T_s;
p = numeric_fields(p);
if ~isscalar(p.k_w) || isnan(p.k_w)
    error('SpeedObserver: k_w must be a scalar.');
end
if ~isscalar(p.k_tau) || isnan(p.k_tau)
    error('SpeedObserver: k_tau must be a scalar.');
end
if ~isscalar(p.w_M) || isnan(p.w_M)
    error('SpeedObserver: w_M must be a scalar.');
end
if ~isscalar(p.T_s) || isnan(p.T_s)
    error('SpeedObserver: T_s must be a scalar.');
end
end

function p = CurrentVectorController_CurrentReferenceGenerator(par, psi_s_nom, ...
    i_s_max, w_s_nom, k_u, k_fw, T_s)
%CURRENTVECTORCONTROLLER_CURRENTREFERENCEGENERATOR Parameters of the block
%   CurrentReferenceGenerator (CurrentReferenceGenerator of motulator) from the
%   arguments of its class.
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
p.w_s_nom = w_s_nom;
p.k_u = k_u;
p.k_fw = k_fw;
p.T_s = T_s;
p.par_n_p = par_n_p;
p.par_R_s = par_R_s;
p.par_R_R = par_R_R;
p.par_L_sgm = par_L_sgm;
p.par_L_M = par_L_M;
p = numeric_fields(p);
if ~isscalar(p.psi_s_nom) || isnan(p.psi_s_nom)
    error('CurrentReferenceGenerator: psi_s_nom must be a scalar.');
end
if ~isscalar(p.i_s_max) || isnan(p.i_s_max)
    error('CurrentReferenceGenerator: i_s_max must be a scalar.');
end
if ~isscalar(p.w_s_nom) || isnan(p.w_s_nom)
    error('CurrentReferenceGenerator: w_s_nom must be a scalar.');
end
if ~isscalar(p.k_u) || isnan(p.k_u)
    error('CurrentReferenceGenerator: k_u must be a scalar.');
end
if ~isscalar(p.T_s) || isnan(p.T_s)
    error('CurrentReferenceGenerator: T_s must be a scalar.');
end
if ~isscalar(p.par_n_p) || isnan(p.par_n_p)
    error('CurrentReferenceGenerator: par_n_p must be a scalar.');
end
if ~isscalar(p.par_R_s) || isnan(p.par_R_s)
    error('CurrentReferenceGenerator: par_R_s must be a scalar.');
end
if ~isscalar(p.par_R_R) || isnan(p.par_R_R)
    error('CurrentReferenceGenerator: par_R_R must be a scalar.');
end
if ~isscalar(p.par_L_sgm) || isnan(p.par_L_sgm)
    error('CurrentReferenceGenerator: par_L_sgm must be a scalar.');
end
if ~isscalar(p.par_L_M) || isnan(p.par_L_M)
    error('CurrentReferenceGenerator: par_L_M must be a scalar.');
end
end

function p = CurrentVectorController_CurrentController(par, alpha_c, alpha_i, ...
    discrete, T_s)
%CURRENTVECTORCONTROLLER_CURRENTCONTROLLER Parameters of the block CurrentController
%   (CurrentController of motulator) from the arguments of its class.
% Fields of the machine model par (InductionMachineInvGammaPars) for the
% C-Script
par_n_p = par.n_p;
par_R_s = par.R_s;
par_R_R = par.R_R;
par_L_sgm = par.L_sgm;
par_L_M = par.L_M;
p = struct();
p.alpha_c = alpha_c;
p.alpha_i = alpha_i;
p.discrete = discrete;
p.T_s = T_s;
p.par_n_p = par_n_p;
p.par_R_s = par_R_s;
p.par_R_R = par_R_R;
p.par_L_sgm = par_L_sgm;
p.par_L_M = par_L_M;
p = numeric_fields(p);
if ~isscalar(p.alpha_c) || isnan(p.alpha_c)
    error('CurrentController: alpha_c must be a scalar.');
end
if ~isscalar(p.alpha_i) || isnan(p.alpha_i)
    error('CurrentController: alpha_i must be a scalar.');
end
if ~isscalar(p.discrete) || isnan(p.discrete)
    error('CurrentController: discrete must be a scalar.');
end
if ~isscalar(p.T_s) || isnan(p.T_s)
    error('CurrentController: T_s must be a scalar.');
end
if ~isscalar(p.par_n_p) || isnan(p.par_n_p)
    error('CurrentController: par_n_p must be a scalar.');
end
if ~isscalar(p.par_R_s) || isnan(p.par_R_s)
    error('CurrentController: par_R_s must be a scalar.');
end
if ~isscalar(p.par_R_R) || isnan(p.par_R_R)
    error('CurrentController: par_R_R must be a scalar.');
end
if ~isscalar(p.par_L_sgm) || isnan(p.par_L_sgm)
    error('CurrentController: par_L_sgm must be a scalar.');
end
if ~isscalar(p.par_L_M) || isnan(p.par_L_M)
    error('CurrentController: par_L_M must be a scalar.');
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
