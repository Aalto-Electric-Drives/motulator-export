function ctrl_par = control_params(info, variables)
%CONTROL_PARAMS Parameters of the blocks of a control system.
%   CTRL = CONTROL_PARAMS(INFO, VARIABLES) gives the struct of the parameters of
%   the MATLAB Function blocks of the control system with the data INFO (see
%   motulator.internal.block_info), nested as the blocks (e.g.,
%   CTRL.SpeedController), from the arguments of its top-level class, the fields of
%   the struct VARIABLES (e.g., vector_ctrl, speed_ctrl, and pwm). It is computed by
%   the function of the parameters of the control system in the folder of the
%   library (e.g., fvc_params.m).
args = cellfun(@(name) variables.(name), cellstr(info.params_args), ...
    'UniformOutput', false);
ctrl_par = feval(info.params, args{:});
end
