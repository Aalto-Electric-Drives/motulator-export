% Gradient networks (GradNets) for magnetics modeling, ported from motulator (MATLAB
% port of ../c/gradnet.c). The functions are embedded in the MATLAB Function blocks
% by motulator_export.simulink.
%
% Python counterpart:
%   motulator/drive/gradnet/_gn.py  GradNet, FluxMap, CurrentMap,
%                                   CurrentMapWithHarmonics
%
% Only the inference is ported (the networks are trained in motulator). The
% computations use double precision, while PyTorch uses single precision. A GradNet
% has a single module, and the input dimension is 2 for the flux and current maps
% and 4 for the current map with spatial harmonics. The linear term is applied to
% the first mu_dim inputs. The activation is 1 (PNormGradient) or 2 (Softmax). The
% computations are vectorized (unlike in the C port, whose results they equal within
% the rounding errors), the sizes being fixed by the weights.

function net = gradnet(in_dim, mu_dim, W, b, mu_log, bias, activation, beta_log, ...
    p, in_base, out_base)
%GRADNET GradNet from its parameters (the fields of the GradNet structs of the
%   model initialization), W being an embed_dim-by-in_dim matrix. The field mu has
%   the gains of the linear term, zero for the inputs beyond the first mu_dim.
m = size(W, 2);
mu = zeros(m, 1);
for k = 1:min(mu_dim, numel(mu_log))
    mu(k) = exp(mu_log(k));
end
net = struct('in_dim', in_dim, 'mu_dim', mu_dim, 'embed_dim', size(W, 1), ...
    'W', W, 'b', b(:), 'mu', mu, 'bias', bias(:), 'activation', activation, ...
    'beta', exp(beta_log), 'p', p, 'in_base', in_base, 'out_base', out_base);
end

function net = gradnet_placeholder()
%GRADNET_PLACEHOLDER GradNet with in_dim = 2 for the machine models without a flux
%   map, so that the code using a flux map compiles.
net = gradnet(2, 0, zeros(1, 2), 0, zeros(2, 1), zeros(2, 1), 2, 0, 1, 1, 1);
end

function [z, jac] = gradnet_forward(net, x, want_jac)
%GRADNET_FORWARD GradNet.forward for a per-unit input x (a column vector).
%   If want_jac is true, the Jacobian matrix (GradNet.jacobian) is computed too. The
%   Jacobians of both activations are of the form c*(diag(d) - u*u^T), so the
%   Jacobian of the network is diag(mu) + c*(W^T diag(d) W - v*v^T) with v = W^T u.
W = net.W;
beta = net.beta;
% Module: W^T act(W x + b)
h = beta*(W*x + net.b);
if net.activation == 1
    % PNormGradient: x^q/(1 + sum(x^(q + 1)))^(q/(q + 1)) with q = p - 1
    q = net.p - 1;
    norm_ = 1 + sum(h.^(q + 1));
    norm_r = norm_^(q/(q + 1));
    c = beta*q/norm_r;
    d = h.^(q - 1);
    u = h.^q/sqrt(norm_);
    h = h.^q/norm_r;
else
    % Softmax
    h = exp(h - max(h));
    h = h/sum(h);
    c = beta;
    d = h;
    u = h;
end
% Linear term (for the first mu_dim inputs), bias, and the module
z = net.mu.*x + net.bias + W'*h;
jac = zeros(size(W, 2));
if want_jac
    v = W'*u;
    jac = diag(net.mu) + c*(W'*(d.*W) - v*v');
end
end

function y = gradnet_map(net, x)
%GRADNET_MAP Evaluate the map (FluxMap or CurrentMap in motulator), symmetrized
%   about the d-axis. The input and output are in SI units.
% Evaluate at the input and its conjugate, and symmetrize about the d-axis
x1 = [real(x); imag(x)]/net.in_base;
x2 = [x1(1); -x1(2)];
z1 = gradnet_forward(net, x1, false);
z2 = gradnet_forward(net, x2, false);
y = 0.5*((z1(1) + 1i*z1(2)) + conj(z2(1) + 1i*z2(2)));
y = y*net.out_base;
end

function jac = gradnet_map_jacobian(net, x)
%GRADNET_MAP_JACOBIAN Jacobian matrix [dy_d/dx_d, dy_d/dx_q; dy_q/dx_d, dy_q/dx_q]
%   of the map (CurrentMap.jacobian in motulator) in SI units.
% Symmetrize as in gradnet_map: the conjugation flips the signs of the off-diagonal
% elements
x1 = [real(x); imag(x)]/net.in_base;
x2 = [x1(1); -x1(2)];
[~, jac1] = gradnet_forward(net, x1, true);
[~, jac2] = gradnet_forward(net, x2, true);
jac = 0.5*(net.out_base/net.in_base)*(jac1 + [1, -1; -1, 1].*jac2);
end

function [i_s, tau_m] = gradnet_current_map_harmonics(net, k, psi_s, theta_m)
%GRADNET_CURRENT_MAP_HARMONICS Evaluate the current map with spatial harmonics
%   (CurrentMapWithHarmonics in motulator) of order k. Returns the stator current
%   (A) in rotor coordinates and the electromagnetic torque (Nm) per pole pair.
% Evaluate at the input and its conjugate, and symmetrize about the d-axis
psi = psi_s/net.in_base;
exp_j_k_theta = exp(1i*k*theta_m);
x1 = [real(psi); imag(psi); real(exp_j_k_theta); imag(exp_j_k_theta)];
x2 = [x1(1); -x1(2); x1(3); -x1(4)];
z1 = gradnet_forward(net, x1, false);
z2 = gradnet_forward(net, x2, false);
i = 0.5*((z1(1) + 1i*z1(2)) + conj(z2(1) + 1i*z2(2)));
dW_dcos = 0.5*(z1(3) + z2(3));
dW_dsin = 0.5*(z1(4) - z2(4));
% Torque in per-unit values, including the reluctance torque due to the spatial
% harmonics
dW_dtheta = k*(real(exp_j_k_theta)*dW_dsin - imag(exp_j_k_theta)*dW_dcos);
tau = imag(i*conj(psi)) - dW_dtheta;
% Scale back to physical units
i_s = i*net.out_base;
tau_m = tau*1.5*net.in_base*net.out_base;
end
