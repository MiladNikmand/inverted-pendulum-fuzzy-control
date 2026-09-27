function [F, info] = ctrl_lqr(x, K_lqr, p)
% CTRL_LQR
% Full-state LQR controller with integral xc action.
%
% STATE (augmented, 5 elements):
%   xa = [xc - xc_ref; xc_dot; theta; theta_dot; xi]
%   xi = integral of (xc - xc_ref)  — eliminates steady-state position error
%
% CONTROL LAW:
%   F = -K5 * xa
%     = -K(1)*(xc-xc_ref) - K(2)*xc_dot - K(3)*theta
%       - K(4)*theta_dot  - K(5)*xi
%
% The integral state xi is maintained externally in fp_run_analysis
% and passed in via the p struct as p.lqr_xi (updated each step).
%
% xc_ref is read from p.xc_ref (set in pendulum_params / run_all).
%
% K_lqr may be a 1x4 vector (legacy, no integral) or 1x5 (augmented).
% If 1x4, falls back to simple reference tracking without integral.

xc_ref = 0;
if isfield(p,'xc_ref'), xc_ref = p.xc_ref; end

xi = 0;
if isfield(p,'lqr_xi'), xi = p.lqr_xi; end

if numel(K_lqr) == 5
    % Augmented 5-state LQR with integral
    xa_err = [x(1)-xc_ref; x(2); x(3); x(4); xi];
    F_raw  = -K_lqr * xa_err;
else
    % Legacy 4-state LQR — reference tracking without integral
    x_err = [x(1)-xc_ref; x(2); x(3); x(4)];
    F_raw = -K_lqr * x_err;
    xa_err = x_err;
end

F = max(-p.Fmax, min(p.Fmax, F_raw));

info = struct();
info.K_lqr  = K_lqr;
info.xa_err = xa_err;
info.xi     = xi;
info.xc_ref = xc_ref;
info.F_raw  = F_raw;
info.label  = 'LQR (augmented 5-state, integral xc)';

end
