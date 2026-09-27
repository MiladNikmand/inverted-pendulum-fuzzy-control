function [F, int_e_new, info] = ctrl_classical_pid(x, int_e, pidcfg, p)
% CTRL_CLASSICAL_PID
% Fixed-gain PID controller — non-fuzzy baseline for comparison.
%
% Uses the same cascade architecture as the fuzzy controllers:
% the error signal is (theta - theta_ref) where theta_ref is set by
% the xc governor (ctrl_xc_governor) once theta is stable.
%
% Comparing against fuzzy PID shows what adaptive gain scheduling adds:
% the fuzzy system uses large gains far from upright and small gains near
% upright, while this controller uses the same gains everywhere.
%
% AUTO-TUNE (pidcfg.auto_tune = true):
%   Estimates gains from linearised dynamics using Ziegler-Nichols method.
%   Ultimate gain Ku derived from |Minv_eq(2,1)| and omega_n.
%
% Inputs:
%   x       — state [xc; xc_dot; theta; theta_dot]
%   int_e   — running integral of error (deg*s), maintained by fp_run_analysis
%   pidcfg  — PID config:
%     .kp, .kd, .ki   gains (N/deg, N*s/deg, N/(deg*s))
%     .auto_tune       logical — override with Z-N estimates
%     .i_clamp         anti-windup clamp (deg*s), default 500
%   p       — pendulum_params struct (p.theta_ref set by governor)

if ~isfield(pidcfg,'i_clamp'), pidcfg.i_clamp = 500; end

% ---- Auto-tune (Ziegler-Nichols) ----------------------------------------
if isfield(pidcfg,'auto_tune') && pidcfg.auto_tune
    omega_n = p.omega_n;
    Tu      = 2*pi / omega_n;
    B_theta = abs(p.Minv_eq(2,1));
    Ku      = omega_n^2 / max(B_theta, 1e-6);
    pidcfg.kp = 0.6   * Ku;
    pidcfg.ki = pidcfg.kp / (0.5  * Tu);
    pidcfg.kd = pidcfg.kp *  0.125 * Tu;
end

% ---- Governor reference -----------------------------------------------
theta_ref_rad = 0;
if isfield(p,'theta_ref'), theta_ref_rad = deg2rad(p.theta_ref); end

theta     = x(3);
theta_dot = x(4);

% Error w.r.t. governor reference
e_deg    = rad2deg(theta - theta_ref_rad);
edot_deg = rad2deg(theta_dot);

% ---- Anti-windup -------------------------------------------------------
% Three protections against integrator wind-up and tail-chasing:
%   1. Hard clamp on accumulated integral
%   2. Freeze when saturated (standard anti-windup)
%   3. Conditional: only integrate when |e| < 10deg (near equilibrium)
int_e_clamped = max(-pidcfg.i_clamp, min(pidcfg.i_clamp, int_e));

% ---- PID control law ---------------------------------------------------
F = pidcfg.kp * e_deg  +  pidcfg.kd * edot_deg  +  pidcfg.ki * int_e_clamped;
F = max(-p.Fmax, min(p.Fmax, F));

% ---- Update integral (freeze at saturation, conditional near zero) -----
if abs(F) < p.Fmax && abs(e_deg) < 10.0
    int_e_new = int_e + e_deg;
else
    int_e_new = int_e;   % freeze: don't wind up during large errors
end

info = struct();
info.kp       = pidcfg.kp;
info.kd       = pidcfg.kd;
info.ki       = pidcfg.ki;
info.e_deg    = e_deg;
info.edot_deg = edot_deg;
info.int_e    = int_e_clamped;
info.theta_ref= rad2deg(theta_ref_rad);
info.label    = 'Classical PID (fixed gains, cascade)';

end
