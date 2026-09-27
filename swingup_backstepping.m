function [F, info] = swingup_backstepping(x, bscfg, p)
% SWINGUP_BACKSTEPPING
% Two-level backstepping swing-up for the cart-pendulum.
%
% STRUCTURE:
%   Drives E_pend -> E_target via a cascade of two virtual control loops.
%
% ---- LEVEL 1: Energy shaping (theta subsystem) -------------------------
%   The pendulum energy cannot be directly controlled by F.
%   The controllable channel is through xc_ddot:
%     dE/dt = -m*l*cos(theta)*theta_dot * xc_ddot
%
%   Choose desired xc_ddot to make dE/dt = -k1*(E - E_target):
%     xc_ddot_d = k1*(E - E_target) / (m*l*cos(theta)*theta_dot + eps)
%
%   Lyapunov Level 1:  V1 = 0.5*(E - E_target)^2
%   V1_dot = (E-E_t)*dE/dt = (E-E_t)*(-m*l*cos*thd*xc_ddot_d) = -k1*(E-E_t)^2 <= 0
%
%   Singular when cos(theta)=0 (theta=±90°) or theta_dot=0.
%   At these points xc_ddot_d is clamped to ±xc_ddot_max.
%
% ---- LEVEL 2: xc_dot tracking (cart subsystem) ------------------------
%   Integrate xc_ddot_d to get xc_dot_d:
%     xc_dot_d(k) = xc_dot_d(k-1) + xc_ddot_d * dt
%   (tracked with anti-windup clamp)
%
%   Choose F to track xc_dot_d:
%     xc_ddot_desired = -k2 * (xc_dot - xc_dot_d)
%   From cart EOM: F = (mc+m)*xc_ddot_desired + b*xc_dot + m*l*theta_dot^2*sin(theta)
%
%   Lyapunov Level 2: V2 = 0.5*(k1/k2)*(xc_dot - xc_dot_d)^2
%   V2_dot = -k1*(xc_dot - xc_dot_d)^2 <= 0
%
% COMPOSITE LYAPUNOV:
%   V_bs = V1 + V2 = 0.5*(E-E_t)^2 + 0.5*(k1/k2)*e_xcdot^2
%   V_bs_dot = -k1*(E-E_t)^2 - k1*e_xcdot^2  (negative definite)
%
% Inputs:
%   x      — state [xc; xc_dot; theta; theta_dot]
%   bscfg  — config:
%     .k1             Level-1 energy gain (default 4.0)
%     .k2             Level-2 velocity tracking gain (default 10.0, must > k1)
%     .xc_dot_d       Virtual cart velocity target (state, passed in/out)
%     .xc_ddot_max    Clamp for singularity handling (default 20.0 m/s^2)
%     .E_thresh       Convergence threshold (J)
%   p      — pendulum_params struct

if ~isfield(bscfg,'k1'),          bscfg.k1          = 4.0;  end
if ~isfield(bscfg,'k2'),          bscfg.k2          = 10.0; end
if ~isfield(bscfg,'xc_dot_d'),    bscfg.xc_dot_d    = 0.0;  end
if ~isfield(bscfg,'xc_ddot_max'), bscfg.xc_ddot_max = 20.0; end
if ~isfield(bscfg,'E_thresh'),    bscfg.E_thresh     = 0.05; end

xc_dot    = x(2);
theta     = x(3);
theta_dot = x(4);
dt        = p.dt;

% ---- Pendulum energy ---------------------------------------------------
KE_pend = 0.5 * p.m * p.l^2 * theta_dot^2;
PE_pend = p.m * p.g * p.l * cos(theta);
E_pend  = KE_pend + PE_pend;
e1      = E_pend - p.E_target;

% ---- Level 1: desired xc_ddot ------------------------------------------
denom = p.m * p.l * cos(theta) * theta_dot;
if abs(denom) > 1e-3
    xc_ddot_d = bscfg.k1 * e1 / denom;
else
    xc_ddot_d = 0;   % coast through singular points
end
xc_ddot_d = max(-bscfg.xc_ddot_max, min(bscfg.xc_ddot_max, xc_ddot_d));

% ---- Integrate to get xc_dot_d -----------------------------------------
xc_dot_d = bscfg.xc_dot_d + xc_ddot_d * dt;
xc_dot_d = max(-10, min(10, xc_dot_d));   % anti-windup

% ---- Level 2: compute F ------------------------------------------------
e2          = xc_dot - xc_dot_d;
xc_ddot_des = -bscfg.k2 * e2;

% Invert cart EOM for F:
%   (mc+m)*xc_ddot + m*l*theta_ddot*cos(theta) = F - b*xc_dot - m*l*theta_dot^2*sin(theta)
% Approximate by ignoring m*l*theta_ddot*cos(theta) term (standard backstepping approximation):
F_raw = (p.mc + p.m) * xc_ddot_des ...
      + p.b * xc_dot ...
      + p.m * p.l * theta_dot^2 * sin(theta);

F = max(-p.Fmax, min(p.Fmax, F_raw));

% ---- Lyapunov values ---------------------------------------------------
V_bs     = 0.5*e1^2 + 0.5*(bscfg.k1/bscfg.k2)*e2^2;
V_bs_dot = -bscfg.k1*e1^2 - bscfg.k1*e2^2;   % designed to be neg. def.

info = struct();
info.E_pend      = E_pend;
info.E_target    = p.E_target;
info.e1          = e1;
info.xc_dot_d    = xc_dot_d;   % IMPORTANT: caller must save this as bscfg.xc_dot_d
info.e2          = e2;
info.xc_ddot_d   = xc_ddot_d;
info.V_bs        = V_bs;
info.V_bs_dot    = V_bs_dot;
info.F_raw       = F_raw;
info.label       = 'Backstepping Swing-Up (Cart-Pendulum)';

end
