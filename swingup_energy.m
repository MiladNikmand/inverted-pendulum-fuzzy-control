function [F, info] = swingup_energy(x, sucfg, p)
% SWINGUP_ENERGY
% Energy-based swing-up for the cart-pendulum.
%
% THEORY:
%   Pendulum mechanical energy:
%     E_pend = 0.5*m*l^2*theta_dot^2 + m*g*l*cos(theta)
%
%   At upright (theta=0, theta_dot=0): E_target = m*g*l (maximum PE)
%   At hanging (theta=pi, theta_dot=0): E_hanging = -m*g*l (minimum PE)
%   Energy to inject: 2*m*g*l
%
% LYAPUNOV FUNCTION:
%   V_su = 0.5*(E_pend - E_target)^2
%
%   Time derivative:
%     dV/dt = (E_pend - E_target) * dE/dt
%
%   From the cart-pendulum EOM, the controllable part of dE/dt is:
%     dE/dt = -m*l*cos(theta)*theta_dot * xc_ddot
%           ≈ -m*l*cos(theta)*theta_dot * F/(mc+m)   [leading order]
%
%   Control law that makes dV/dt <= 0:
%     F = k_su * (E_pend - E_target) * cos(theta) * theta_dot
%
%   This gives:
%     dV/dt = -k_su*(ml/(mc+m))*(E_pend-E_target)^2*cos^2(theta)*theta_dot^2 <= 0
%
%   LYAPUNOV GUARANTEED — V_su decreases monotonically (except at singular
%   points theta=pi/2 where cos=0, or theta_dot=0 where pendulum is momentarily
%   at rest; the pendulum escapes these by gravity and resumes converging).
%
% KEY DIFFERENCE FROM SIGN-BASED LAWS:
%   This law uses (E-E_target)*cos(theta)*theta_dot directly — no sign()
%   function. This avoids chattering (sign flipping every timestep when
%   theta_dot passes through zero near the hanging position).
%
%   The proportional form also naturally scales down the force as the
%   pendulum approaches E_target, giving a smooth handoff to the
%   stabilising controller.
%
% Inputs:
%   x      — state [xc; xc_dot; theta; theta_dot]
%   sucfg  — config: .k_su (gain, default 10.0), .E_thresh (convergence threshold)
%   p      — pendulum_params struct
%
% Outputs:
%   F     — cart force (N), saturated to ±p.Fmax
%   info  — struct with energy tracking values for fp_lyapunov

if ~isfield(sucfg,'k_su'),     sucfg.k_su     = 10.0; end
if ~isfield(sucfg,'E_thresh'), sucfg.E_thresh = 0.05; end

theta     = x(3);
theta_dot = x(4);

% ---- Pendulum mechanical energy ----------------------------------------
KE_pend = 0.5 * p.m * p.l^2 * theta_dot^2;
PE_pend = p.m * p.g * p.l * cos(theta);
E_pend  = KE_pend + PE_pend;

E_target = p.E_target;
E_error  = E_pend - E_target;    % negative while below target

% ---- Control law -------------------------------------------------------
% F = k_su * (E_pend - E_target) * cos(theta) * theta_dot
% Lyapunov-guaranteed: dV/dt <= 0 everywhere (see theory above)
F_raw = sucfg.k_su * E_error * cos(theta) * theta_dot;

% Saturate to actuator limit
F = max(-p.Fmax, min(p.Fmax, F_raw));

% ---- Lyapunov values ---------------------------------------------------
V_su     = 0.5 * E_error^2;
% Approximate V_su_dot (ignoring dissipation):
% dV/dt = E_error * dE/dt = E_error * (-m*l*cos(theta)*theta_dot * xc_ddot)
% After control: xc_ddot ~ F/(mc+m)
xc_ddot_approx = F / (p.mc + p.m);
V_su_dot = E_error * (-p.m * p.l * cos(theta) * theta_dot * xc_ddot_approx);

info = struct();
info.E_pend    = E_pend;
info.KE_pend   = KE_pend;
info.PE_pend   = PE_pend;
info.E_target  = E_target;
info.E_error   = E_error;
info.V_su      = V_su;
info.V_su_dot  = V_su_dot;
info.F_raw     = F_raw;
info.label     = 'Energy-Based Swing-Up (Cart-Pendulum)';

end
