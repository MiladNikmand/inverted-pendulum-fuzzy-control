function p = pendulum_params(preset)
% PENDULUM_PARAMS
% Physical parameters for the inverted pendulum on a motorised cart.
%
% CONFIGURATION:
%   A cart of mass mc slides along a horizontal track (or rotates on a
%   horizontal ring — the dynamics are identical; the ring just prevents
%   the cart hitting the track ends). A pendulum of length l and bob
%   mass m hangs from the cart pivot and swings in the vertical plane.
%
%   State:  x = [xc; xc_dot; theta; theta_dot]
%     xc        = cart position (m) — unbounded on ring
%     xc_dot    = cart velocity (m/s)
%     theta     = pendulum angle from upright (rad), 0 = upright, pi = hanging
%     theta_dot = pendulum angular velocity (rad/s)
%
%   Input: F = horizontal force on cart (N), positive = rightward
%
% EQUATIONS OF MOTION (Lagrangian, full nonlinear):
%   (mc+m)*xc_ddot + m*l*theta_ddot*cos(theta) = F - b*xc_dot
%                                                 - m*l*theta_dot^2*sin(theta)
%   m*l^2*theta_ddot + m*l*xc_ddot*cos(theta)  = m*g*l*sin(theta) - bp*theta_dot
%
% LINEARISED MASS MATRIX at upright (theta=0):
%   M_eq = [(mc+m)   m*l ]    det(M) = mc*m*l^2 + m^2*l^2*(1-1) 
%          [m*l      m*l^2]           = mc*m*l^2
%
%   M^{-1} * [1;0] = [1/(mc+m+m^2*l^2/(mc*m*l^2)); ...] — rank-4 controllable.
%
% SCALE PRESETS:
%   'small'   — lightweight demonstration rig (tabletop)
%   'medium'  — standard lab rig (default)
%   'large'   — heavy-duty industrial rig
%   'custom'  — returns medium defaults, user modifies fields
%
% Usage:
%   p = pendulum_params()              % medium
%   p = pendulum_params('small')

if nargin < 1, preset = 'medium'; end

switch lower(preset)

    case 'small'
        p.mc   = 0.5;     % cart mass (kg)
        p.m    = 0.15;    % bob mass (kg)
        p.l    = 0.30;    % rod length (m)
        p.g    = 9.81;
        p.b    = 0.001;   % cart viscous friction (N*s/m)
        p.bp   = 0.0001;  % pendulum joint friction (N*m*s/rad)
        p.Fmax = 2.0;     % max actuator force (N)
        p.name = 'Small (tabletop demo rig)';

    case 'medium'
        p.mc   = 2.5;
        p.m    = 0.20;
        p.l    = 0.50;
        p.g    = 9.81;
        p.b    = 0.05;
        p.bp   = 0.002;
        p.Fmax = 15.0;
        p.name = 'Medium (standard lab rig)';

    case 'large'
        p.mc   = 5.0;
        p.m    = 0.50;
        p.l    = 0.80;
        p.g    = 9.81;
        p.b    = 0.10;
        p.bp   = 0.005;
        p.Fmax = 30.0;
        p.name = 'Large (industrial rig)';

    case {'custom', 'default'}
        p = pendulum_params('medium');
        p.name = 'Custom';

    otherwise
        warning('Unknown preset "%s", using medium.', preset);
        p = pendulum_params('medium');
end

p.preset = lower(preset);

% ---- Derived quantities -------------------------------------------------
p.omega_n  = sqrt(p.g / p.l);        % natural frequency (rad/s)
p.E_target = p.m * p.g * p.l;        % energy at upright equilibrium (J)

% Linearised mass matrix at upright (theta=0)
p.M11_eq  = p.mc + p.m;              % [kg]
p.M12_eq  = p.m * p.l;               % [kg*m]
p.M22_eq  = p.m * p.l^2;             % [kg*m^2]
detM      = p.M11_eq * p.M22_eq - p.M12_eq^2;
p.Minv_eq = [p.M22_eq, -p.M12_eq; -p.M12_eq, p.M11_eq] / detM;

% Simulation defaults
p.dt           = 0.005;
p.t_end        = 25.0;
p.theta0       = pi;
p.theta_dot0   = 0.05;
p.xc0          = 0.0;
p.xc_dot0      = 0.0;
p.xc_ref       = 0.0;      % desired cart position for regulation (m)
p.theta_ref    = 0.0;      % reference angle from governor (deg), set each step
p.govcfg       = struct(); % governor config, populated by fp_run_analysis

% Capture zone for swing-up -> stabilisation handoff
p.capture_deg  = 25.0;     % |theta| < this AND moving toward upright
p.capture_vel  = 2.0;      % |theta_dot| < this (rad/s)

end
