function xdot = pendulum_dynamics(t, x, F, p, dcfg)
% PENDULUM_DYNAMICS
% Nonlinear equations of motion for an inverted pendulum on a cart.
%
% STATE:  x = [xc; xc_dot; theta; theta_dot]
%   xc        — cart position (m), unbounded (ring or long track)
%   xc_dot    — cart velocity (m/s)
%   theta     — pendulum angle from upright (rad): 0=upright, pi=hanging
%   theta_dot — pendulum angular velocity (rad/s)
%
% INPUT:  F — horizontal force on cart (N), positive = rightward
%
% FULL NONLINEAR EOM (derived via Lagrangian mechanics):
%
%   M(theta) * [xc_ddot; theta_ddot] = rhs
%
%   M(theta) = [(mc+m)       m*l*cos(theta)]
%              [m*l*cos(theta)  m*l^2      ]
%
%   rhs = [F - b*xc_dot - m*l*theta_dot^2*sin(theta)]
%         [m*g*l*sin(theta) - bp*theta_dot           ]
%
% SIGN CONVENTION:
%   theta = 0   : upright (unstable equilibrium, controller target)
%   theta = pi  : hanging (stable equilibrium, swing-up start)
%   theta > 0   : pendulum tilted to the right of upright
%   F > 0       : cart pushed right  ->  theta_ddot < 0 (Minv(2,1) < 0)
%
%   Therefore error convention: e = +theta (not -theta).
%   Positive error (theta > 0) requires positive F to correct:
%     F = kp * theta  ->  theta_ddot = Minv(2,1)*kp*theta < 0  (corrective)
%
% SWING-UP ENERGY LAW:
%   E_pend = 0.5*m*l^2*theta_dot^2 + m*g*l*cos(theta)
%   E_target = m*g*l  (energy at upright with theta_dot=0)
%   dE/dt = -m*l*cos(theta)*theta_dot * xc_ddot
%   Control law: F = k*(E - E_target)*cos(theta)*theta_dot
%   gives dV/dt = -k*(ml/(mc+m))*(E-E_target)^2*cos^2(theta)*theta_dot^2 <= 0
%   This is Lyapunov-guaranteed energy injection toward E_target.

xc        = x(1);
xc_dot    = x(2);
theta     = x(3);
theta_dot = x(4);

% ---- Disturbances -------------------------------------------------------
if nargin < 5 || isempty(dcfg)
    d_cart  = 0;    % extra force on cart (N)
    d_pend  = 0;    % extra torque on pendulum (N*m)
    m_eff   = p.m;  % effective bob mass (for parametric disturbance)
else
    [d_cart, d_pend, m_eff] = fp_disturbance(t, x, dcfg, p);
end

% ---- Mass matrix and right-hand side -----------------------------------
sth = sin(theta);
cth = cos(theta);

M = [(p.mc + m_eff),     m_eff * p.l * cth;
      m_eff * p.l * cth,  m_eff * p.l^2   ];

rhs = [(F + d_cart)  - p.b * xc_dot - m_eff * p.l * theta_dot^2 * sth;
        m_eff * p.g * p.l * sth - p.bp * theta_dot + d_pend            ];

% ---- Solve for accelerations -------------------------------------------
acc = M \ rhs;   % [xc_ddot; theta_ddot]

xc_ddot    = acc(1);
theta_ddot = acc(2);

% ---- State derivative --------------------------------------------------
xdot = [xc_dot; xc_ddot; theta_dot; theta_ddot];

end
