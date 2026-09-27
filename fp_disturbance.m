function [d_cart, d_pend, m_eff] = fp_disturbance(t, x, dcfg, p)
% FP_DISTURBANCE
% Generates disturbance forces/torques for the cart-pendulum simulation.
%
% Five disturbance types, each independently configurable:
%
%   1. Impulse kick      — sudden horizontal force on the cart
%      Physical meaning: someone bumps the rig or a step input is applied
%      Channel: d_cart (N, directly added to F in cart EOM)
%
%   2. Wind gust         — lateral aerodynamic drag on the pendulum rod
%      Physical meaning: gust of wind hitting the swinging rod/bob
%      Model: von Karman ramp-up / hold / ramp-down profile
%      Channel: d_pend (N*m, torque on pendulum joint)
%
%   3. Track friction    — Coulomb + viscous friction spike on cart motion
%      Physical meaning: debris on track, worn bearing
%      Channel: d_cart (N, opposes xc_dot)
%
%   4. Parametric mass   — bob mass suddenly increases (payload added)
%      Physical meaning: object dropped onto/attached to pendulum bob
%      Effect: changes m_eff returned to pendulum_dynamics
%
%   5. Track inclination — track tilted from horizontal
%      Physical meaning: rig placed on an uneven surface
%      Effect: sinusoidal gravity bias on cart at track inclination
%
% Inputs:
%   t     — current time (s)
%   x     — state [xc; xc_dot; theta; theta_dot]
%   dcfg  — disturbance config struct (from run_all / pendulum_launch)
%   p     — pendulum_params struct
%
% Outputs:
%   d_cart  — extra horizontal force on cart (N)
%   d_pend  — extra torque on pendulum joint (N*m)
%   m_eff   — effective bob mass (= p.m unless parametric mass active)

d_cart = 0;
d_pend = 0;
m_eff  = p.m;

if nargin < 3 || isempty(dcfg), return; end

xc_dot = x(2);
theta  = x(3);

% =========================================================================
%  1. IMPULSE KICK — horizontal force on cart
% =========================================================================
if isfield(dcfg,'kick_enabled') && dcfg.kick_enabled
    sigma  = max(dcfg.kick_duration, 0.03);
    % Gaussian impulse: peak force = kick_mag (N), width = sigma (s)
    d_cart = d_cart + dcfg.kick_mag * exp(-0.5*((t - dcfg.kick_time)/sigma)^2);
end

% =========================================================================
%  2. WIND GUST — torque on pendulum joint
% =========================================================================
if isfield(dcfg,'wind_enabled') && dcfg.wind_enabled
    rho_air = 1.225;        % air density (kg/m^3)
    Cd_rod  = 1.2;          % drag coefficient (cylindrical rod)
    A_rod   = 0.5*p.l*0.02; % projected area: length * diameter (m^2)
    z_com   = 0.5*p.l;      % centre of pressure height (m)

    t1 = dcfg.wind_start;
    t2 = t1 + dcfg.wind_ramp;
    t3 = t2 + dcfg.wind_hold;
    t4 = t3 + dcfg.wind_ramp;

    if t >= t1 && t <= t4
        if     t <= t2, frac = (t-t1) / max(dcfg.wind_ramp, 0.01);
        elseif t <= t3, frac = 1.0;
        else,           frac = 1.0 - (t-t3) / max(dcfg.wind_ramp, 0.01);
        end
        v_wind  = dcfg.wind_speed * frac;
        F_wind  = 0.5 * rho_air * Cd_rod * A_rod * v_wind^2;
        % Torque on pendulum: force at centre of pressure * moment arm
        d_pend  = d_pend + F_wind * z_com * cos(theta);
    end
end

% =========================================================================
%  3. TRACK FRICTION — opposes cart velocity
% =========================================================================
if isfield(dcfg,'friction_enabled') && dcfg.friction_enabled
    if t >= dcfg.friction_start && t <= dcfg.friction_end
        F_coulomb = dcfg.friction_mag  * sign(xc_dot + 1e-8);
        F_viscous = dcfg.friction_visc * xc_dot;
        d_cart = d_cart - (F_coulomb + F_viscous);
    end
end

% =========================================================================
%  4. PARAMETRIC MASS CHANGE
% =========================================================================
if isfield(dcfg,'mass_enabled') && dcfg.mass_enabled
    if t >= dcfg.mass_time
        m_eff = p.m + dcfg.mass_delta;
        m_eff = max(0.01, m_eff);
    end
end

% =========================================================================
%  5. TRACK INCLINATION — gravity bias on cart
% =========================================================================
if isfield(dcfg,'tilt_enabled') && dcfg.tilt_enabled
    % Tilted track: component of gravity along the cart direction
    % For a track tilted at angle alpha, the gravity component is:
    % F_tilt = (mc + m_eff) * g * sin(alpha) * sin(tilt_phase + small oscillation)
    % Modelled as a sinusoidal bias that varies with cart position
    alpha  = deg2rad(dcfg.tilt_angle_deg);
    xc     = x(1);
    % Phase encodes cart position on the ring (2*pi per ring circumference)
    R_disp = 0.35;   % display ring radius — affects spatial period only
    d_cart = d_cart - (p.mc + m_eff) * p.g * sin(alpha) * ...
                       sin(xc / R_disp + dcfg.tilt_phase);
end

end