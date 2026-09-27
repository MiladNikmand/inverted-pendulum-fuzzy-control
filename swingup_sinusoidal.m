function [F, info] = swingup_sinusoidal(x, sucfg, p)
% SWINGUP_SINUSOIDAL
% Bang-bang sinusoidal swing-up controller (original project baseline).
%
% The simplest swing-up strategy: apply a periodic square-wave force
% at a frequency close to the natural frequency of the pendulum,
% gradually building oscillation amplitude until the capture zone is reached.
%
% This is the baseline comparison for the energy-based and backstepping
% strategies. It does not use any state feedback — it is pure feedforward.
% Its key weaknesses are:
%   1. Fixed frequency: if the pendulum length changes (parametric disturbance),
%      resonance is lost and swing-up fails.
%   2. No convergence guarantee: unlike the Lyapunov-based energy method,
%      there is no proof that this reaches the capture zone.
%   3. Sensitive to initial conditions and friction.
%
% Despite these weaknesses it is included because:
%   a) It was in the original project and is a legitimate engineering approach
%   b) It shows what a non-Lyapunov strategy looks like in comparison
%   c) It succeeds reliably under nominal conditions
%
% Inputs:
%   x      — state [phi; phi_dot; theta; theta_dot]  (used for phase-aware variant)
%   sucfg  — config:
%     .omega     forcing frequency (rad/s), default = 0.8 * omega_n
%     .F_amp     force amplitude (N), default = 0.9 * p.Fmax
%     .phase_aware  logical — use theta sign to time the pushes (default true)
%   p      — pendulum_params
%
% Outputs:
%   F     — control force (N)
%   info  — diagnostic struct

if ~isfield(sucfg,'omega'),       sucfg.omega       = 0.8 * p.omega_n; end
if ~isfield(sucfg,'F_amp'),       sucfg.F_amp       = 0.85 * p.Fmax;  end
if ~isfield(sucfg,'phase_aware'), sucfg.phase_aware = true;            end

theta     = x(3);
theta_dot = x(4);

if sucfg.phase_aware
    % Phase-aware version: push in direction that adds energy
    % If pendulum is swinging toward upright (theta*theta_dot < 0 for upright at 0):
    %   push in the direction of theta_dot (add kinetic energy)
    % else:
    %   reverse to not fight the swing
    F = sucfg.F_amp * sign(theta_dot * cos(theta));
else
    % Pure time-based bang-bang at natural frequency
    % Caller must pass current time in sucfg.t
    t = sucfg.t;
    F = sucfg.F_amp * sign(sin(sucfg.omega * t));
end

F = max(-p.Fmax, min(p.Fmax, F));

% Energy for comparison with other methods
KE = 0.5 * p.m * p.l^2 * theta_dot^2;
PE = p.m * p.g * p.l * cos(theta);
E  = KE + PE;

info = struct();
info.E        = E;
info.E_target = p.E_target;
info.E_error  = E - p.E_target;
info.label    = 'Sinusoidal Bang-Bang Swing-Up';

end
