function [F, info] = ctrl_fuzzy_swingup(x, rule_base, fcfg, p)
% CTRL_FUZZY_SWINGUP
% Fuzzy controller for the swing-up phase (pendulum near hanging position).
%
% Unlike the energy-based and backstepping strategies (which are derived
% from analytical Lyapunov arguments), this controller encodes swing-up
% intuition directly as fuzzy rules operating on the pendulum angle and
% angular velocity. It is the only fully fuzzy swing-up strategy and
% completes the project's focus on fuzzy methods.
%
% The rule base intuition:
%   - If theta ≈ -180° (hanging) and theta_dot ≈ 0: push hard in one direction
%   - If theta is rising toward upright (theta_dot in correct direction): assist
%   - If theta is near upright (|theta| < capture zone): hand off to stabiliser
%   - If pendulum is swinging back (wrong direction): push harder to reverse
%
% The same fp_rule_base tables are used but the error is defined differently:
%   swing_error = theta + pi   (0 = hanging down, pi = upright)
%   NOT the same convention as the stabilisation controllers
%
% Inputs:
%   x          — state [phi; phi_dot; theta; theta_dot]
%   rule_base  — rule base struct (same kp/kd/ki tables, repurposed)
%   fcfg       — fuzzy config
%   p          — pendulum_params
%
% Outputs:
%   F     — control force (N)
%   info  — diagnostic struct

theta     = x(3);
theta_dot = x(4);

% ---- Swing-up error convention -----------------------------------------
% Furuta error convention: e = +theta (see ctrl_fuzzy_pid for explanation)
e_deg    = rad2deg(theta);
edot_deg = rad2deg(theta_dot);

% ---- Fuzzy inference ---------------------------------------------------
[kp_raw, kd_raw, ~, dbg] = fp_inference(e_deg, edot_deg, rule_base, fcfg);

% Swing-up uses only proportional + derivative (no integral term)
% Scale outputs to full force range for aggressive swing-up
kp = max(0, min(p.Fmax / max(180, abs(e_deg) + 1), kp_raw));
kd = max(0, min(p.Fmax / max(180, abs(edot_deg) + 1), kd_raw));

% ---- Swing-up force ----------------------------------------------------
% During swing-up we apply full bang-like forces scaled by fuzzy gains
% The direction sign is critical: we push in the direction of motion
% to build energy, but reverse near the top to catch the pendulum.
E_current = 0.5 * p.m * p.l^2 * theta_dot^2 - p.m * p.g * p.l * cos(theta);
E_target  = p.E_target;   % energy at upright: mgl (potential), zero kinetic
E_error   = E_current - E_target;

% Primary force from fuzzy gains
F_fuzzy = kp * e_deg + kd * edot_deg;

% Energy correction: if below target energy, add energy-proportional boost
% This hybrid approach makes the fuzzy swing-up more reliable than pure
% rule-based since the energy term handles the global swing-up and the
% fuzzy term handles the fine positioning near upright.
F_energy = sign(theta_dot * cos(theta)) * min(p.Fmax * 0.8, abs(E_error) * 0.5);

% Blend based on distance from upright:
% Far from upright -> energy-based dominant
% Near upright -> fuzzy dominant
blend = min(1, abs(theta) / (pi * 0.5));   % 1 when far, 0 when close
F = blend * F_energy + (1 - blend) * F_fuzzy;
F = max(-p.Fmax, min(p.Fmax, F));

info = struct();
info.e_deg     = e_deg;
info.edot_deg  = edot_deg;
info.E_current = E_current;
info.E_target  = E_target;
info.E_error   = E_error;
info.blend     = blend;
info.F_fuzzy   = F_fuzzy;
info.F_energy  = F_energy;
info.firing_w  = dbg.w;
info.label     = 'Fuzzy Swing-Up';

end
