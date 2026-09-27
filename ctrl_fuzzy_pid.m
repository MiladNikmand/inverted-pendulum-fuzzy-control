function [F, info] = ctrl_fuzzy_pid(x, int_e, gcfg, rule_base, fcfg, p)
% CTRL_FUZZY_PID
% Fuzzy PID stabilising controller — inner loop of the cascade.
%
% ROLE:
%   Regulates theta around theta_ref (set by ctrl_xc_governor outer loop).
%   When the governor is inactive (theta_ref=0), this is pure balancing.
%   When the governor is active, theta_ref is nonzero and the controller
%   tilts the pendulum to drive the cart toward xc_ref.
%
% ERROR SIGNAL:
%   e = theta - theta_ref   [positive = pendulum right of reference]
%   e_dot = theta_dot       [unchanged]
%
% GAIN SCHEDULING:
%   Uses LQR-matched base gains (from p.govcfg) near equilibrium.
%   Boosts gains for large errors (fuzzy advantage during capture transient).
%   |e| > 20 deg: mult=4.0   |e| > 10: mult=2.5
%   |e| > 5 deg:  mult=1.5   |e| <= 5: mult=1.0  (LQR-matched)
%
% The fuzzy inference also runs and its output is used for large-error
% regions where the rule base provides nonlinear gain scheduling.

% ---- Unpack state ------------------------------------------------------
theta      = x(3);
theta_dot  = x(4);

% Theta reference from governor (0 when governor inactive)
theta_ref = 0;
if isfield(p,'theta_ref'), theta_ref = deg2rad(p.theta_ref); end

% Error w.r.t. reference (degrees for fuzzy inference)
e_deg    = rad2deg(theta - theta_ref);
edot_deg = rad2deg(theta_dot);

% ---- Base gains from governor config (K5-derived) ----------------------
kp_base = 4.16;   kd_base = 1.08;   % medium preset fallback
if isfield(p,'govcfg')
    if isfield(p.govcfg,'kp_inner'), kp_base = p.govcfg.kp_inner; end
    if isfield(p.govcfg,'kd_inner'), kd_base = p.govcfg.kd_inner; end
end

% ---- Fuzzy inference (for large-error nonlinear scheduling) -----------
[kp_raw, kd_raw, ki_raw, dbg] = fp_inference(e_deg, edot_deg, rule_base, fcfg);
[kp_fuzz, kd_fuzz, ~] = fp_gain_map(kp_raw, kd_raw, ki_raw, gcfg);

% ---- Gain scheduling multiplier ----------------------------------------
ae = abs(e_deg);
if     ae > 20, mult = 4.0;
elseif ae > 10, mult = 2.5;
elseif ae >  5, mult = 1.5;
else,           mult = 1.0;
end

% Large errors: use fuzzy-scheduled gains (rule base expertise)
% Near equilibrium: use LQR-matched base gains (stable, well-calibrated)
if ae > 5
    kp_eff = kp_fuzz * mult;
    kd_eff = kd_fuzz * mult;
else
    kp_eff = kp_base * mult;
    kd_eff = kd_base * mult;
end

% ---- PD control law ----------------------------------------------------
F = kp_eff * e_deg + kd_eff * edot_deg;
F = max(-p.Fmax, min(p.Fmax, F));

% ---- Info struct -------------------------------------------------------
info = struct();
info.kp        = kp_eff;
info.kd        = kd_eff;
info.ki        = 0;
info.e_deg     = e_deg;
info.theta_ref = rad2deg(theta_ref);
info.mult      = mult;
info.dbg       = dbg;
info.label     = sprintf('FuzzyPID (%s)', fcfg.inference);

end
