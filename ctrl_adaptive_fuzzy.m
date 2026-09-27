function [F, rule_base_out, info] = ctrl_adaptive_fuzzy(x, int_e, rule_base, fcfg, gcfg, p, adapt_cfg)
% CTRL_ADAPTIVE_FUZZY
% Adaptive fuzzy controller with online gradient-descent rule update.
% Inner loop of the cascade — regulates theta around theta_ref.
% See ctrl_fuzzy_pid for base architecture description.

theta     = x(3); theta_dot = x(4);
theta_ref = 0; if isfield(p,'theta_ref'), theta_ref = deg2rad(p.theta_ref); end

e_deg    = rad2deg(theta - theta_ref);
edot_deg = rad2deg(theta_dot);

kp_base = 4.16; kd_base = 1.08;
if isfield(p,'govcfg')
    if isfield(p.govcfg,'kp_inner'), kp_base = p.govcfg.kp_inner; end
    if isfield(p.govcfg,'kd_inner'), kd_base = p.govcfg.kd_inner; end
end

[kp_raw, kd_raw, ki_raw, dbg] = fp_inference(e_deg, edot_deg, rule_base, fcfg);
[kp_fuzz, kd_fuzz, ~] = fp_gain_map(kp_raw, kd_raw, ki_raw, gcfg);

ae = abs(e_deg);
if     ae > 20, mult = 4.0;
elseif ae > 10, mult = 2.5;
elseif ae >  5, mult = 1.5;
else,           mult = 1.0;
end

if ae > 5
    kp_eff = kp_fuzz * mult;
    kd_eff = kd_fuzz * mult;
else
    kp_eff = kp_base * mult;
    kd_eff = kd_base * mult;
end

F = max(-p.Fmax, min(p.Fmax, kp_eff*e_deg + kd_eff*edot_deg));

% ---- Adaptive rule update (gradient descent on composite error) --------
rule_base_out = rule_base;
freeze = false; if isfield(adapt_cfg,'freeze'), freeze = adapt_cfg.freeze; end

if ~freeze && isfield(dbg,'fired_rules') && ~isempty(dbg.fired_rules)
    eta_kp = 0.01; eta_kd = 0.005;
    if isfield(adapt_cfg,'eta_kp'), eta_kp = adapt_cfg.eta_kp; end
    if isfield(adapt_cfg,'eta_kd'), eta_kd = adapt_cfg.eta_kd; end

    delta_kp = -eta_kp * e_deg;
    delta_kd = -eta_kd * edot_deg;

    for ri = 1:numel(dbg.fired_rules)
        idx = dbg.fired_rules(ri);
        if idx >= 1 && idx <= numel(rule_base_out.kp)
            kp_bounds = [0 500]; kd_bounds = [0 100];
            if isfield(adapt_cfg,'kp_bounds'), kp_bounds = adapt_cfg.kp_bounds; end
            if isfield(adapt_cfg,'kd_bounds'), kd_bounds = adapt_cfg.kd_bounds; end
            rule_base_out.kp(idx) = max(kp_bounds(1), ...
                min(kp_bounds(2), rule_base_out.kp(idx) + delta_kp));
            rule_base_out.kd(idx) = max(kd_bounds(1), ...
                min(kd_bounds(2), rule_base_out.kd(idx) + delta_kd));
        end
    end
end

info = struct('kp',kp_eff,'kd',kd_eff,'ki',0, ...
              'e_deg',e_deg,'theta_ref',rad2deg(theta_ref), ...
              'mult',mult,'dbg',dbg,'label','AdaptiveFuzzy');
end
