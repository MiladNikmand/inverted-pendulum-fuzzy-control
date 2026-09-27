function [F, info] = ctrl_type2_fuzzy_pid(x, int_e, gcfg, rule_base, fcfg, p)
% CTRL_TYPE2_FUZZY_PID
% Type-2 fuzzy PID — inner loop of the cascade.
% Same architecture as ctrl_fuzzy_pid but uses interval Type-2 MFs
% with Karnik-Mendel type reduction.
% See ctrl_fuzzy_pid for full architecture description.

theta     = x(3); theta_dot = x(4);
theta_ref = 0; if isfield(p,'theta_ref'), theta_ref = deg2rad(p.theta_ref); end

e_deg    = rad2deg(theta - theta_ref);
edot_deg = rad2deg(theta_dot);

kp_base = 4.16; kd_base = 1.08;
if isfield(p,'govcfg')
    if isfield(p.govcfg,'kp_inner'), kp_base = p.govcfg.kp_inner; end
    if isfield(p.govcfg,'kd_inner'), kd_base = p.govcfg.kd_inner; end
end

fcfg_t2 = fcfg; fcfg_t2.type2 = true;
if ~isfield(fcfg_t2,'fou_width'), fcfg_t2.fou_width = 0.20; end

[kp_raw, kd_raw, ki_raw, dbg] = fp_inference(e_deg, edot_deg, rule_base, fcfg_t2);
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

info = struct('kp',kp_eff,'kd',kd_eff,'ki',0, ...
              'e_deg',e_deg,'theta_ref',rad2deg(theta_ref), ...
              'mult',mult,'dbg',dbg,'label','FuzzyPID_Type2 (KM)');
end
