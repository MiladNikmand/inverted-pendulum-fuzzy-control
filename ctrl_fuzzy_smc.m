function [F, info] = ctrl_fuzzy_smc(x, smccfg, rule_base, fcfg, p)
% CTRL_FUZZY_SMC
% Fuzzy Sliding Mode Controller — inner loop of the cascade.
%
% SLIDING SURFACE:
%   s = lambda * e + edot
%   where e = theta - theta_ref   (error w.r.t. governor reference)
%         edot = theta_dot
%
% CONTROL LAW:
%   F = F_eq + F_sw
%
%   F_eq — full nonlinear equivalent control.
%   Derived from the full cart-pendulum EOM to cancel all coupling terms
%   including the xc_dot damping and centripetal coupling:
%     theta_ddot = Minv21*(F - b*xc_dot - m*l*thd^2*sin(theta))
%                + Minv22*(m*g*l*sin(theta) - bp*theta_dot)
%   For s_dot = -eta*s:
%     F_eq = (-eta*s - lambda*edot - nl_terms) / Minv21
%
%   F_sw — fuzzy-scheduled switching term.
%   The fuzzy system maps (s, s_dot) to a switching gain K in [K_min, K_max].
%   This replaces the fixed conservative bound and reduces chattering.
%
% NOTE ON STEADY-STATE BEHAVIOUR:
%   Pure SMC without integral action exhibits steady-state theta error when
%   the cart is in sustained motion (xc_dot != 0). The governor cascade
%   handles position regulation; the SMC handles theta tracking only.
%   Residual cart drift is expected without an integral xc term.
%
% GOVERNOR INTEGRATION:
%   theta_ref is read from p.theta_ref (set by fp_run_analysis when
%   the xc governor is active). When governor is inactive: theta_ref = 0.

% ---- Unpack state ------------------------------------------------------
xc_dot    = x(2);
theta     = x(3);
theta_dot = x(4);

theta_ref_rad = 0;
if isfield(p,'theta_ref'), theta_ref_rad = deg2rad(p.theta_ref); end

% Error w.r.t. governor reference
e    = theta - theta_ref_rad;
edot = theta_dot;

% ---- Sliding variable --------------------------------------------------
s    = smccfg.lambda * e + edot;
sdot = smccfg.lambda * edot;   % approximate

% ---- Full nonlinear equivalent control ---------------------------------
% Cancels gravity, damping, and xc_dot coupling terms from full EOM.
Minv21 = p.Minv_eq(2,1);
Minv22 = p.Minv_eq(2,2);
sth    = sin(theta);

nl_term = Minv21 * (-p.b * xc_dot - p.m * p.l * theta_dot^2 * sth) ...
        + Minv22 * (p.m * p.g * p.l * sth - p.bp * theta_dot);

eta    = 2.0;
if isfield(smccfg,'eta'), eta = smccfg.eta; end

F_eq = (-eta * s - smccfg.lambda * edot - nl_term) / Minv21;

% ---- Fuzzy switching gain ----------------------------------------------
s_deg    = rad2deg(s);
sdot_deg = rad2deg(sdot);

[K_raw, ~, ~, dbg] = fp_inference(s_deg, sdot_deg, rule_base, fcfg);

max_raw = max(rule_base.kp_rule(:));
if max_raw < 1e-9, max_raw = 1; end
K_fuzzy = smccfg.K_min + (K_raw / max_raw) * (smccfg.K_max - smccfg.K_min);
K_fuzzy = max(smccfg.K_min, min(smccfg.K_max, K_fuzzy));

% ---- Switching term (saturation in boundary layer) --------------------
phi_b = smccfg.phi_boundary;
if abs(s) <= phi_b
    sat_s = s / phi_b;
else
    sat_s = sign(s);
end

F_sw = -K_fuzzy * sat_s;

% ---- Total force -------------------------------------------------------
F = max(-p.Fmax, min(p.Fmax, F_eq + F_sw));

% ---- Info --------------------------------------------------------------
info = struct();
info.kp       = K_fuzzy;   % repurposed: switching gain as "kp" for metrics
info.kd       = 0;
info.ki       = 0;
info.s        = s;
info.sdot     = sdot;
info.K_fuzzy  = K_fuzzy;
info.F_eq     = F_eq;
info.F_sw     = F_sw;
info.sat_s    = sat_s;
info.theta_ref= rad2deg(theta_ref_rad);
info.label    = 'FuzzySMC';

end
