function [cost, metrics_out] = opt_cost(params, ctrl_name, base_cfg, cost_weights)
% OPT_COST
% Cost function for pendulum gain optimisation.
%
% Runs one simulation for a single controller with the given parameter
% vector, then returns a scalar cost combining settling time, overshoot,
% cart drift, and control effort.
%
% USAGE:
%   cost = opt_cost(params, 'FuzzyPID_Sugeno', base_cfg, weights)
%   cost = opt_cost(params, 'LQR',             base_cfg, weights)
%
% PARAMETER VECTORS (per controller type) — all length 6
%
%   FuzzyPID_Sugeno / FuzzyPID_Mamdani / FuzzyPID_Type2:
%     [kp_max, kd_max, inner_scale, kp_gov, kd_gov, max_tilt]
%       kp_max/kd_max  gain-map bounds, drive the large-error regime
%       inner_scale    multiplies the LQR-derived near-equilibrium gains
%       kp_gov/kd_gov  governor gains (deg/m, deg·s/m)
%       max_tilt       governor tilt limit (deg)
%
%   AdaptiveFuzzy:
%     [kp_max, kd_max, eta_kp, eta_kd, kp_gov, kd_gov]
%
%   FuzzySMC:
%     [lambda, phi_boundary, K_min, K_max, kp_gov, kd_gov]
%
%   LQR:
%     [Q1, Q2, Q3, Q4, qi, R]
%       governor gains are re-derived from these weights each evaluation
%
%   ClassicalPID:
%     [kp, kd, ki, kp_gov, kd_gov, max_tilt]
%
% COST FUNCTION:
%   cost = w1 * t_settle_norm
%        + w2 * overshoot_norm
%        + w3 * xc_rms_norm
%        + w4 * effort_norm
%        + penalty  (large if failed to capture or settle)
%
%   Normalised values are divided by reference values so each term is O(1):
%     t_settle_norm  = t_settle  / 5.0    (5s is a reasonable reference)
%     overshoot_norm = overshoot / 30.0   (30 deg reference)
%     xc_rms_norm    = xc_rms   / 0.5    (0.5m reference)
%     effort_norm    = effort   / 100.0   (100 N*s reference)
%
% INPUTS:
%   params       — parameter vector (see above)
%   ctrl_name    — controller name string
%   base_cfg     — full config struct from run_all (used as template;
%                  only the relevant gain fields are overridden)
%   cost_weights — [w1, w2, w3, w4] default [2, 1, 0.5, 0.3]
%
% OUTPUTS:
%   cost        — scalar cost (lower = better)
%   metrics_out — struct with raw metrics for logging

% ---- Default weights ---------------------------------------------------
if nargin < 4 || isempty(cost_weights)
    cost_weights = [2.0, 1.0, 0.5, 0.3];
end
w1=cost_weights(1); w2=cost_weights(2);
w3=cost_weights(3); w4=cost_weights(4);

% ---- Build config from base, override with params ----------------------
cfg = base_cfg;
cfg.controllers  = {ctrl_name};
cfg.quiet        = true;      % suppress all fprintf output
cfg.run_dir      = tempname(); % throw-away folder (no files written)
mkdir(cfg.run_dir);

% Unpack params into the right config struct
cfg = unpack_params(params, ctrl_name, cfg);

% ---- Run simulation ----------------------------------------------------
cost    = 1e6;   % default: very high (will be returned on error)
metrics_out = struct('t_settle',NaN,'overshoot_deg',NaN, ...
                     'xc_rms',NaN,'total_effort',NaN, ...
                     'captured',false,'cost',cost);
try
    results = fp_run_analysis(cfg);
catch ME
    % Simulation crashed (usually numerical instability at bad gains)
    cleanup(cfg.run_dir);
    return;
end

% ---- Extract metrics ---------------------------------------------------
if ~isfield(results.runs, ctrl_name)
    cleanup(cfg.run_dir);
    return;
end

r = results.runs.(ctrl_name);
m = r.metrics;

% ---- Penalty for failure -----------------------------------------------
PENALTY_NO_CAPTURE = 1e4;
PENALTY_NO_SETTLE  = 500;

if ~r.captured
    cost = PENALTY_NO_CAPTURE;
    metrics_out.captured = false;
    cleanup(cfg.run_dir);
    return;
end

% ---- Normalised cost terms --------------------------------------------
t_settle   = m.t_settle;
overshoot  = m.overshoot_deg;
effort     = m.total_effort;
xc_err     = m.xc_final_error;   % cart position error at end of run

% Hard constraint on xc_final_error:
% Controllers that fail to bring cart within 0.2m of xc_ref are penalised
% heavily regardless of theta performance. This enforces position regulation
% as a primary objective alongside balancing.
XC_ERR_THRESH  = 0.20;   % 20 cm tolerance
PENALTY_XC     = 300;    % applied per metre of excess error

% Normalise each term (reference values: 5s, 30deg, 0.5m, 100N*s)
t_settle_norm  = min(t_settle,  cfg.t_end) / 5.0;
overshoot_norm = min(overshoot, 180.0)     / 30.0;
xc_rms_norm    = min(xc_err,    5.0)       / 0.5;
effort_norm    = min(effort,    1e4)       / 100.0;

% No-settle penalty
if isnan(m.t_settle)
    no_settle_pen = PENALTY_NO_SETTLE;
else
    no_settle_pen = 0;
end

% xc position penalty (hard constraint beyond threshold)
if xc_err > XC_ERR_THRESH
    xc_pen = PENALTY_XC * (xc_err - XC_ERR_THRESH);
else
    xc_pen = 0;
end

cost = w1*t_settle_norm + w2*overshoot_norm ...
     + w3*xc_rms_norm   + w4*effort_norm  ...
     + no_settle_pen + xc_pen;

% ---- Fill output struct ------------------------------------------------
metrics_out.t_settle      = t_settle;
metrics_out.overshoot_deg = overshoot;
metrics_out.xc_rms        = xc_err;
metrics_out.xc_final_error= xc_err;
metrics_out.xc_settled    = m.xc_settled;
metrics_out.total_effort  = effort;
metrics_out.captured      = r.captured;
metrics_out.cost          = cost;

cleanup(cfg.run_dir);
end


% =========================================================================
%  UNPACK PARAMS
% =========================================================================
function cfg = unpack_params(params, ctrl_name, cfg)
% Maps the flat parameter vector onto the config structs the cascade
% architecture actually reads.
%
% For every non-LQR controller the search space is split between the
% INNER loop (how hard the controller fights theta error) and the
% GOVERNOR (how hard it pushes the cart toward xc_ref). Those are the
% two knobs that trade balancing quality against position accuracy,
% which is exactly the trade-off the Pareto front is meant to expose.
%
% Note: govcfg fields are only honoured by fp_run_analysis when present,
% otherwise it derives them from the augmented LQR K5 solution. Setting
% them here overrides that derivation for this evaluation only.

if ~isfield(cfg,'govcfg'), cfg.govcfg = struct(); end

switch ctrl_name

    case {'FuzzyPID_Sugeno','FuzzyPID_Mamdani','FuzzyPID_Type2'}
        % [kp_max, kd_max, inner_scale, kp_gov, kd_gov, max_tilt]
        cfg.gcfg.kp_max       = max(5,    params(1));
        cfg.gcfg.kd_max       = max(0.5,  params(2));
        cfg.govcfg.inner_scale= max(0.1,  params(3));
        cfg.govcfg.kp_gov     = max(0.05, params(4));
        cfg.govcfg.kd_gov     = max(0.05, params(5));
        cfg.govcfg.max_tilt   = max(1.0,  params(6));

    case 'AdaptiveFuzzy'
        % [kp_max, kd_max, eta_kp, eta_kd, kp_gov, kd_gov]
        cfg.gcfg.kp_max       = max(5,    params(1));
        cfg.gcfg.kd_max       = max(0.5,  params(2));
        cfg.adapt_cfg.eta_kp  = max(0,    params(3));
        cfg.adapt_cfg.eta_kd  = max(0,    params(4));
        cfg.govcfg.kp_gov     = max(0.05, params(5));
        cfg.govcfg.kd_gov     = max(0.05, params(6));

    case 'FuzzySMC'
        % [lambda, phi_boundary, K_min, K_max, kp_gov, kd_gov]
        cfg.smccfg.lambda       = max(0.1,  params(1));
        cfg.smccfg.phi_boundary = max(0.005,params(2));
        cfg.smccfg.K_min        = max(0,    params(3));
        cfg.smccfg.K_max        = max(params(3)+0.1, params(4));
        cfg.govcfg.kp_gov       = max(0.05, params(5));
        cfg.govcfg.kd_gov       = max(0.05, params(6));

    case 'LQR'
        % [Q1, Q2, Q3, Q4, qi, R]
        % The governor gains are DERIVED from this same Q5/R solution, so
        % tuning the weights moves the inner loop and the governor together
        % and keeps the controller internally consistent.
        cfg.lqr_Q  = [max(1e-3,params(1)), max(1e-3,params(2)), ...
                      max(1e-3,params(3)), max(1e-3,params(4))];
        cfg.lqr_qi = max(1e-4, params(5));
        cfg.lqr_R  = max(1e-4, params(6));
        cfg.govcfg = struct();   % force re-derivation from the new weights

    case 'ClassicalPID'
        % [kp, kd, ki, kp_gov, kd_gov, max_tilt]
        cfg.pidcfg.kp       = max(0.1,  params(1));
        cfg.pidcfg.kd       = max(0.01, params(2));
        cfg.pidcfg.ki       = max(0,    params(3));
        cfg.govcfg.kp_gov   = max(0.05, params(4));
        cfg.govcfg.kd_gov   = max(0.05, params(5));
        cfg.govcfg.max_tilt = max(1.0,  params(6));

    otherwise
        error('opt_cost: unknown controller "%s"', ctrl_name);
end
end


% =========================================================================
%  CLEANUP
% =========================================================================
function cleanup(run_dir)
% Remove the temporary run folder (no useful output from optimiser runs)
try
    if exist(run_dir,'dir')
        rmdir(run_dir,'s');
    end
catch
    % Non-fatal if cleanup fails
end
end
