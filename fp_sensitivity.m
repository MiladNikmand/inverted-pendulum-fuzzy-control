function results = fp_sensitivity(p_base, fcfg_base, rule_base_base, cfg_base, opts)
% FP_SENSITIVITY
% Parameter and rule-base sensitivity analysis.
%
% Runs the simulation multiple times, varying one parameter at a time,
% and records how key performance metrics change. Produces data for
% three sensitivity figures:
%
%   1. MF sensitivity  — vary n_sets (5/7/9) and mf_type (tri/gauss/trap)
%      Metric: settling time, overshoot, total control effort
%
%   2. Rule base sensitivity — vary rule table preset or individual rules
%      Metric: settling time, rms_theta, total_effort per preset
%
%   3. Physical parameter sensitivity — vary m, l, R, mc one at a time
%      Metric: settling time vs parameter value (like bike param_sensitivity)
%
% Inputs:
%   p_base        — nominal pendulum_params struct
%   fcfg_base     — nominal fuzzy config
%   rule_base_base — nominal rule base
%   cfg_base      — simulation config (t_end, dt, x0, etc.)
%   opts          — sensitivity options:
%     .run_mf_sweep        logical (default true)
%     .run_rule_sweep      logical (default true)
%     .run_param_sweep     logical (default true)
%     .param_names         cell of param names to sweep (default all 4)
%     .param_ranges        cell of [min,max] for each param
%     .n_param_pts         points per parameter (default 8)
%     .verbose             logical (default false)

if nargin < 5, opts = struct(); end
if ~isfield(opts,'run_mf_sweep'),    opts.run_mf_sweep    = true; end
if ~isfield(opts,'run_rule_sweep'),  opts.run_rule_sweep  = true; end
if ~isfield(opts,'run_param_sweep'), opts.run_param_sweep = true; end
if ~isfield(opts,'n_param_pts'),     opts.n_param_pts     = 8;    end
if ~isfield(opts,'verbose'),         opts.verbose         = false; end

% Default parameter ranges (±40% around nominal)
if ~isfield(opts,'param_names')
    opts.param_names  = {'m',   'l',   'R',   'mc'};
    opts.param_ranges = {
        [p_base.m*0.5,  p_base.m*2.0];
        [p_base.l*0.5,  p_base.l*2.0];
        [p_base.R*0.5,  p_base.R*2.0];
        [p_base.mc*0.5, p_base.mc*2.0]
    };
end

results = struct();

% =========================================================================
%  1. MF TYPE AND COUNT SENSITIVITY
% =========================================================================
if opts.run_mf_sweep
    fprintf('[fp_sensitivity] MF sweep...\n');
    mf_types  = {'triangular','gaussian','trapezoidal'};
    n_sets_v  = [5, 7, 9];
    mf_results = struct();

    for ti = 1:numel(mf_types)
        for ni = 1:numel(n_sets_v)
            fcfg_test           = fcfg_base;
            fcfg_test.mf_type   = mf_types{ti};
            fcfg_test.n_sets    = n_sets_v(ni);
            rb_test             = fp_rule_base('expert', n_sets_v(ni));

            key = sprintf('%s_N%d', mf_types{ti}, n_sets_v(ni));
            try
                [~, metrics_i] = fp_run_single(p_base, fcfg_test, rb_test, cfg_base);
                mf_results.(key) = metrics_i;
                if opts.verbose
                    fprintf('  %s: settle=%.2f overshoot=%.1f effort=%.1f\n', ...
                            key, metrics_i.t_settle, metrics_i.overshoot_deg, ...
                            metrics_i.total_effort);
                end
            catch ME
                warning('fp_sensitivity MF sweep failed for %s: %s', key, ME.message);
                mf_results.(key) = struct('t_settle',NaN,'overshoot_deg',NaN,'total_effort',NaN);
            end
        end
    end
    results.mf_sweep = mf_results;
    results.mf_types = mf_types;
    results.n_sets_v = n_sets_v;
end

% =========================================================================
%  2. RULE BASE PRESET SENSITIVITY
% =========================================================================
if opts.run_rule_sweep
    fprintf('[fp_sensitivity] Rule base sweep...\n');
    presets = {'expert','aggressive','conservative','symmetric'};
    rule_results = struct();

    for pi_idx = 1:numel(presets)
        pr = presets{pi_idx};
        rb_test = fp_rule_base(pr, fcfg_base.n_sets);
        try
            [~, metrics_i] = fp_run_single(p_base, fcfg_base, rb_test, cfg_base);
            rule_results.(pr) = metrics_i;
            if opts.verbose
                fprintf('  %s: settle=%.2f rms=%.2f effort=%.1f\n', ...
                        pr, metrics_i.t_settle, metrics_i.rms_theta_deg, ...
                        metrics_i.total_effort);
            end
        catch ME
            warning('fp_sensitivity rule sweep failed for %s: %s', pr, ME.message);
            rule_results.(pr) = struct('t_settle',NaN,'rms_theta_deg',NaN,'total_effort',NaN);
        end
    end
    results.rule_sweep   = rule_results;
    results.rule_presets = presets;
end

% =========================================================================
%  3. PHYSICAL PARAMETER SENSITIVITY
% =========================================================================
if opts.run_param_sweep
    fprintf('[fp_sensitivity] Parameter sweep...\n');
    n_params = numel(opts.param_names);
    param_results = struct();

    for pi_idx = 1:n_params
        pname = opts.param_names{pi_idx};
        rng_i = opts.param_ranges{pi_idx};
        vals  = linspace(rng_i(1), rng_i(2), opts.n_param_pts);
        settle_v  = nan(size(vals));
        overshoot_v = nan(size(vals));
        effort_v  = nan(size(vals));

        for vi = 1:numel(vals)
            p_test = p_base;
            p_test.(pname) = vals(vi);
            % Recompute derived quantities
            p_test.omega_n  = sqrt(p_test.g / p_test.l);
            p_test.E_target = p_test.m * p_test.g * p_test.l;
            % Recompute mass matrix derived fields for Furuta
            p_test.M11_eq   = (p_test.mc + p_test.m) * p_test.R^2;
            p_test.M12_eq   = p_test.m * p_test.R * p_test.l;
            p_test.M22_eq   = p_test.m * p_test.l^2;
            detM            = p_test.M11_eq*p_test.M22_eq - p_test.M12_eq^2;
            p_test.Minv_eq  = [p_test.M22_eq, -p_test.M12_eq; ...
                               -p_test.M12_eq, p_test.M11_eq] / detM;

            try
                [~, metrics_i] = fp_run_single(p_test, fcfg_base, rule_base_base, cfg_base);
                settle_v(vi)    = metrics_i.t_settle;
                overshoot_v(vi) = metrics_i.overshoot_deg;
                effort_v(vi)    = metrics_i.total_effort;
            catch
                % leave as NaN
            end
        end

        param_results.(pname) = struct( ...
            'values',     vals, ...
            'nominal',    p_base.(pname), ...
            'settle',     settle_v, ...
            'overshoot',  overshoot_v, ...
            'effort',     effort_v);

        if opts.verbose
            fprintf('  %s: settle range [%.2f, %.2f] s\n', pname, ...
                    min(settle_v), max(settle_v));
        end
    end
    results.param_sweep  = param_results;
    results.param_names  = opts.param_names;
end

fprintf('[fp_sensitivity] Done.\n\n');
end

% =========================================================================
%  LOCAL: run a single short simulation for sensitivity testing
% =========================================================================
function [t_out, metrics] = fp_run_single(p, fcfg, rb, cfg)
% Minimal simulation used only by fp_sensitivity (no GUI, no checkpoint).
% Uses energy swing-up + fuzzy PID stabilisation.

dt     = cfg.dt;
t_end  = min(cfg.t_end, 20);   % cap at 20s for sensitivity runs
t_out  = 0:dt:t_end;
N      = numel(t_out);
x      = zeros(N, 4);

% Initial conditions
x(1,:) = [0, 0, cfg.x0(3), cfg.x0(4)];

% Default configs for controllers
gcfg_def = struct('kp_max',300,'kd_max',50,'ki_max',0.1, ...
                  'kp_min',0,'kd_min',0,'ki_min',0,'use_lpf',false);
sucfg_def = struct('k_su', 3.0, 'E_thresh', 0.05);
cap_rad   = deg2rad(p.capture_deg);
captured  = false;
int_e     = 0;
F_hist    = zeros(N,1);

for k = 1:N-1
    xk = x(k,:)';
    theta = xk(3);

    if ~captured && abs(theta) < cap_rad && abs(xk(4)) < p.capture_vel
        captured = true;
    end

    if ~captured
        [F, ~] = swingup_energy(xk, sucfg_def, p);
    else
        int_e = int_e + deg2rad(-theta) * dt;
        [F, ~] = ctrl_fuzzy_pid(xk, int_e, gcfg_def, rb, fcfg, p);
    end

    F_hist(k) = F;
    xdot = pendulum_dynamics(t_out(k), xk, F, p, []);
    x(k+1,:) = (xk + dt*xdot)';
    if abs(x(k+1,3)) > pi*1.05
        x(k+1:end,:) = NaN;
        break;
    end
end

metrics = fp_metrics(t_out(:), x, F_hist, p, cfg);
end
