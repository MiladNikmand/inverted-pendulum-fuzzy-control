function [rb_suggested, report] = fp_rule_designer(p, fcfg, gcfg, sucfg, opts)
% FP_RULE_DESIGNER
% Offline data-driven rule base suggestion tool.
%
% Sweeps the (error, error_dot) operating space and finds the control gain
% at each point that minimises a local cost function. The resulting gain
% surface is then mapped back to the N x N rule consequent tables, giving
% a data-derived alternative to the hand-designed expert rules.
%
% METHOD:
% For each (e_i, edot_j) point in a grid:
%   1. Initialise the pendulum near the corresponding state
%      (theta = -e_i in rad, theta_dot = -edot_j in rad/s)
%   2. Run a short simulation (cfg.horizon steps) with a PD controller
%      that uses gains (kp_test, kd_test)
%   3. Measure the cost: J = integral(theta^2 + 0.01*F^2) over horizon
%   4. Record the (kp, kd) pair that minimised cost at this operating point
%
% The sweep runs on a coarse grid and interpolates to the N x N table.
% This is the "observe what works" approach to rule base design.
%
% The result is presented to the user as a heatmap in the GUI (Tab 2)
% where they can accept, modify, or reject each cell.
%
% Inputs:
%   p      — pendulum_params
%   fcfg   — fuzzy config (to know n_sets and range)
%   gcfg   — gain map config (for bounds)
%   sucfg  — (unused here, for API consistency)
%   opts   — designer options:
%     .grid_n      grid points per axis for the sweep (default 9)
%     .horizon     steps per test simulation (default 200)
%     .dt_test     integration step for test runs (default 0.01)
%     .kp_range    [min max] kp to try (default [0 400])
%     .kd_range    [min max] kd to try (default [0 60])
%     .n_kp        number of kp values to try per grid point (default 8)
%     .n_kd        number of kd values to try per grid point (default 8)
%     .verbose     logical (default false)
%
% Outputs:
%   rb_suggested — rule base struct in fp_rule_base format
%   report       — struct with cost maps, suggested surfaces, comparison info

if nargin < 5, opts = struct(); end
if ~isfield(opts,'grid_n'),   opts.grid_n   = 9;    end
if ~isfield(opts,'horizon'),  opts.horizon  = 200;  end
if ~isfield(opts,'dt_test'),  opts.dt_test  = 0.01; end
if ~isfield(opts,'kp_range'), opts.kp_range = [0 400]; end
if ~isfield(opts,'kd_range'), opts.kd_range = [0 60];  end
if ~isfield(opts,'n_kp'),     opts.n_kp     = 8;    end
if ~isfield(opts,'n_kd'),     opts.n_kd     = 8;    end
if ~isfield(opts,'verbose'),  opts.verbose  = false; end

n_grid = opts.grid_n;

% Grid of (error, error_dot) values in degree space
e_grid    = linspace(fcfg.range(1), fcfg.range(2), n_grid);
edot_grid = linspace(fcfg.range(1), fcfg.range(2), n_grid);

kp_test   = linspace(opts.kp_range(1), opts.kp_range(2), opts.n_kp);
kd_test   = linspace(opts.kd_range(1), opts.kd_range(2), opts.n_kd);

% Storage
kp_best_grid = zeros(n_grid, n_grid);
kd_best_grid = zeros(n_grid, n_grid);
cost_min_grid = inf(n_grid, n_grid);

fprintf('[fp_rule_designer] Sweeping %dx%d operating space (%d test runs)...\n', ...
        n_grid, n_grid, n_grid^2 * opts.n_kp * opts.n_kd);
tic;

for i = 1:n_grid
    for j = 1:n_grid
        e_ij    = e_grid(i);
        edot_ij = edot_grid(j);

        % Initial state corresponding to this (error, error_dot) point
        theta0     = deg2rad(-e_ij);
        theta_dot0 = deg2rad(-edot_ij);

        % Skip very large initial states that immediately fall
        if abs(theta0) > deg2rad(170), continue; end

        % Sweep kp x kd grid for this operating point
        for kpi = 1:opts.n_kp
            for kdi = 1:opts.n_kd
                kp_i = kp_test(kpi);
                kd_i = kd_test(kdi);

                % Short simulation with this PD gain
                x_k  = [0; 0; theta0; theta_dot0];
                cost = 0;
                for step = 1:opts.horizon
                    th_k  = x_k(3);
                    thd_k = x_k(4);
                    e_k   = rad2deg(-th_k);
                    ed_k  = rad2deg(-thd_k);
                    F_k   = kp_i*e_k + kd_i*ed_k;
                    F_k   = max(-p.Fmax, min(p.Fmax, F_k));

                    % Step dynamics (simple Euler, no disturbance, phi fixed)
                    sth = sin(th_k); cth = cos(th_k);
                    arm = p.R + p.l*sth;
                    M11 = p.mc*p.R^2 + p.m*arm^2;
                    phi_ddot = p.R*F_k / M11;
                    theta_ddot = arm*p.l*cth*(x_k(2)^2)/p.l - p.g*sin(th_k)/p.l;
                    x_k = x_k + opts.dt_test * [x_k(2); phi_ddot; x_k(4); theta_ddot];

                    cost = cost + th_k^2 + 0.001*(F_k/p.Fmax)^2;
                    if abs(th_k) > pi, break; end
                end

                if cost < cost_min_grid(i,j)
                    cost_min_grid(i,j) = cost;
                    kp_best_grid(i,j)  = kp_i;
                    kd_best_grid(i,j)  = kd_i;
                end
            end
        end

        if opts.verbose
            fprintf('  (e=%+6.1f, edot=%+6.1f) -> kp=%.1f kd=%.1f cost=%.3f\n', ...
                    e_ij, edot_ij, kp_best_grid(i,j), kd_best_grid(i,j), cost_min_grid(i,j));
        end
    end
end

elapsed = toc;
fprintf('[fp_rule_designer] Done in %.1f s.\n', elapsed);

% ---- Interpolate grid results to N x N rule table ----------------------
n_sets  = fcfg.n_sets;
centres = linspace(fcfg.range(1), fcfg.range(2), n_sets);

[Xg, Yg] = meshgrid(e_grid, edot_grid);
[Xi, Yi] = meshgrid(centres, centres);

% Interpolate kp and kd surfaces
kp_rule_raw = interp2(Xg, Yg, kp_best_grid, Xi, Yi, 'linear', 0);
kd_rule_raw = interp2(Xg, Yg, kd_best_grid, Xi, Yi, 'linear', 0);

% Clamp to gain bounds
kp_rule = max(gcfg.kp_min, min(gcfg.kp_max, kp_rule_raw));
kd_rule = max(gcfg.kd_min, min(gcfg.kd_max, kd_rule_raw));

% Ki: set to a small fraction of kp (integral action)
ki_rule = kp_rule * 0.0003;
ki_rule = max(0, min(gcfg.ki_max, ki_rule));

% ---- Build output rule base -------------------------------------------
rb_suggested = struct();
rb_suggested.kp_rule    = kp_rule;
rb_suggested.kd_rule    = kd_rule;
rb_suggested.ki_rule    = ki_rule;
rb_suggested.preset     = 'data_driven';
rb_suggested.n_sets     = n_sets;
rb_suggested.description = sprintf( ...
    'Data-driven rules from %dx%d sweep (grid_n=%d, horizon=%d steps)', ...
    n_sets, n_sets, n_grid, opts.horizon);

% ---- Report ------------------------------------------------------------
report = struct();
report.e_grid         = e_grid;
report.edot_grid      = edot_grid;
report.kp_best_grid   = kp_best_grid;
report.kd_best_grid   = kd_best_grid;
report.cost_min_grid  = cost_min_grid;
report.kp_rule        = kp_rule;
report.kd_rule        = kd_rule;
report.elapsed_s      = elapsed;
report.grid_n         = n_grid;
report.suggestion_note = ['These are SUGGESTED starting rules. '...
    'Review the heatmap in the GUI (Tab 2 -> Rule Designer) and '...
    'accept, modify, or reject individual cells before running.'];

fprintf('  Rule suggestion: kp range [%.0f, %.0f], kd range [%.0f, %.0f]\n', ...
        min(kp_rule(:)), max(kp_rule(:)), min(kd_rule(:)), max(kd_rule(:)));

end
