% OPT_RUN
% Nelder-Mead optimisation driver for pendulum gain tuning.
%
% Finds the best gains for each controller by running the full nonlinear
% simulation as the cost function. Uses LQR-derived warm starts so the
% optimiser begins in a physically sensible neighbourhood, then refines
% empirically using the actual nonlinear dynamics.
%
% Strategy:
%   1. Compute LQR gains from the linearised model (warm start)
%   2. Translate LQR gains to initial parameters for each controller type
%   3. Run fminsearch (Nelder-Mead) for each controller
%   4. Repeat from N_starts perturbed initial points (multi-start)
%   5. Keep the best result across all starts
%   6. Save results to opt_results.mat
%
% Usage:
%   >> opt_run             % optimise all controllers, medium preset
%   >> opt_run             % edit CONFIG block below to change settings
%
% After running:
%   >> opt_visualise       % plot convergence and Pareto front
%   >> opt_apply           % write best gains into run_all config

% =========================================================================
%  CONFIG  (edit this block)
% =========================================================================

% ---- Which controllers to optimise -------------------------------------
% Comment out any you want to skip (LQR is fast; AdaptiveFuzzy is slow)
controllers_to_opt = {
    'FuzzyPID_Sugeno'
    'FuzzyPID_Mamdani'
    'FuzzyPID_Type2'
    'FuzzySMC'
    'AdaptiveFuzzy'
    'LQR'
    'ClassicalPID'
};

% ---- Preset and simulation settings ------------------------------------
scale_preset   = 'medium';    % 'small' | 'medium' | 'large'
swingup_strat  = 'energy';    % 'energy' | 'backstepping'
t_end          = 20.0;        % simulation duration (s) — shorter = faster opt
dt             = 0.005;       % integration step (s)
k_su           = 10.0;        % swing-up energy gain

% ---- Cost function weights  [t_settle, overshoot, xc_rms, effort] -----
% Higher weight = optimiser tries harder to minimise that term
cost_weights = [2.0, 1.0, 0.5, 0.3];

% ---- Optimiser settings ------------------------------------------------
max_evals  = 300;    % max function evaluations per start
tol_fun    = 1e-3;   % convergence tolerance on cost
tol_x      = 1e-4;   % convergence tolerance on params
N_starts   = 3;      % number of random restarts (multi-start)
perturb_sd = 0.20;   % std dev of start perturbation (fraction of warm start)
rng_seed   = 42;     % random seed for reproducibility

% ---- LQR Q/R for warm start (used to derive ALL initial gains) --------
lqr_Q_ws  = [0.1, 0.02, 200.0, 20.0];   % Q diagonal  [xc xc_dot th th_dot]
lqr_qi_ws = 0.1;                         % integral-xc weight (5th state)
lqr_R_ws  = 0.01;                        % R scalar

% =========================================================================
%  SETUP
% =========================================================================
rng(rng_seed);
fprintf('\n');
fprintf('##########################################################\n');
fprintf('##   Pendulum Gain Optimisation — Nelder-Mead          ##\n');
fprintf('##########################################################\n\n');

t_total = tic;

% Build base configuration (template for opt_cost)
p         = pendulum_params(scale_preset);
base_cfg  = build_base_cfg(p, scale_preset, swingup_strat, t_end, dt, k_su, lqr_Q_ws, lqr_R_ws);

% Compute the analytically-derived gains used as the warm start.
% These are the same values fp_run_analysis derives when no override is
% supplied, so evaluation #1 of every controller reproduces the untuned
% baseline exactly.
[K5_ws, gov_ws] = compute_lqr(p, lqr_Q_ws, lqr_qi_ws, lqr_R_ws);
fprintf('Augmented LQR K5: [%.3f %.3f %.3f %.3f %.3f]\n', K5_ws);
fprintf('Derived governor: kp_gov=%.3f deg/m   kd_gov=%.3f deg*s/m\n', ...
        gov_ws.kp_gov, gov_ws.kd_gov);
fprintf('Derived inner:    kp=%.4f N/deg       kd=%.4f N*s/deg\n\n', ...
        gov_ws.kp_inner, gov_ws.kd_inner);

% Storage for results
opt_results = struct();
opt_results.preset       = scale_preset;
opt_results.swingup      = swingup_strat;
opt_results.cost_weights = cost_weights;
opt_results.lqr_K5_ws    = K5_ws;
opt_results.gov_ws       = gov_ws;
opt_results.generated    = char(datetime('now','Format','yyyy-MM-dd HH:mm:ss'));
opt_results.controllers  = struct();

% =========================================================================
%  OPTIMISE EACH CONTROLLER
% =========================================================================
for ci = 1:numel(controllers_to_opt)
    ctrl = controllers_to_opt{ci};
    fprintf('----------------------------------------------------------\n');
    fprintf('  Optimising: %s\n', ctrl);
    fprintf('----------------------------------------------------------\n');

    t_ctrl = tic;

    % ---- Initial parameter vector from LQR warm start ------------------
    p0 = warm_start(ctrl, gov_ws, base_cfg);
    fprintf('  Warm start: [%s]\n', num2str(p0,'%.3f  '));

    % ---- Parameter bounds (for clamping inside opt_cost unpack_params) -
    [lb, ub] = param_bounds(ctrl);
    p0 = min(ub, max(lb, p0));   % clamp warm start to bounds

    % ---- fminsearch options --------------------------------------------
    fmin_opts = optimset('MaxFunEvals', max_evals, ...
                         'TolFun',      tol_fun, ...
                         'TolX',        tol_x, ...
                         'Display',     'off');

    % ---- Cost at the analytical warm start ------------------------------
    % Recorded so the report can state exactly what the tuning added over
    % the untuned, analytically-derived design.
    cost_ws = safe_cost(p0, ctrl, base_cfg, cost_weights);
    fprintf('  Warm-start cost: %.4f\n', cost_ws);

    % ---- Multi-start Nelder-Mead ----------------------------------------
    best_cost   = inf;
    best_params = p0;
    best_metrics= struct();
    cost_history= [];   % all evaluations across all starts

    for si = 1:N_starts
        if si == 1
            p_start = p0;   % first start: exact warm start
        else
            % Perturb warm start: add Gaussian noise scaled by parameter magnitude
            noise   = randn(size(p0)) * perturb_sd;
            p_start = p0 .* (1 + noise);
            p_start = min(ub, max(lb, p_start));   % re-clamp
        end

        fprintf('  Start %d/%d: [%s]\n', si, N_starts, num2str(p_start,'%.3f  '));

        % Wrap cost function to log history
        eval_count = 0;
        hist_this  = [];

        cost_fn = @(params) eval_and_log(params, ctrl, base_cfg, ...
                                          cost_weights, hist_this);

        % Run optimiser
        try
            [p_opt, cost_opt] = fminsearch( ...
                @(params) safe_cost(params, ctrl, base_cfg, cost_weights), ...
                p_start, fmin_opts);
        catch ME
            fprintf('    WARNING: fminsearch failed (%s), using start point.\n', ME.message);
            p_opt    = p_start;
            cost_opt = safe_cost(p_start, ctrl, base_cfg, cost_weights);
        end

        fprintf('    Cost: %.4f\n', cost_opt);

        if cost_opt < best_cost
            best_cost    = cost_opt;
            best_params  = p_opt;
            % Get full metrics for the best result
            [~, best_metrics] = opt_cost(p_opt, ctrl, base_cfg, cost_weights);
        end
    end

    elapsed = toc(t_ctrl);
    fprintf('  Best cost: %.4f  (%.1fs elapsed)\n', best_cost, elapsed);
    if isfield(best_metrics,'t_settle') && ~isnan(best_metrics.t_settle)
        fprintf('  Best metrics: settle=%.2fs  overshoot=%.1fdeg  xc_rms=%.3fm  effort=%.1fN*s\n', ...
                best_metrics.t_settle, best_metrics.overshoot_deg, ...
                best_metrics.xc_rms,  best_metrics.total_effort);
    end
    fprintf('\n');

    % ---- Store results -------------------------------------------------
    r = struct();
    r.ctrl_name    = ctrl;
    r.best_params  = best_params;
    r.best_cost    = best_cost;
    r.cost_warm    = cost_ws;
    r.best_metrics = best_metrics;
    r.warm_start   = p0;
    r.param_names  = param_names(ctrl);
    r.lb           = lb;
    r.ub           = ub;
    opt_results.controllers.(ctrl) = r;
end

% =========================================================================
%  SAVE
% =========================================================================
save('opt_results.mat', 'opt_results', '-v7.3');
fprintf('##########################################################\n');
fprintf('##  Optimisation complete  (%.1fs total)              ##\n', toc(t_total));
fprintf('##  Results saved to opt_results.mat\n');
fprintf('##\n');
fprintf('##  Next steps:\n');
fprintf('##    >> opt_visualise    (convergence + Pareto front)\n');
fprintf('##    >> opt_apply        (write best gains to config)\n');
fprintf('##########################################################\n\n');


% =========================================================================
%  HELPERS
% =========================================================================
function cfg = build_base_cfg(p, preset, swingup, t_end, dt, k_su, lqr_Q, lqr_R)
% Build a complete cfg struct that opt_cost can use as a template

cfg.p          = p;
cfg.swingup    = swingup;
cfg.t_end      = t_end;
cfg.dt         = dt;
cfg.x0         = [0; 0; pi; 0.05];
cfg.run_dir    = '';       % will be replaced by tempname() in opt_cost
cfg.chk_interval = 1e9;   % effectively disable checkpointing during opt

% Swing-up config
cfg.sucfg.k_su     = k_su;
cfg.sucfg.k1       = 4.0;
cfg.sucfg.k2       = 10.0;
cfg.sucfg.E_thresh = 0.05;
cfg.sucfg.omega    = 0.8 * p.omega_n;
cfg.sucfg.F_amp    = 0.85 * p.Fmax;
cfg.sucfg.phase_aware = true;

% Default fuzzy config
cfg.fcfg = struct('n_sets',7,'range',[-180 180],'mf_type','triangular', ...
                  'partition','uniform','inference','sugeno','and_op','min', ...
                  'defuzz','weighted_avg','type2',false,'fou_width',0.20);

% Rule base
cfg.rule_base = fp_rule_base('expert', 7);

% Default gain map (will be overridden per controller in opt_cost)
cfg.gcfg = struct('kp_max',300,'kd_max',50,'ki_max',0.1, ...
                  'kp_min',0,'kd_min',0,'ki_min',0,'use_lpf',false, ...
                  'kp_xc',1.5,'kd_xc',2.0);

% Default PID config
cfg.pidcfg = struct('kp',80,'kd',15,'ki',0.02,'auto_tune',false, ...
                    'i_clamp',500,'kp_xc',1.5,'kd_xc',2.0);

% Default SMC config
cfg.smccfg = struct('lambda',5,'phi_boundary',0.05,'K_min',2,'K_max',15, ...
                    'kp_xc',1.5,'kd_xc',2.0);

% Default adaptive config
cfg.adapt_cfg = struct('eta_kp',0.01,'eta_kd',0.005,'eta_ki',0.001, ...
                       'freeze',false,'kp_bounds',[0 500], ...
                       'kd_bounds',[0 100],'ki_bounds',[0 0.5]);

% LQR weights (qi = integral-xc weight for the augmented solution)
cfg.lqr_Q  = lqr_Q;
cfg.lqr_qi = lqr_Q(1);
cfg.lqr_R  = lqr_R;

% Governor: empty = fp_run_analysis derives gains from the K5 solution.
% opt_cost overrides individual fields per evaluation.
cfg.govcfg = struct();

% Disturbance (off during optimisation)
cfg.disturbance = struct('kick_enabled',false,'wind_enabled',false, ...
    'friction_enabled',false,'mass_enabled',false,'tilt_enabled',false);

% Quiet mode for optimiser
cfg.quiet = true;

% Type-2 MF config for FuzzyPID_Type2
cfg.fcfg_t2         = cfg.fcfg;
cfg.fcfg_t2.type2   = true;
cfg.fcfg_t2.fou_width = 0.20;
end


function [K5, gov] = compute_lqr(p, Q_diag, qi, R_val)
% Augmented 5-state LQR: state = [xc; xc_dot; theta; theta_dot; int(xc_err)]
% Returns K5 and the governor/inner gains derived from it — the same
% derivation fp_run_analysis performs, so warm starts land exactly on the
% analytically-derived operating point.
K5  = zeros(1,5);
gov = struct('kp_gov',3.0,'kd_gov',5.0,'kp_inner',4.0,'kd_inner',1.0);
try
    [~,~,~,~,lin] = pendulum_linearise(p, true);
    [~,~,K] = care(lin.Aa, lin.Ba, diag([Q_diag(:); qi]), R_val);
    K5 = K(:)';
    gov.kp_gov   = abs(K5(1)/K5(3)) * (180/pi);
    gov.kd_gov   = abs(K5(2)/K5(3)) * (180/pi);
    gov.kp_inner = abs(K5(3)) * (pi/180);
    gov.kd_inner = abs(K5(4)) * (pi/180);
catch
end
end


function p0 = warm_start(ctrl_name, gov, cfg)
% Initial parameter vector, anchored on the analytically-derived gains.
%
% gov holds kp_gov / kd_gov / kp_inner / kd_inner from the augmented LQR
% solution — i.e. exactly what the controller would use with no tuning at
% all. Starting the optimiser there means any improvement it reports is a
% genuine gain over the analytical design, not a recovery from a bad guess.

switch ctrl_name
    case {'FuzzyPID_Sugeno','FuzzyPID_Mamdani','FuzzyPID_Type2'}
        % [kp_max, kd_max, inner_scale, kp_gov, kd_gov, max_tilt]
        p0 = [cfg.gcfg.kp_max, cfg.gcfg.kd_max, 1.0, ...
              gov.kp_gov, gov.kd_gov, 10.0];

    case 'AdaptiveFuzzy'
        % [kp_max, kd_max, eta_kp, eta_kd, kp_gov, kd_gov]
        p0 = [cfg.gcfg.kp_max, cfg.gcfg.kd_max, ...
              cfg.adapt_cfg.eta_kp, cfg.adapt_cfg.eta_kd, ...
              gov.kp_gov, gov.kd_gov];

    case 'FuzzySMC'
        % [lambda, phi_boundary, K_min, K_max, kp_gov, kd_gov]
        p0 = [cfg.smccfg.lambda, cfg.smccfg.phi_boundary, ...
              cfg.smccfg.K_min,  cfg.smccfg.K_max, ...
              gov.kp_gov, gov.kd_gov];

    case 'LQR'
        % [Q1, Q2, Q3, Q4, qi, R]
        p0 = [cfg.lqr_Q(1), cfg.lqr_Q(2), cfg.lqr_Q(3), cfg.lqr_Q(4), ...
              cfg.lqr_qi,   cfg.lqr_R];

    case 'ClassicalPID'
        % [kp, kd, ki, kp_gov, kd_gov, max_tilt]
        % Start from the LQR-equivalent gains, not the legacy defaults.
        p0 = [gov.kp_inner, gov.kd_inner, 0.02, ...
              gov.kp_gov,   gov.kd_gov,   10.0];

    otherwise
        p0 = ones(1,6);
end
end


function [lb, ub] = param_bounds(ctrl_name)
switch ctrl_name
    case {'FuzzyPID_Sugeno','FuzzyPID_Mamdani','FuzzyPID_Type2'}
        %     kp_max  kd_max  in_scale  kp_gov  kd_gov  max_tilt
        lb = [  20,     2,      0.25,     0.3,    0.3,     3  ];
        ub = [ 600,   150,      3.00,    12.0,   20.0,    25  ];

    case 'AdaptiveFuzzy'
        %     kp_max  kd_max  eta_kp   eta_kd   kp_gov  kd_gov
        lb = [  20,     2,    1e-4,    1e-5,     0.3,    0.3 ];
        ub = [ 600,   150,    0.10,    0.05,    12.0,   20.0 ];

    case 'FuzzySMC'
        %     lambda  phi_b  K_min  K_max  kp_gov  kd_gov
        lb = [  0.5,  0.005,   0,     1,     0.3,    0.3 ];
        ub = [ 20.0,  0.500,  10,    40,    12.0,   20.0 ];

    case 'LQR'
        %      Q1      Q2      Q3     Q4     qi       R
        lb = [ 0.01,  0.001,   10,     1,   0.001,  1e-4 ];
        ub = [ 100,   50,    2000,   500,   50,     10   ];

    case 'ClassicalPID'
        %      kp     kd     ki    kp_gov  kd_gov  max_tilt
        lb = [ 0.5,  0.10,   0,     0.3,    0.3,     3  ];
        ub = [ 100,  30.0,   1.0,  12.0,   20.0,    25  ];

    otherwise
        lb = zeros(1,6);
        ub = ones(1,6) * 1000;
end
end


function names = param_names(ctrl_name)
switch ctrl_name
    case {'FuzzyPID_Sugeno','FuzzyPID_Mamdani','FuzzyPID_Type2'}
        names = {'kp_max','kd_max','inner_scale','kp_gov','kd_gov','max_tilt'};
    case 'AdaptiveFuzzy'
        names = {'kp_max','kd_max','eta_kp','eta_kd','kp_gov','kd_gov'};
    case 'FuzzySMC'
        names = {'lambda','phi_boundary','K_min','K_max','kp_gov','kd_gov'};
    case 'LQR'
        names = {'Q1','Q2','Q3','Q4','qi','R'};
    case 'ClassicalPID'
        names = {'kp','kd','ki','kp_gov','kd_gov','max_tilt'};
    otherwise
        names = {};
end
end


function cost = safe_cost(params, ctrl_name, base_cfg, cost_weights)
% Wrapper that catches errors and returns a large cost instead of crashing
try
    cost = opt_cost(params, ctrl_name, base_cfg, cost_weights);
catch
    cost = 1e6;
end
% Guard against NaN/Inf from numerical issues
if ~isfinite(cost), cost = 1e6; end
end


function [cost, metrics] = eval_and_log(params, ctrl, base_cfg, weights, hist)
% Wrapper used for logging — not currently wired to live history capture
% but provides the hook for Segment O3 visualisation.
[cost, metrics] = opt_cost(params, ctrl, base_cfg, weights);
end

