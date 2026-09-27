function fp_build_report(varargin)
% FP_BUILD_REPORT
% Generates report_data.js from the run folders, then opens report.html.
%
% Emits, per baseline run:
%   - physical / fuzzy / linearisation configuration
%   - per-controller metrics INCLUDING position regulation
%     (xc_final_error, t_xc_settle, xc_settled, xc_rms) and the xc trace
%   - a disturbances array, one entry per <run>/disturbance/<type>/,
%     with recovery metrics computed from each trace
%
% Emits, once, if opt_results.mat is present:
%   - the optimisation summary: per-controller tuned parameters, the
%     analytically-derived warm start, and the cost improvement between them
%
% Usage:
%   fp_build_report                              % scan all baseline runs
%   fp_build_report('Pendulum_Control_Run_003')  % one folder

% =========================================================================
%  LOCATE BASELINE RUNS
% =========================================================================
if nargin >= 1
    run_dirs = {varargin{1}};
else
    d = dir('Pendulum_Control_Run_*');
    d = d([d.isdir]);
    if isempty(d)
        error('No Pendulum_Control_Run_NNN folders found.');
    end
    % Sort by run number so the report lists them in order
    nums = nan(1,numel(d));
    for i = 1:numel(d)
        tok = regexp(d(i).name,'^Pendulum_Control_Run_(\d+)$','tokens','once');
        if ~isempty(tok), nums(i) = str2double(tok{1}); end
    end
    keep = ~isnan(nums);
    d = d(keep); nums = nums(keep);
    [~,o] = sort(nums);
    run_dirs = {d(o).name};
end

fprintf('\n[fp_build_report] Building report from %d run(s)...\n', numel(run_dirs));

DIST_TYPES = {'kick','wind','friction','mass','tilt'};

runs_json = {};
for ri = 1:numel(run_dirs)
    rdir = run_dirs{ri};
    fprintf('  Processing: %s\n', rdir);

    mat_file = fullfile(rdir,'results.mat');
    if ~isfile(mat_file)
        fprintf('    skipped (no results.mat)\n');
        continue;
    end
    try
        S   = load(mat_file,'results','cfg');
        res = S.results;
        cfg = S.cfg;
    catch
        fprintf('    skipped (could not load)\n');
        continue;
    end
    if ~isstruct(res) || ~isfield(res,'runs')
        fprintf('    skipped (unexpected contents)\n');
        continue;
    end

    p    = res.p;
    fcfg = res.fcfg;
    rb   = res.rule_base;
    keys = fieldnames(res.runs);

    xc_ref = 0;
    if isfield(p,'xc_ref'), xc_ref = p.xc_ref; end

    jrun = struct();
    jrun.id      = rdir;
    jrun.name    = rdir;
    jrun.swingup = cfg.swingup;

    jrun.params = struct('mc',p.mc,'m',p.m,'l',p.l, ...
                         'Fmax',p.Fmax,'g',p.g,'name',p.name, ...
                         'omega_n',p.omega_n,'E_target',p.E_target, ...
                         'capture_deg',p.capture_deg, ...
                         'xc_ref',xc_ref);

    jrun.fuzzy = struct('n_sets',fcfg.n_sets,'mf_type',fcfg.mf_type, ...
                        'partition',fcfg.partition,'inference',fcfg.inference, ...
                        'and_op',fcfg.and_op,'defuzz',fcfg.defuzz, ...
                        'type2',fcfg.type2,'fou_width',fcfg.fou_width, ...
                        'rule_preset',rb.preset);

    jrun.lin_info = struct( ...
        'rank_co',  res.lin_info.rank_co, ...
        'rank_obs', res.lin_info.rank_obs, ...
        'note',     res.lin_info.note);
    if isfield(res.lin_info,'rank_aug')
        jrun.lin_info.rank_aug = res.lin_info.rank_aug;
    end

    % ---- Governor settings actually used -------------------------------
    jrun.governor = struct('kp_gov',NaN,'kd_gov',NaN, ...
                           'max_tilt',NaN,'kp_inner',NaN,'kd_inner',NaN);
    if isfield(p,'govcfg')
        g = p.govcfg;
        f = {'kp_gov','kd_gov','max_tilt','kp_inner','kd_inner'};
        for i = 1:numel(f)
            if isfield(g,f{i}), jrun.governor.(f{i}) = g.(f{i}); end
        end
    end

    % ---- Controllers ----------------------------------------------------
    ctrl_arr = {};
    for ki = 1:numel(keys)
        key  = keys{ki};
        run  = res.runs.(key);
        m    = run.metrics;
        lyap = run.lyapunov;

        [t_ds, idx] = downsample_t(run.t, 500);
        th_ds = rad2deg(run.x(idx,3));
        xc_ds = run.x(idx,1);
        E_ds  = run.E(idx);
        F_ds  = run.F(idx);
        th_ds(isnan(th_ds)) = 999;
        E_ds(isnan(E_ds))   = 999;
        xc_ds(isnan(xc_ds)) = 999;

        ctrl = struct();
        ctrl.key          = key;
        ctrl.captured     = run.captured;

        % Angle regulation
        ctrl.t_capture    = m.t_capture;
        ctrl.t_settle     = m.t_settle;
        ctrl.overshoot    = m.overshoot_deg;
        ctrl.rms_theta    = m.rms_theta_deg;

        % Position regulation
        ctrl.xc_ref         = getf(m,'xc_ref',        xc_ref);
        ctrl.xc_final       = getf(m,'xc_final',      NaN);
        ctrl.xc_final_error = getf(m,'xc_final_error',NaN);
        ctrl.xc_rms         = getf(m,'xc_rms',        NaN);
        ctrl.t_xc_settle    = getf(m,'t_xc_settle',   NaN);
        ctrl.xc_settled     = logical(getf(m,'xc_settled',false));

        % Effort and Lyapunov
        ctrl.total_effort = m.total_effort;
        ctrl.rms_F        = m.rms_F;
        ctrl.pct_ok_su    = lyap.pct_ok_su;
        ctrl.pct_ok_stab  = lyap.pct_ok_stab;

        % Traces
        ctrl.t      = t_ds(:)';
        ctrl.theta  = th_ds(:)';
        ctrl.xc     = xc_ds(:)';
        ctrl.energy = E_ds(:)';
        ctrl.F      = F_ds(:)';

        % Governor reference trace, when recorded
        if isfield(run,'theta_ref_hist') && numel(run.theta_ref_hist) >= max(idx)
            tr = run.theta_ref_hist(idx);
            tr(isnan(tr)) = 0;
            ctrl.theta_ref = tr(:)';
        else
            ctrl.theta_ref = zeros(1,numel(t_ds));
        end

        % ---- Asset paths (match the current file names) -----------------
        fig_dir  = fullfile(rdir,'figures');
        anim_dir = fullfile(rdir,'animation');
        ctrl.assets = struct( ...
            'response_png', bld_path(fig_dir,  [key '_response.png']), ...
            'lyapunov_png', bld_path(fig_dir,  [key '_lyapunov.png']), ...
            'phase_png',    bld_path(fig_dir,  [key '_phase_plane.png']), ...
            'gains_png',    bld_path(fig_dir,  [key '_gains.png']), ...
            'pendulum_gif', bld_path(anim_dir, [key '_pendulum.gif']), ...
            'graphs_gif',   bld_path(anim_dir, [key '_graphs.gif']));

        ctrl_arr{end+1} = ctrl; %#ok<AGROW>
    end
    jrun.controllers = ctrl_arr;

    % ---- Shared figures --------------------------------------------------
    fdir = fullfile(rdir,'figures');
    jrun.params_png     = bld_path(fdir,'00_parameters.png');
    jrun.mf_png         = bld_path(fdir,'fuzzy_mf.png');
    jrun.surface_png    = bld_path(fdir,'control_surface.png');
    jrun.heatmap_png    = bld_path(fdir,'rulebase_heatmap.png');
    jrun.comparison_png = bld_path(fdir,'comparison_all.png');
    jrun.metrics_png    = bld_path(fdir,'metrics_comparison.png');

    jrun.kp_rule = rb.kp_rule;
    jrun.kd_rule = rb.kd_rule;

    % ---- Disturbance sub-runs -------------------------------------------
    jrun.disturbances = scan_disturbances(rdir, DIST_TYPES, xc_ref);
    if ~isempty(jrun.disturbances)
        n_media = 0;
        for dq = 1:numel(jrun.disturbances)
            cs = jrun.disturbances{dq}.controllers;
            for cq = 1:numel(cs)
                if ~isempty(cs{cq}.assets.pendulum_gif)
                    n_media = n_media + 1;
                end
            end
        end
        fprintf('    disturbances: %d  (%d with animations)\n', ...
                numel(jrun.disturbances), n_media);
    end

    runs_json{end+1} = jrun; %#ok<AGROW>
end

if isempty(runs_json)
    error('No valid runs found.');
end

% =========================================================================
%  OPTIMISATION RESULTS (optional)
% =========================================================================
opt_json = build_opt_section();

% =========================================================================
%  WRITE
% =========================================================================
data = struct();
data.schema        = 'fuzzy-pendulum-control/2.0';
data.generated_utc = [char(datetime('now','TimeZone','UTC', ...
                      'Format','yyyy-MM-dd HH:mm:ss')) ' UTC'];
data.runs          = runs_json;
data.optimisation  = opt_json;

json_str = jsonencode(data,'PrettyPrint',true);
fid = fopen('report_data.js','w');
fprintf(fid,'/* Auto-generated by fp_build_report.m -- do not edit */\n');
fprintf(fid,'window.PENDULUM_REPORT_DATA = %s;\n', json_str);
fclose(fid);

n_dist = 0;
for i = 1:numel(runs_json)
    n_dist = n_dist + numel(runs_json{i}.disturbances);
end

fprintf('\n[fp_build_report] Done.\n');
fprintf('  runs:          %d\n', numel(runs_json));
fprintf('  disturbances:  %d\n', n_dist);
if isempty(opt_json)
    fprintf('  optimisation:  none (run opt_run to include it)\n');
else
    fprintf('  optimisation:  %d controller(s)\n', numel(opt_json.controllers));
end
fprintf('  -> report_data.js\n\n');

html_path = fullfile(pwd,'report.html');
if isfile(html_path)
    try
        if ispc,      system(sprintf('start "" "%s"', html_path));
        elseif ismac, system(sprintf('open "%s"', html_path));
        else,         system(sprintf('xdg-open "%s" &', html_path));
        end
    catch
    end
end
end


% =========================================================================
%  DISTURBANCE SCAN
% =========================================================================
function arr = scan_disturbances(rdir, types, xc_ref_default)
% Reads <rdir>/disturbance/<type>/results.mat and computes, per controller,
% how far the disturbance pushed the system and how long recovery took.

arr = {};
droot = fullfile(rdir,'disturbance');
if ~isfolder(droot), return; end

for ti = 1:numel(types)
    dtype = types{ti};
    ddir  = fullfile(droot, dtype);
    mf    = fullfile(ddir,'results.mat');
    if ~isfile(mf), continue; end

    try
        D   = load(mf,'results','cfg');
        res = D.results;
        cfg = D.cfg;
    catch
        continue;
    end
    if ~isfield(res,'runs'), continue; end

    p = res.p;
    xc_ref = xc_ref_default;
    if isfield(p,'xc_ref'), xc_ref = p.xc_ref; end

    % When the disturbance starts, so "recovery" is measured from onset
    t_onset = disturbance_onset(dtype, cfg);

    entry = struct();
    entry.type    = dtype;
    entry.dir     = strrep(ddir,'\','/');
    entry.t_onset = t_onset;
    entry.xc_ref  = xc_ref;
    entry.warm_start = isfield(cfg,'warm_start');

    keys = fieldnames(res.runs);
    carr = {};
    for ki = 1:numel(keys)
        key = keys{ki};
        run = res.runs.(key);
        t   = run.t(:);
        th  = rad2deg(run.x(:,3));
        xc  = run.x(:,1);

        post = t >= t_onset;
        if ~any(post), post = true(size(t)); end

        c = struct();
        c.key = key;

        % Peak excursion after the disturbance hits
        c.max_theta_dev = max(abs(th(post)));
        c.max_xc_dev    = max(abs(xc(post) - xc_ref));

        % Recovery: first time after onset from which the signal stays
        % inside tolerance for the remainder of the run
        c.recover_theta = recovery_time(t, abs(th),            3.00, t_onset);
        c.recover_xc    = recovery_time(t, abs(xc - xc_ref),   0.05, t_onset);

        % Did it survive at all
        c.lost = any(abs(th(post)) > 60);

        % End state
        c.theta_final     = th(end);
        c.xc_final_error  = abs(xc(end) - xc_ref);

        % Short traces for plotting
        [t_ds, idx]  = downsample_t(t, 300);
        c.t     = t_ds(:)';
        c.theta = th(idx)';
        c.xc    = xc(idx)';

        % Figures and animations for this disturbance run. These are only
        % present if the run was exported/animated (GUI buttons, or
        % export_figures = true in the disturbance opts).
        dfig  = fullfile(ddir,'figures');
        danim = fullfile(ddir,'animation');
        c.assets = struct( ...
            'response_png', bld_path(dfig,  [key '_response.png']), ...
            'phase_png',    bld_path(dfig,  [key '_phase_plane.png']), ...
            'lyapunov_png', bld_path(dfig,  [key '_lyapunov.png']), ...
            'gains_png',    bld_path(dfig,  [key '_gains.png']), ...
            'pendulum_gif', bld_path(danim, [key '_pendulum.gif']), ...
            'graphs_gif',   bld_path(danim, [key '_graphs.gif']));

        carr{end+1} = c; %#ok<AGROW>
    end
    entry.controllers = carr;
    arr{end+1} = entry; %#ok<AGROW>
end
end


function t0 = disturbance_onset(dtype, cfg)
t0 = 0;
if ~isfield(cfg,'disturbance'), return; end
d = cfg.disturbance;
switch dtype
    case 'kick',     t0 = getf(d,'kick_time',0);
    case 'wind',     t0 = getf(d,'wind_start',0);
    case 'friction', t0 = getf(d,'friction_start',0);
    case 'mass',     t0 = getf(d,'mass_time',0);
    case 'tilt',     t0 = 0;    % tilt is present from t = 0
end
end


function tr = recovery_time(t, err, tol, t_onset)
% Seconds after t_onset before err stays below tol for the rest of the run.
% NaN if it never settles.
tr = NaN;
i0 = find(t >= t_onset, 1);
if isempty(i0), return; end
for k = i0:numel(t)
    if all(err(k:end) < tol)
        tr = t(k) - t_onset;
        return;
    end
end
end


% =========================================================================
%  OPTIMISATION SECTION
% =========================================================================
function o = build_opt_section()
% Reads opt_results.mat, if present, and summarises the tuning outcome.
o = [];
if ~isfile('opt_results.mat'), return; end

try
    S = load('opt_results.mat','opt_results');
    r = S.opt_results;
catch
    return;
end
if ~isfield(r,'controllers'), return; end

o = struct();
o.preset       = getf(r,'preset','');
o.swingup      = getf(r,'swingup','');
o.generated    = getf(r,'generated','');
o.cost_weights = getf(r,'cost_weights',[]);

if isfield(r,'gov_ws')
    o.warm_start_gains = r.gov_ws;
end
if isfield(r,'lqr_K5_ws')
    o.lqr_K5 = r.lqr_K5_ws;
end

keys = fieldnames(r.controllers);
carr = {};
for i = 1:numel(keys)
    k  = keys{i};
    rc = r.controllers.(k);
    m  = rc.best_metrics;

    c = struct();
    c.key         = k;
    c.cost        = rc.best_cost;
    c.cost_warm   = getf(rc,'cost_warm',NaN);
    c.params      = rc.best_params(:)';
    c.warm_start  = rc.warm_start(:)';
    c.param_names = rc.param_names;
    c.lb          = rc.lb(:)';
    c.ub          = rc.ub(:)';

    c.t_settle       = getf(m,'t_settle',NaN);
    c.overshoot      = getf(m,'overshoot_deg',NaN);
    c.xc_final_error = getf(m,'xc_final_error',NaN);
    c.total_effort   = getf(m,'total_effort',NaN);
    c.captured       = logical(getf(m,'captured',false));

    % Normalised parameters, so controllers with different units compare
    span = max(c.ub - c.lb, 1e-9);
    c.params_norm     = (c.params     - c.lb) ./ span;
    c.warm_start_norm = (c.warm_start - c.lb) ./ span;

    carr{end+1} = c; %#ok<AGROW>
end
o.controllers = carr;
end


% =========================================================================
%  HELPERS
% =========================================================================
function [t_ds, idx] = downsample_t(t, max_pts)
n    = numel(t);
skip = max(1, floor(n/max_pts));
idx  = 1:skip:n;
t_ds = t(idx);
end

function v = getf(s, f, default)
if isstruct(s) && isfield(s,f) && ~isempty(s.(f))
    v = s.(f);
else
    v = default;
end
end

function s = bld_path(folder, fname)
full = fullfile(folder,fname);
if isfile(full)
    s   = strrep(full,'\','/');
    cwd = strrep(pwd,'\','/');
    if startsWith(s,cwd), s = s(length(cwd)+2:end); end
else
    s = '';
end
end
