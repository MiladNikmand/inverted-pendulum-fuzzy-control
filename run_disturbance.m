function run_disturbance(opts)
% RUN_DISTURBANCE
% Runs disturbance experiments from pre-saved settled states.
%
% Callable two ways:
%   >> run_disturbance            % uses the defaults below
%   >> run_disturbance(opts)      % uses caller-supplied settings (GUI)
%
% OUTPUT LAYOUT — everything stays inside the baseline run folder:
%
%   Pendulum_Control_Run_012/          <- baseline (from run_all)
%     results.mat, figures/, animation/
%     disturbance/
%       kick/     results.mat, figures/, animation/, README.txt
%       wind/     ...
%
% OPTS FIELDS (all optional; defaults shown in the block below)
%   .base_run_dir   baseline to warm-start from ('' = auto-detect latest)
%   .test_kick .test_wind .test_friction .test_mass .test_tilt   logical
%   .kick_mag .kick_time .kick_duration
%   .wind_speed .wind_start .wind_ramp .wind_hold
%   .fric_start .fric_end .fric_mag .fric_visc
%   .mass_time .mass_delta
%   .tilt_angle_deg .tilt_phase
%   .t_disturbance  duration of each disturbance run (s)
%   .export_figures write PNGs for each run (slower)
%   .warm_start     true = start from settled state; false = swing up first
%
% WHY WARM START IS VALID
%   Testing disturbance rejection FROM a settled state asks a clean
%   question: "given the system is balanced at xc_ref, how does each
%   controller recover?" This separates swing-up performance (already
%   measured in the baseline run) from disturbance rejection.

if nargin < 1 || isempty(opts), opts = struct(); end

% =========================================================================
%  DEFAULTS  (any field supplied in opts overrides these)
% =========================================================================
opts = dflt(opts,'base_run_dir',   '');

opts = dflt(opts,'test_kick',      true);
opts = dflt(opts,'test_wind',      true);
opts = dflt(opts,'test_friction',  false);
opts = dflt(opts,'test_mass',      false);
opts = dflt(opts,'test_tilt',      false);

opts = dflt(opts,'kick_mag',       5.0);
opts = dflt(opts,'kick_time',      3.0);
opts = dflt(opts,'kick_duration',  0.1);

opts = dflt(opts,'wind_speed',     5.0);
opts = dflt(opts,'wind_start',     2.0);
opts = dflt(opts,'wind_ramp',      0.5);
opts = dflt(opts,'wind_hold',      3.0);

opts = dflt(opts,'fric_start',     2.0);
opts = dflt(opts,'fric_end',       4.0);
opts = dflt(opts,'fric_mag',       2.0);
opts = dflt(opts,'fric_visc',      0.5);

opts = dflt(opts,'mass_time',      2.0);
opts = dflt(opts,'mass_delta',     0.1);

opts = dflt(opts,'tilt_angle_deg', 2.0);
opts = dflt(opts,'tilt_phase',     0);

opts = dflt(opts,'t_disturbance',  20.0);
opts = dflt(opts,'export_figures', false);
opts = dflt(opts,'warm_start',     true);

base_run_dir   = opts.base_run_dir;
t_disturbance  = opts.t_disturbance;
export_figures = opts.export_figures;
use_warm_start = opts.warm_start;

% =========================================================================
%  FIND BASELINE RUN
% =========================================================================
if isempty(base_run_dir)
    base_run_dir = find_latest_baseline();
end

mat_file = fullfile(base_run_dir, 'results.mat');
if ~isfile(mat_file)
    error('results.mat not found in %s', base_run_dir);
end

fprintf('\n');
fprintf('##########################################################\n');
fprintf('##   Disturbance Experiments                           ##\n');
fprintf('##########################################################\n\n');
fprintf('  Baseline: %s\n', base_run_dir);

S = load(mat_file, 'results', 'cfg');
if ~isfield(S,'results') || ~isfield(S,'cfg')
    error(['%s does not contain "results" and "cfg".\n' ...
           'Re-run run_all to regenerate it.'], mat_file);
end
results_base = S.results;
cfg_base     = S.cfg;

% Baseline run number (used for summary headers)
tok = regexp(base_run_dir,'Pendulum_Control_Run_(\d+)','tokens','once');
if isempty(tok), base_num = 0; else, base_num = str2double(tok{1}); end

% =========================================================================
%  EXTRACT SETTLED STATES
% =========================================================================
ctrl_names = fieldnames(results_base.runs);
warm_start = struct();

fprintf('\n  Extracting settled states:\n');
for ci = 1:numel(ctrl_names)
    ctrl = ctrl_names{ci};
    r    = results_base.runs.(ctrl);
    m    = r.metrics;

    % Prefer a time after BOTH theta and xc have settled
    t_use = NaN;
    if ~isnan(m.t_capture)
        if ~isnan(m.t_settle)
            t_use = m.t_capture + m.t_settle + 2.0;
        end
        if isfield(m,'t_xc_settle') && ~isnan(m.t_xc_settle)
            t_use = max(t_use, m.t_capture + m.t_xc_settle + 1.0);
        end
    end
    if isnan(t_use)
        t_use = r.t(end) - 3.0;    % fallback: near the end
    end
    t_use = min(t_use, r.t(end) - 1.0);
    t_use = max(t_use, r.t(1)   + 1.0);

    [~, k_use] = min(abs(r.t - t_use));
    x_settled  = r.x(k_use,:)';

    ws = struct();
    ws.x0       = x_settled;
    ws.t_start  = r.t(k_use);
    ws.lqr_xi   = 0;
    ws.fuzzy_xi = 0;
    warm_start.(ctrl) = ws;

    fprintf('    %-20s t=%6.2fs  theta=%7.2f deg  xc=%7.3f m\n', ...
            ctrl, ws.t_start, rad2deg(x_settled(3)), x_settled(1));
end

% =========================================================================
%  BUILD DISTURBANCE LIST
% =========================================================================
dist_list = {};
if opts.test_kick,     dist_list{end+1} = 'kick';     end
if opts.test_wind,     dist_list{end+1} = 'wind';     end
if opts.test_friction, dist_list{end+1} = 'friction'; end
if opts.test_mass,     dist_list{end+1} = 'mass';     end
if opts.test_tilt,     dist_list{end+1} = 'tilt';     end

if isempty(dist_list)
    error('No disturbances enabled. Set at least one test_* flag to true.');
end

% Parent folder for all disturbance runs
dist_root = fullfile(base_run_dir, 'disturbance');
if ~exist(dist_root,'dir'), mkdir(dist_root); end

fprintf('\n  Output root: %s\n', dist_root);
fprintf('  Disturbances: %s\n', strjoin(dist_list, ', '));
fprintf('  Duration:     %.1f s each\n', t_disturbance);
if use_warm_start
    fprintf('  Start mode:   warm (from settled state)\n\n');
else
    fprintf('  Start mode:   fresh (swing up first)\n\n');
end

t_all = tic;

% =========================================================================
%  RUN EACH DISTURBANCE
% =========================================================================
for di = 1:numel(dist_list)
    dtype = dist_list{di};
    fprintf('----------------------------------------------------------\n');
    fprintf('  [%d/%d]  %s\n', di, numel(dist_list), upper(dtype));
    fprintf('----------------------------------------------------------\n');

    % ---- Subfolder for this disturbance ---------------------------------
    run_dir = fullfile(dist_root, dtype);
    if ~exist(run_dir,'dir'), mkdir(run_dir); end
    if ~exist(fullfile(run_dir,'figures'),'dir')
        mkdir(fullfile(run_dir,'figures'));
    end
    if ~exist(fullfile(run_dir,'animation'),'dir')
        mkdir(fullfile(run_dir,'animation'));
    end

    % ---- Disturbance config (all off, then enable this one) -------------
    dcfg = struct( ...
        'kick_enabled',    false, ...
        'kick_time',       opts.kick_time, ...
        'kick_mag',        opts.kick_mag, ...
        'kick_duration',   opts.kick_duration, ...
        'wind_enabled',    false, ...
        'wind_start',      opts.wind_start, ...
        'wind_ramp',       opts.wind_ramp, ...
        'wind_hold',       opts.wind_hold, ...
        'wind_speed',      opts.wind_speed, ...
        'friction_enabled',false, ...
        'friction_start',  opts.fric_start, ...
        'friction_end',    opts.fric_end, ...
        'friction_mag',    opts.fric_mag, ...
        'friction_visc',   opts.fric_visc, ...
        'mass_enabled',    false, ...
        'mass_time',       opts.mass_time, ...
        'mass_delta',      opts.mass_delta, ...
        'tilt_enabled',    false, ...
        'tilt_angle_deg',  opts.tilt_angle_deg, ...
        'tilt_phase',      opts.tilt_phase);

    switch dtype
        case 'kick',     dcfg.kick_enabled     = true;
        case 'wind',     dcfg.wind_enabled     = true;
        case 'friction', dcfg.friction_enabled = true;
        case 'mass',     dcfg.mass_enabled     = true;
        case 'tilt',     dcfg.tilt_enabled     = true;
    end

    % ---- Config: baseline with warm start and this disturbance ----------
    cfg_d              = cfg_base;
    cfg_d.disturbance  = dcfg;
    cfg_d.t_end        = t_disturbance;
    if use_warm_start
        cfg_d.warm_start = warm_start;
    end
    cfg_d.run_dir      = run_dir;
    cfg_d.chk_interval = 1e9;        % no checkpointing here
    cfg_d.quiet        = false;
    cfg_d.dist_type    = dtype;      % tag for the report
    cfg_d.base_run_dir = base_run_dir;
    cfg_d.x0           = warm_start.(ctrl_names{1}).x0;   % fallback only

    % ---- Run -------------------------------------------------------------
    results = fp_run_analysis(cfg_d);   %#ok<NASGU>
    cfg     = cfg_d;                    %#ok<NASGU>

    % ---- Save with standard variable names -------------------------------
    save(fullfile(run_dir,'results.mat'), 'results', 'cfg', '-v7.3');

    % ---- Summary ---------------------------------------------------------
    try
        fp_write_summary(run_dir, base_num, results, cfg);
    catch ME
        fprintf('  (summary skipped: %s)\n', ME.message);
    end

    % ---- Optional static figures ----------------------------------------
    if export_figures
        try
            eopts = struct('dpi',150,'make_gifs',false, ...
                           'gif_fps',10,'gif_max_frames',400);
            fp_export_figures(run_dir, results, cfg, eopts);
        catch ME
            fprintf('  (figures skipped: %s)\n', ME.message);
        end
    end

    fprintf('  -> %s\n\n', run_dir);
end

% =========================================================================
%  DONE
% =========================================================================
fprintf('##########################################################\n');
fprintf('##  Disturbance runs complete  (%.1f s)\n', toc(t_all));
fprintf('##\n');
fprintf('##  %s/\n', base_run_dir);
fprintf('##    disturbance/\n');
for di = 1:numel(dist_list)
    fprintf('##      %s/\n', dist_list{di});
end
fprintf('##\n');
fprintf('##  Animate one with:\n');
fprintf('##    animate_pendulum(''%s'')\n', ...
        fullfile(base_run_dir,'disturbance',dist_list{1}));
fprintf('##########################################################\n\n');

end   % run_disturbance


% =========================================================================
%  HELPER — fill a default field if the caller did not supply it
% =========================================================================
function o = dflt(o, f, v)
if ~isfield(o,f) || isempty(o.(f)), o.(f) = v; end
end


% =========================================================================
%  HELPER — find the latest baseline run folder
% =========================================================================
function rd = find_latest_baseline()
% Highest-numbered Pendulum_Control_Run_NNN that has a readable
% results.mat at its top level (i.e. a baseline run, not a disturbance).

dirs = dir('Pendulum_Control_Run_*');
dirs = dirs([dirs.isdir]);
if isempty(dirs)
    error(['No Pendulum_Control_Run_NNN folders found in:\n  %s\n' ...
           'Run run_all first.'], pwd);
end

nums = nan(1,numel(dirs));
for di = 1:numel(dirs)
    tok = regexp(dirs(di).name,'^Pendulum_Control_Run_(\d+)$','tokens','once');
    if ~isempty(tok), nums(di) = str2double(tok{1}); end
end
keep = ~isnan(nums);
dirs = dirs(keep); nums = nums(keep);
if isempty(dirs)
    error('No correctly named run folders found.');
end

[~, order] = sort(nums,'descend');
dirs = dirs(order);

for di = 1:numel(dirs)
    mf = fullfile(dirs(di).name,'results.mat');
    if ~isfile(mf), continue; end
    vars = who('-file', mf);
    if ismember('results', vars) && ismember('cfg', vars)
        rd = dirs(di).name;
        return;
    end
end

error(['Found %d run folder(s) but none contain a usable results.mat.\n' ...
       'Run run_all to create a baseline.'], numel(dirs));
end