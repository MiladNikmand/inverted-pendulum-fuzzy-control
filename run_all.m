% RUN_ALL
% Non-interactive batch driver for the Fuzzy Inverted Pendulum project.
% Runs the full pipeline without opening any GUI window.
% For the interactive GUI use:  >> pendulum_launch
%
% Quick start:
%   >> cd path/to/inverted-pendulum-fuzzy-control
%   >> check_setup
%   >> run_all

% =========================================================================
%  CONFIGURATION  (edit this block — mirrors every GUI option)
% =========================================================================

% ---- Optimised gains toggle --------------------------------------------
% Set to true after running opt_run + opt_apply to use optimised gains.
% Set to false to use the manual defaults below.
use_optimised_gains = true;
if use_optimised_gains && isfile('optimised_config.m')
    run optimised_config;
    fprintf('[run_all] Using OPTIMISED gains from optimised_config.m\n');
end
% ------------------------------------------------------------------------

% ---- System scale preset ------------------------------------------------
% 'small' | 'medium' | 'large' | 'custom'
scale_preset = 'medium';

% ---- Custom parameter overrides (only used when scale_preset = 'custom')
custom_mc = 2.5;    % cart mass (kg)
custom_m  = 0.20;   % bob mass (kg)
custom_l  = 0.50;   % rod length (m)

% ---- Initial conditions -------------------------------------------------
xc0_m         = 0;       % cart start position (m)
xc_dot0       = 0;       % cart start velocity (m/s)
theta0_deg    = -178;    % pendulum start angle (deg) — near hanging
theta_dot0    = 0.05;    % tiny nudge (rad/s)

% ---- Simulation ---------------------------------------------------------
t_end         = 25.0;    % total duration (s)
dt_sim        = 0.005;   % integration step (s)  — RK4 at 200 Hz
chk_interval  = 1000;    % checkpoint every N steps

% ---- Fuzzy system design ------------------------------------------------
mf_type    = 'triangular';   % 'triangular' | 'gaussian' | 'trapezoidal'
n_sets     = 7;              % 5 | 7 | 9
partition  = 'uniform';      % 'uniform' | 'concentrated'
inference  = 'sugeno';       % 'sugeno' | 'mamdani'
and_op     = 'min';          % 'min' | 'product'
defuzz     = 'weighted_avg'; % 'weighted_avg' | 'centroid' | 'mom'
type2      = false;          % logical — enable Type-2 FLS
fou_width  = 0.20;           % FOU half-width (Type-2 only)
e_range    = [-180, 180];    % universe of discourse (deg)

% ---- Rule base ----------------------------------------------------------
% 'expert' | 'aggressive' | 'conservative' | 'symmetric' | 'data_driven'
rule_preset = 'expert';

% ---- Swing-up strategy --------------------------------------------------
% 'energy' | 'backstepping' | 'sinusoidal' | 'fuzzy'
swingup_strategy = 'energy';

% Swing-up config (used by the selected strategy)
su_k_su      = 10.0;   % energy gain (energy strategy)
su_k1        = 4.0;    % level-1 gain (backstepping)
su_k2        = 10.0;   % level-2 gain (backstepping)
su_omega     = [];     % forcing frequency (sinusoidal, [] = 0.8*omega_n)
su_F_amp     = [];     % force amplitude (sinusoidal, [] = 0.85*Fmax)
su_capture_deg = 25;   % capture zone threshold (deg)
%   Suggested values: 15 (tight), 25 (default), 40 (relaxed)

% ---- Controllers to run -------------------------------------------------
% Include any subset of the following:
%   'FuzzyPID_Sugeno'   'FuzzyPID_Mamdani'   'FuzzyPID_Type2'
%   'FuzzySMC'          'AdaptiveFuzzy'
%   'LQR'               'ClassicalPID'
controllers = {'FuzzyPID_Sugeno', 'FuzzyPID_Mamdani', 'FuzzyPID_Type2', ...
               'FuzzySMC', 'AdaptiveFuzzy', 'LQR', 'ClassicalPID'};

% ---- LQR weights --------------------------------------------------------
% Q penalises [xc, xc_dot, theta, theta_dot]; R penalises control effort
% A 5th weight (lqr_qi) penalises the integral of xc error.
% Q5 = [0.1, 0.02, 200, 20, 0.1] verified working across all presets.
lqr_Q  = [0.1, 0.02, 200, 20];  % 4-state weights
lqr_qi = 0.1;                    % integral xc weight (5th state)
lqr_R  = 0.01;
xc_ref = 0.0;                    % desired cart position (m)

% ---- Classical PID gains ------------------------------------------------
pid_kp        = 80;    % proportional gain (N/deg)
pid_kd        = 15;    % derivative gain   (N*s/deg)
pid_ki        = 0.02;  % integral gain     (N/(deg*s))
pid_auto_tune = false; % override with Ziegler-Nichols estimates

% ---- Fuzzy SMC config ---------------------------------------------------
smc_lambda        = 5.0;   % sliding surface slope (rad/s per rad)
smc_phi_boundary  = 0.05;  % boundary layer thickness (rad)
smc_K_min         = 2.0;   % minimum switching gain (N)
smc_K_max         = 15.0;  % maximum switching gain (N)

% ---- Adaptive fuzzy config ----------------------------------------------
adapt_eta_kp  = 0.01;    % learning rate — kp
adapt_eta_kd  = 0.005;   % learning rate — kd
adapt_eta_ki  = 0.001;   % learning rate — ki
adapt_freeze  = false;   % freeze adaptation (use initial rules only)

% ---- Disturbances -------------------------------------------------------
kick_enabled      = false;
kick_time         = 8.0;    % s
kick_mag          = 5.0;    % N (impulse magnitude)
kick_duration     = 0.1;    % s (Gaussian half-width)

wind_enabled      = false;
wind_start        = 10.0;   % s
wind_ramp         = 0.5;    % s
wind_hold         = 3.0;    % s
wind_speed        = 5.0;    % m/s (peak gust speed)

friction_enabled  = false;
friction_start    = 5.0;
friction_end      = 7.0;
friction_mag      = 2.0;    % N (Coulomb component)
friction_visc     = 0.5;    % N*s/m (viscous component)

mass_enabled      = false;
mass_time         = 12.0;   % s — when mass is added
mass_delta        = 0.1;    % kg — additional mass on bob

tilt_enabled      = false;
tilt_angle_deg    = 2.0;    % track inclination angle (deg)
tilt_phase        = 0;      % phase offset of bias (rad)

% ---- Export options -----------------------------------------------------
gif_fps       = 10;
gif_max_frames= 400;    % cap: gif_frames = min(gif_fps*t_end, gif_max_frames)
make_gifs     = true;
dpi           = 150;
run_sensitivity = false;   % run sensitivity sweep (adds ~2-5 min)
run_rule_designer = false; % run data-driven rule suggestion

% =========================================================================
%  BUILD CONFIG STRUCT
% =========================================================================
fprintf('##########################################################\n');
fprintf('##   Fuzzy Inverted Pendulum on Circular Track         ##\n');
fprintf('##   Batch Run                                         ##\n');
fprintf('##########################################################\n\n');

t_total = tic;

% Physical params
p = pendulum_params(scale_preset);
if strcmp(scale_preset,'custom')
    p.mc = custom_mc; p.m = custom_m; p.l = custom_l;
    p.omega_n  = sqrt(p.g/p.l);
    p.E_target = p.m*p.g*p.l;
    M11_ = p.mc+p.m; M12_ = p.m*p.l; M22_ = p.m*p.l^2;
    detM_ = M11_*M22_ - M12_^2;
    p.M11_eq = M11_; p.M12_eq = M12_; p.M22_eq = M22_;
    p.Minv_eq = [M22_,-M12_;-M12_,M11_]/detM_;
end
p.capture_deg = su_capture_deg;
p.capture_vel = 2.0;

% Initial state
x0_run = [xc0_m; xc_dot0; deg2rad(theta0_deg); theta_dot0];

% Fuzzy config
fcfg = struct('n_sets',n_sets,'range',e_range,'mf_type',mf_type, ...
              'partition',partition,'inference',inference,'and_op',and_op, ...
              'defuzz',defuzz,'type2',type2,'fou_width',fou_width);

% Rule base
rb = fp_rule_base(rule_preset, n_sets);

% Run data-driven rule designer if requested
if run_rule_designer
    fprintf('>> Running rule designer...\n');
    gcfg_tmp = struct('kp_max',300,'kd_max',50,'ki_max',0.1, ...
                      'kp_min',0,'kd_min',0,'ki_min',0,'use_lpf',false);
    cfg_tmp  = struct('dt',dt_sim,'t_end',t_end,'x0',x0_run);
    [rb_suggested, rd_report] = fp_rule_designer(p, fcfg, gcfg_tmp, [], ...
        struct('grid_n',7,'horizon',150,'verbose',false));
    fprintf('  Suggested rules generated. Using data-driven rule base.\n');
    rb = rb_suggested;
end

% ---- Governor config (outer position loop) ------------------------------
% Gains are auto-computed from K5 in fp_run_analysis.
% Only override here if you want to manually tune them.
% max_tilt: auto-set to 15 for Small (Fmax<=3), 10 for Medium/Large.
govcfg = struct();   % empty = use auto-computed K5-derived values

% ---- Gain map config ---------------------------------------------------
gcfg = struct('kp_max',300,'kd_max',50,'ki_max',0.1, ...
              'kp_min',0,'kd_min',0,'ki_min',0,'use_lpf',false);

% Swing-up config
sucfg = struct('k_su',su_k_su,'k1',su_k1,'k2',su_k2,'E_thresh',0.05);
if isempty(su_omega),  sucfg.omega = 0.8*p.omega_n; else, sucfg.omega = su_omega; end
if isempty(su_F_amp),  sucfg.F_amp = 0.85*p.Fmax;  else, sucfg.F_amp = su_F_amp;  end
sucfg.phase_aware = true;

% PID config
pidcfg = struct('kp',pid_kp,'kd',pid_kd,'ki',pid_ki, ...
                'auto_tune',pid_auto_tune,'i_clamp',500);

% SMC config
smccfg = struct('lambda',smc_lambda,'phi_boundary',smc_phi_boundary, ...
                'K_min',smc_K_min,'K_max',smc_K_max);

% Adaptive fuzzy config
adapt_cfg = struct('eta_kp',adapt_eta_kp,'eta_kd',adapt_eta_kd, ...
                   'eta_ki',adapt_eta_ki,'freeze',adapt_freeze, ...
                   'kp_bounds',[0 500],'kd_bounds',[0 100],'ki_bounds',[0 0.5]);

% Disturbance config
disturbance = struct( ...
    'kick_enabled',kick_enabled,'kick_time',kick_time, ...
    'kick_mag',kick_mag,'kick_duration',kick_duration, ...
    'wind_enabled',wind_enabled,'wind_start',wind_start, ...
    'wind_ramp',wind_ramp,'wind_hold',wind_hold,'wind_speed',wind_speed, ...
    'friction_enabled',friction_enabled,'friction_start',friction_start, ...
    'friction_end',friction_end,'friction_mag',friction_mag, ...
    'friction_visc',friction_visc, ...
    'mass_enabled',mass_enabled,'mass_time',mass_time,'mass_delta',mass_delta, ...
    'tilt_enabled',tilt_enabled,'tilt_angle_deg',tilt_angle_deg, ...
    'tilt_phase',tilt_phase);

% =========================================================================
%  CREATE RUN FOLDER
% =========================================================================
existing = dir('Pendulum_Control_Run_*');
existing = existing([existing.isdir]);
if isempty(existing)
    run_num = 1;
else
    nums = zeros(1,numel(existing));
    for ii = 1:numel(existing)
        tok = regexp(existing(ii).name,'Pendulum_Control_Run_(\d+)','tokens');
        if ~isempty(tok), nums(ii) = str2double(tok{1}{1}); end
    end
    run_num = max(nums) + 1;
end
run_dir = sprintf('Pendulum_Control_Run_%03d', run_num);
mkdir(run_dir);
mkdir(fullfile(run_dir,'figures'));
mkdir(fullfile(run_dir,'animation'));
fprintf('Run folder: %s\n\n', run_dir);

% =========================================================================
%  ASSEMBLE AND RUN
% =========================================================================
cfg = struct();
p.xc_ref         = xc_ref;    % pass desired cart position into params
cfg.p            = p;
cfg.fcfg         = fcfg;
cfg.rule_base    = rb;
cfg.swingup      = swingup_strategy;
cfg.sucfg        = sucfg;
cfg.controllers  = controllers;
cfg.gcfg         = gcfg;
cfg.govcfg       = govcfg;
cfg.lqr_Q        = lqr_Q;
cfg.lqr_qi       = lqr_qi;
cfg.lqr_R        = lqr_R;
cfg.pidcfg       = pidcfg;
cfg.smccfg       = smccfg;
cfg.adapt_cfg    = adapt_cfg;
cfg.disturbance  = disturbance;
cfg.t_end        = t_end;
cfg.dt           = dt_sim;
cfg.x0           = x0_run;
cfg.run_dir      = run_dir;
cfg.chk_interval = chk_interval;

fprintf('>>> Simulation...\n');
results = fp_run_analysis(cfg);

fprintf('>>> Exporting figures...\n');
eopts = struct('dpi',dpi,'make_gifs',make_gifs,'gif_fps',gif_fps, ...
               'gif_max_frames',gif_max_frames);
fp_export_figures(run_dir, results, cfg, eopts);

fprintf('>>> Writing summary...\n');
fp_write_summary(run_dir, run_num, results, cfg);
save(fullfile(run_dir,'results.mat'), 'results', 'cfg', '-v7.3');

% =========================================================================
%  OPTIONAL: SENSITIVITY SWEEP
% =========================================================================
if run_sensitivity
    fprintf('>>> Sensitivity sweep...\n');
    sens_opts = struct('run_mf_sweep',true,'run_rule_sweep',true, ...
                       'run_param_sweep',true,'n_param_pts',8,'verbose',false);
    sens_results = fp_sensitivity(p, fcfg, rb, cfg, sens_opts);
    save(fullfile(run_dir,'sensitivity.mat'), 'sens_results', '-v7.3');
    fprintf('  Sensitivity results saved to %s/sensitivity.mat\n', run_dir);
end

% =========================================================================
%  DONE
% =========================================================================
elapsed = toc(t_total);
fprintf('\n##########################################################\n');
fprintf('##  Run complete  (%.1f s total)\n', elapsed);
fprintf('##  Results: %s\n', run_dir);
fprintf('##\n');
fprintf('##  Next steps:\n');
fprintf('##    >> animate_pendulum        (view animation)\n');
fprintf('##    >> fp_build_report         (update HTML report)\n');
fprintf('##########################################################\n\n');
