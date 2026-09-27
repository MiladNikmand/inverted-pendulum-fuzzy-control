function results = fp_run_analysis(cfg)
% FP_RUN_ANALYSIS
% Main simulation engine for the fuzzy inverted pendulum project.
%
% Orchestrates: swing-up -> stabilisation -> metrics -> Lyapunov analysis
% for each selected controller. Supports crash-safe checkpointing and
% resume. Called by both pendulum_launch (GUI) and run_all (batch).
%
% Each controller run follows the same structure:
%   Phase 1 (SWING-UP):   selected strategy drives pendulum from hanging
%                          to the capture zone (|theta| < capture_deg)
%   Phase 2 (STABILISE):  selected stabilising controller holds upright
%
% The simulation uses fixed-step RK4 integration for accuracy.
%
% Inputs:
%   cfg — configuration struct built by pendulum_launch or run_all:
%     .p               pendulum_params struct
%     .fcfg            fuzzy config (mf_type, n_sets, inference, etc.)
%     .rule_base       rule base struct from fp_rule_base or fp_rule_designer
%     .swingup         swing-up strategy name: 'energy'|'backstepping'|'sinusoidal'|'fuzzy'
%     .sucfg           swing-up strategy config
%     .controllers     cell array of controller names to run
%     .gcfg            gain map config
%     .lqr_Q           LQR Q matrix diagonal [4 values]
%     .lqr_R           LQR R scalar
%     .pidcfg          classical PID config
%     .smccfg          fuzzy SMC config
%     .adapt_cfg       adaptive fuzzy config
%     .disturbance     disturbance config (dcfg for fp_disturbance)
%     .t_end           simulation duration (s)
%     .dt              integration step (s)
%     .x0              initial state [phi0; phi_dot0; theta0; theta_dot0]
%     .run_dir         output folder path (for checkpointing)
%     .chk_interval    checkpoint every N steps (default 500)
%
% Outputs:
%   results — struct with fields:
%     .runs      struct, one field per controller
%     .p         pendulum_params used
%     .cfg       config used
%     .fcfg      fuzzy config used
%     .rule_base rule base used

% ---- Optional: quiet mode for optimiser calls --------------------------
quiet = isfield(cfg,'quiet') && cfg.quiet;

if ~quiet
    fprintf('\n[fp_run_analysis] Starting simulation\n');
    fprintf('  Swing-up: %s  |  Controllers: %s\n', ...
            cfg.swingup, strjoin(cfg.controllers, ', '));
    fprintf('  t_end=%.1fs  dt=%.4fs  (%d steps)\n', ...
            cfg.t_end, cfg.dt, round(cfg.t_end/cfg.dt));
end

p       = cfg.p;
fcfg    = cfg.fcfg;
rb      = cfg.rule_base;
dt      = cfg.dt;
N       = round(cfg.t_end / cfg.dt) + 1;
t_vec   = linspace(0, cfg.t_end, N);
x0      = cfg.x0(:);
cap_rad = deg2rad(p.capture_deg);
cap_vel = p.capture_vel;

if ~isfield(cfg,'chk_interval'), cfg.chk_interval = 500; end

% Ensure xc position correction fields exist (default = off)
if ~isfield(cfg.gcfg,'kp_xc'),   cfg.gcfg.kp_xc   = 0; end
if ~isfield(cfg.gcfg,'kd_xc'),   cfg.gcfg.kd_xc   = 0; end
if ~isfield(cfg.pidcfg,'kp_xc'), cfg.pidcfg.kp_xc = 0; end
if ~isfield(cfg.pidcfg,'kd_xc'), cfg.pidcfg.kd_xc = 0; end
if ~isfield(cfg.smccfg,'kp_xc'), cfg.smccfg.kp_xc = 0; end
if ~isfield(cfg.smccfg,'kd_xc'), cfg.smccfg.kd_xc = 0; end
if ~isfield(cfg,'govcfg'),        cfg.govcfg        = struct(); end
p.lqr_xi          = 0;
p.lqr_xi_prev_err = 0;
p.fuzzy_xi         = 0;
p.theta_ref        = 0;

% ---- Pre-compute LQR gain (once, shared across runs) -------------------
if ~quiet, fprintf('  Linearising system...\n'); end
[A_lin, B_lin, ~, ~, lin_info] = pendulum_linearise(p, quiet);
if ~quiet, fprintf('  %s\n', lin_info.note); end

K_lqr = zeros(1,5);   % 5-state augmented (4 state + 1 integral)
if any(strcmp(cfg.controllers,'LQR'))
    q = cfg.lqr_Q(:);
    if numel(q) ~= 4
        warning('lqr_Q has %d elements, expected 4. Padding/trimming to 4.', numel(q));
        q4 = zeros(4,1); q4(1:min(4,numel(q))) = q(1:min(4,numel(q)));
        q = q4;
    end
    % Augmented Q: add integral weight as 5th diagonal element
    % Default integral weight = q(1) (same as xc weight)
    qi = q(1);
    if isfield(cfg,'lqr_qi'), qi = cfg.lqr_qi; end
    Q5_diag = [q(:); qi];
    R_lqr   = cfg.lqr_R;
    try
        Aa = lin_info.Aa;
        Ba = lin_info.Ba;
        [~,~,K5] = care(Aa, Ba, diag(Q5_diag), R_lqr);
        K_lqr = K5(:)';
        if ~quiet
            fprintf('  LQR gains (5-state): [%.2f %.2f %.2f %.2f %.2f]\n', K_lqr);
        end
    catch ME
        warning('LQR care() failed: %s. Using zero gains.', ME.message);
    end
end

% ---- Governor gains (computed from augmented LQR K5) -------------------
% Always computed regardless of whether LQR is in controllers list,
% because all fuzzy controllers use the governor.
govcfg = struct();
if isfield(cfg,'govcfg'), govcfg = cfg.govcfg; end

% Compute K5 for governor gain derivation (use same Q5 as LQR or default)
q_gov = [0.1, 0.02, 200, 20];
if isfield(cfg,'lqr_Q') && numel(cfg.lqr_Q)==4, q_gov = cfg.lqr_Q(:)'; end
qi_gov = q_gov(1);
if isfield(cfg,'lqr_qi'), qi_gov = cfg.lqr_qi; end
try
    Aa_gov = lin_info.Aa; Ba_gov = lin_info.Ba;
    [~,~,K5_gov] = care(Aa_gov, Ba_gov, diag([q_gov(:);qi_gov]), cfg.lqr_R);
    K5_gov = K5_gov(:)';
    % Governor gains: ratio of xc/theta gains from LQR (converted to deg)
    if ~isfield(govcfg,'kp_gov')
        govcfg.kp_gov = abs(K5_gov(1)/K5_gov(3)) * (180/pi);
    end
    if ~isfield(govcfg,'kd_gov')
        govcfg.kd_gov = abs(K5_gov(2)/K5_gov(3)) * (180/pi);
    end
    % Inner loop base gains from K5 (exact, preset-specific).
    % govcfg.inner_scale (default 1) multiplies both, letting the
    % optimiser trade inner-loop stiffness against governor authority
    % without having to know the preset-specific absolute values.
    inner_scale = 1.0;
    if isfield(govcfg,'inner_scale'), inner_scale = govcfg.inner_scale; end
    if ~isfield(govcfg,'kp_inner')
        govcfg.kp_inner = abs(K5_gov(3)) * (pi/180) * inner_scale;
    end
    if ~isfield(govcfg,'kd_inner')
        govcfg.kd_inner = abs(K5_gov(4)) * (pi/180) * inner_scale;
    end
    if ~quiet
        fprintf('  Governor gains: kp_gov=%.3f deg/m  kd_gov=%.3f deg/(m/s)\n', ...
                govcfg.kp_gov, govcfg.kd_gov);
        fprintf('  Inner gains:    kp_inner=%.4f N/deg  kd_inner=%.4f N*s/deg', ...
                govcfg.kp_inner, govcfg.kd_inner);
        if inner_scale ~= 1.0
            fprintf('  (scale %.2f)', inner_scale);
        end
        fprintf('\n');
    end
catch
    % Fallback: use Minv-based defaults (will be used by ctrl_xc_governor)
    if ~quiet, fprintf('  Governor: using Minv-based defaults\n'); end
end

% max_tilt: larger for Small preset (low Fmax needs more tilt authority)
if ~isfield(govcfg,'max_tilt')
    if p.Fmax <= 3.0
        govcfg.max_tilt = 15.0;   % Small preset
    else
        govcfg.max_tilt = 10.0;   % Medium and Large
    end
end

% Store back so controllers can read it
p.govcfg = govcfg;

results.p          = p;
results.cfg        = cfg;
results.fcfg       = fcfg;
results.rule_base  = rb;
results.lin_info   = lin_info;
results.K_lqr      = K_lqr;
results.t_vec      = t_vec;
results.runs       = struct();

% =========================================================================
%  RUN EACH SELECTED CONTROLLER
% =========================================================================
for ci = 1:numel(cfg.controllers)
    ctrl_name = cfg.controllers{ci};
    if ~quiet, fprintf('\n  --- Controller: %s ---\n', ctrl_name); end

    % ---- Allocate storage ----------------------------------------------
    x_hist    = zeros(N, 4);
    F_hist    = zeros(N, 1);
    kp_hist   = zeros(N, 1);
    kd_hist   = zeros(N, 1);
    ki_hist   = zeros(N, 1);
    w_hist    = zeros(N, p.capture_deg);
    E_hist    = zeros(N, 1);
    phase_hist      = zeros(N, 1);
    theta_ref_hist  = zeros(N, 1);   % governor reference per step
    rb_hist   = rb;

    x_hist(1,:) = x0';

    % ---- Simulation loop variables -------------------------------------
    theta_stable_timer = 0.0;
    xc_int_active      = false;
    theta_stable_band  = deg2rad(5.0);
    theta_stable_req   = 1.5;

    % ---- Check for checkpoint ------------------------------------------
    k_start  = 1;
    captured = false;
    int_e    = 0;

    % Load checkpoint first to check if it matches THIS controller.
    % If it belongs to a different controller (stale from previous loop
    % iteration) discard it immediately so the warning is suppressed.
    [chk_found, chk_state] = fp_checkpoint('check', cfg.run_dir);
    if chk_found && (~isfield(chk_state,'ctrl_name') || ...
                     ~strcmp(chk_state.ctrl_name, ctrl_name))
        fp_checkpoint('clear', cfg.run_dir);
        chk_found = false;
    end
    if chk_found && isfield(chk_state,'ctrl_name') && ...
       strcmp(chk_state.ctrl_name, ctrl_name)
        k_start   = chk_state.k_resume;
        x_hist(1:k_start,:) = chk_state.x_hist_partial;
        F_hist(1:k_start)   = chk_state.F_hist_partial;
        captured  = chk_state.captured;
        int_e     = chk_state.int_e;
        if isfield(chk_state,'rb_hist'), rb_hist = chk_state.rb_hist; end
        if ~quiet
            fprintf('    Resumed from checkpoint at step %d (t=%.2fs)\n', ...
                    k_start, t_vec(k_start));
        end
    end

    % ---- Warm-start from settled state (disturbance mode) --------------
    % Applied AFTER the checkpoint block so it is not overwritten by it.
    % cfg.warm_start holds per-controller settled states saved from a
    % previous normal run. The simulation begins already balanced, with
    % the stability timer past its threshold so the governor is active
    % from step 1. This isolates disturbance rejection from swing-up.
    warm_started = false;
    if isfield(cfg,'warm_start') && isstruct(cfg.warm_start) && ...
       isfield(cfg.warm_start, ctrl_name)
        ws = cfg.warm_start.(ctrl_name);
        if isfield(ws,'x0') && isfield(ws,'t_start')
            k_start      = 1;
            x_hist(1,:)  = ws.x0(:)';
            captured     = true;
            int_e        = 0;
            warm_started = true;
            theta_stable_timer = theta_stable_req + 1;
            xc_int_active      = true;
            if isfield(ws,'lqr_xi'),   p.lqr_xi   = ws.lqr_xi;   end
            if isfield(ws,'fuzzy_xi'), p.fuzzy_xi = ws.fuzzy_xi; end
            if ~quiet
                fprintf(['    Warm-start: theta=%.2f deg  xc=%.3f m  ' ...
                         '(from t=%.2fs of baseline)\n'], ...
                        rad2deg(ws.x0(3)), ws.x0(1), ws.t_start);
            end
        end
    end

    for k = k_start:N-1
        t_k  = t_vec(k);
        xk   = x_hist(k,:)';
        theta = xk(3);

        % ---- Phase detection: swing-up or stabilise --------------------
        if ~captured
            near_upright = abs(theta) < cap_rad && abs(xk(4)) < cap_vel;
            if near_upright
                captured = true;
                p.lqr_xi          = 0;
                p.lqr_xi_prev_err = 0;
                p.fuzzy_xi        = 0;
                p.theta_ref       = 0;
                if ~quiet
                    fprintf('    Captured at t=%.3f s (theta=%.2f deg)\n', ...
                            t_k, rad2deg(theta));
                end
            end
        end
        phase_hist(k) = double(captured);

        % ---- Disturbance -----------------------------------------------
        dcfg_k = cfg.disturbance;

        % ---- Controller selection --------------------------------------
        kp_k = 0; kd_k = 0; ki_k = 0;

        if ~captured
            % SWING-UP PHASE
            switch cfg.swingup
                case 'energy'
                    [F_k, su_info] = swingup_energy(xk, cfg.sucfg, p);
                case 'backstepping'
                    [F_k, su_info] = swingup_backstepping(xk, cfg.sucfg, p);
                case 'sinusoidal'
                    cfg.sucfg.t    = t_k;
                    [F_k, su_info] = swingup_sinusoidal(xk, cfg.sucfg, p);
                case 'fuzzy'
                    [F_k, su_info] = ctrl_fuzzy_swingup(xk, rb, fcfg, p);
                otherwise
                    [F_k, su_info] = swingup_energy(xk, cfg.sucfg, p);
            end

        else
            % STABILISATION PHASE
            int_e = int_e + deg2rad(theta) * dt;  % theta integral

            % ---- Stability timer ----------------------------------------
            % Governor only activates once theta has been within
            % theta_stable_band for theta_stable_req seconds continuously.
            xc_ref_k = 0;
            if isfield(p,'xc_ref'), xc_ref_k = p.xc_ref; end

            if abs(theta) < theta_stable_band
                theta_stable_timer = theta_stable_timer + dt;
            else
                theta_stable_timer = 0.0;
                xc_int_active      = false;
            end
            if theta_stable_timer >= theta_stable_req
                xc_int_active = true;
            end

            % ---- Reference governor ------------------------------------
            % Computes theta_ref when theta is stable.
            % theta_ref = 0 during transient (pure balancing).
            p.theta_ref = 0.0;
            if xc_int_active
                [theta_ref_k, ~] = ctrl_xc_governor(xk, p.govcfg, p);
                p.theta_ref = theta_ref_k;
            end

            switch ctrl_name
                case 'FuzzyPID_Sugeno'
                    fcfg_s = fcfg; fcfg_s.inference = 'sugeno';
                    [F_k, c_info] = ctrl_fuzzy_pid(xk, int_e, cfg.gcfg, rb, fcfg_s, p);
                    kp_k = c_info.kp; kd_k = c_info.kd; ki_k = c_info.ki;

                case 'FuzzyPID_Mamdani'
                    fcfg_m = fcfg; fcfg_m.inference = 'mamdani';
                    [F_k, c_info] = ctrl_fuzzy_pid(xk, int_e, cfg.gcfg, rb, fcfg_m, p);
                    kp_k = c_info.kp; kd_k = c_info.kd; ki_k = c_info.ki;

                case 'FuzzyPID_Type2'
                    [F_k, c_info] = ctrl_type2_fuzzy_pid(xk, int_e, cfg.gcfg, rb, fcfg, p);
                    kp_k = c_info.kp; kd_k = c_info.kd; ki_k = c_info.ki;

                case 'FuzzySMC'
                    [F_k, c_info] = ctrl_fuzzy_smc(xk, cfg.smccfg, rb, fcfg, p);

                case 'AdaptiveFuzzy'
                    [F_k, rb_hist, c_info] = ctrl_adaptive_fuzzy( ...
                        xk, int_e, rb_hist, fcfg, cfg.gcfg, p, cfg.adapt_cfg);
                    kp_k = c_info.kp; kd_k = c_info.kd; ki_k = c_info.ki;

                case 'LQR'
                    % Integral of xc error — gated by stability timer
                    % Anti-windup: three protections against cart rotation:
                    %   1. Only accumulate when theta is stable (gate already set)
                    %   2. Sign-change reset: clear when xc crosses xc_ref
                    %   3. Conditional: only accumulate when |xc_err| < 0.5m
                    %      (proportional term handles large errors; integral
                    %       handles residual near the target only)
                    if xc_int_active
                        xc_err_k = xk(1) - xc_ref_k;
                        % Sign-change reset
                        if isfield(p,'lqr_xi_prev_err') && ...
                           sign(xc_err_k) ~= sign(p.lqr_xi_prev_err) && ...
                           abs(p.lqr_xi_prev_err) > 0.01
                            p.lqr_xi = 0;
                        end
                        p.lqr_xi_prev_err = xc_err_k;
                        % Conditional integration (near target only)
                        if abs(xc_err_k) < 0.5
                            p.lqr_xi = p.lqr_xi + xc_err_k * dt;
                            p.lqr_xi = max(-20, min(20, p.lqr_xi));
                        end
                    end
                    [F_k, c_info] = ctrl_lqr(xk, K_lqr, p);

                case 'ClassicalPID'
                    [F_k, int_e, c_info] = ctrl_classical_pid(xk, int_e, cfg.pidcfg, p);
                    kp_k = c_info.kp; kd_k = c_info.kd; ki_k = c_info.ki;

                otherwise
                    warning('Unknown controller: %s', ctrl_name);
                    F_k = 0;
            end
        end

        % ---- Record ----------------------------------------------------
        F_hist(k)           = F_k;
        kp_hist(k)          = kp_k;
        kd_hist(k)          = kd_k;
        ki_hist(k)          = ki_k;
        E_hist(k)           = 0.5*p.m*p.l^2*xk(4)^2 + p.m*p.g*p.l*cos(xk(3));
        theta_ref_hist(k)   = p.theta_ref;

        % ---- Integrate (RK4) ------------------------------------------
        dcfg_now = dcfg_k;
        k1 = pendulum_dynamics(t_k,        xk,               F_k, p, dcfg_now);
        k2 = pendulum_dynamics(t_k+dt/2,   xk + dt/2*k1,    F_k, p, dcfg_now);
        k3 = pendulum_dynamics(t_k+dt/2,   xk + dt/2*k2,    F_k, p, dcfg_now);
        k4 = pendulum_dynamics(t_k+dt,     xk + dt*k3,      F_k, p, dcfg_now);
        x_next = xk + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);

        % Wrap theta to [-pi, pi]
        x_next(3) = atan2(sin(x_next(3)), cos(x_next(3)));

        x_hist(k+1,:) = x_next';

        % ---- Checkpoint ------------------------------------------------
        if mod(k, cfg.chk_interval) == 0
            chk_state_save = struct( ...
                'ctrl_name',      ctrl_name, ...
                'k_resume',       k+1, ...
                't_resume',       t_vec(k+1), ...
                'x_hist_partial', x_hist(1:k+1,:), ...
                'F_hist_partial', F_hist(1:k+1), ...
                'captured',       captured, ...
                'int_e',          int_e, ...
                'rb_hist',        rb_hist, ...
                'cfg',            cfg);
            fp_checkpoint('save', cfg.run_dir, chk_state_save);
        end
    end % simulation loop

    F_hist(N)  = F_hist(N-1);
    E_hist(N)  = 0.5*p.m*p.l^2*x_hist(N,4)^2 + p.m*p.g*p.l*cos(x_hist(N,3));

    % ---- Compute metrics -----------------------------------------------
    metrics = fp_metrics(t_vec(:), x_hist, F_hist, p, cfg);

    % ---- Compute Lyapunov values ---------------------------------------
    phase_info = struct('t_capture', metrics.t_capture, 'A_cl', []);
    lyap = fp_lyapunov(t_vec(:), x_hist, F_hist, p, phase_info);

    % ---- Store run results ---------------------------------------------
    run_struct = struct();
    run_struct.ctrl_name       = ctrl_name;
    run_struct.swingup         = cfg.swingup;
    run_struct.t               = t_vec(:);
    run_struct.x               = x_hist;
    run_struct.F               = F_hist;
    run_struct.F_hist          = F_hist;    % alias for animate_pendulum
    run_struct.kp              = kp_hist;
    run_struct.kd              = kd_hist;
    run_struct.ki              = ki_hist;
    run_struct.E               = E_hist;
    run_struct.phase           = phase_hist;
    run_struct.theta_ref_hist  = theta_ref_hist;
    run_struct.metrics     = metrics;
    run_struct.lyapunov    = lyap;
    run_struct.captured    = captured;
    if strcmp(ctrl_name,'AdaptiveFuzzy')
        run_struct.rb_final = rb_hist;
    end

    results.runs.(ctrl_name) = run_struct;

    % ---- Console summary -----------------------------------------------
    if ~quiet
        fprintf('    Result: captured=%d', captured);
        if ~isnan(metrics.t_capture)
            fprintf('  t_capture=%.2fs', metrics.t_capture);
        end
        if ~isnan(metrics.t_settle)
            fprintf('  t_settle=%.2fs', metrics.t_settle);
        end
        if ~isnan(metrics.overshoot_deg)
            fprintf('  overshoot=%.1fdeg', metrics.overshoot_deg);
        end
        fprintf('  effort=%.1fN*s\n', metrics.total_effort);
        fprintf('    Lyapunov: %s\n', lyap.assessment);
    end

end % controller loop

if ~quiet, fprintf('\n[fp_run_analysis] All controllers done.\n\n'); end

% Clear checkpoint — run completed successfully
fp_checkpoint('clear', cfg.run_dir);

end
