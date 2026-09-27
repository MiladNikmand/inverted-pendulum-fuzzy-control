% CHECK_SETUP  (Fuzzy Inverted Pendulum project)
% Verifies all required files are present before running.
%
% Usage:
%   check_setup          % prints pass/fail report
%   ok = check_setup()   % returns true if everything present

function ok = check_setup()

required = {
    % Core model
    'pendulum_params.m',            'Physical parameters and scale presets'
    'pendulum_dynamics.m',          'Cart-pendulum nonlinear ODE (4-state)'
    'pendulum_linearise.m',         'Linearisation + controllability analysis'
    % Fuzzy engine
    'fp_membership.m',              'MF evaluator (tri/gauss/trap, T1/T2)'
    'fp_inference.m',               'Sugeno/Mamdani inference engine'
    'fp_rule_base.m',               'Rule table presets (5/7/9 sets)'
    'fp_gain_map.m',                'Raw fuzzy output -> bounded PID gains'
    % Controllers
    'ctrl_fuzzy_pid.m',             'Type-1/2 Sugeno/Mamdani fuzzy PID'
    'ctrl_type2_fuzzy_pid.m',       'Type-2 fuzzy PID (Karnik-Mendel)'
    'ctrl_fuzzy_smc.m',             'Fuzzy sliding mode controller'
    'ctrl_xc_governor.m',           'XC reference governor (outer loop)'
    'ctrl_adaptive_fuzzy.m',        'Online gradient-descent rule adaptation'
    'ctrl_fuzzy_swingup.m',         'Fuzzy swing-up controller'
    'ctrl_lqr.m',                   'LQR baseline (linearised approx.)'
    'ctrl_classical_pid.m',         'Fixed-gain PID baseline'
    % Swing-up strategies
    'swingup_energy.m',             'Energy-based Lyapunov swing-up'
    'swingup_backstepping.m',       'Nonlinear backstepping swing-up'
    'swingup_sinusoidal.m',         'Bang-bang sinusoidal swing-up'
    % Simulation engine
    'fp_run_analysis.m',            'Main simulation engine (RK4, checkpoint)'
    'fp_disturbance.m',             'Five disturbance types'
    'fp_metrics.m',                 'Performance metric computation'
    'fp_lyapunov.m',                'Lyapunov V(x) and V_dot(x) analysis'
    'fp_checkpoint.m',              'Crash-safe save/resume'
    'fp_rule_designer.m',           'Data-driven rule suggestion tool'
    'fp_sensitivity.m',             'MF/rule/parameter sensitivity sweeps'
    % Export pipeline
    'fp_export_figures.m',          'PNG and GIF export'
    'fp_write_summary.m',           'Console + TXT + JSON output'
    'fp_build_report.m',            'Generates report_data.js'
    'animate_pendulum.m',           '3-panel animation viewer + GIF export'
    % Entry points
    'pendulum_launcher.m',          'GUI entry point (6-tab uifigure)'
    'run_all.m',                    'Non-interactive batch driver'
    'run_disturbance.m',            'Disturbance suite (warm-start from baseline)'
    % Optimisation (optional layer)
    'opt_cost.m',                   'Scalar cost function for gain tuning'
    'opt_run.m',                    'Nelder-Mead driver with LQR warm start'
    'opt_visualise.m',              'Convergence, Pareto front, gain comparison'
    'opt_apply.m',                  'Writes optimised_config.m'
};

n  = size(required,1);
ok = true;
w  = 36;

fprintf('\n');
fprintf('  Fuzzy Inverted Pendulum — Setup Check\n');
fprintf('  %-*s  %s\n', w, 'File', 'Status');
fprintf('  %-*s  %s\n', w, repmat('-',1,w), '--------');

missing = {};
for i = 1:n
    fname  = required{i,1};
    desc   = required{i,2};
    exists = isfile(fname);
    if exists
        tag = 'OK';
    else
        tag = 'MISSING';
        ok  = false;
        missing{end+1} = fname; %#ok<AGROW>
    end
    fprintf('  %-*s  [%s]  %s\n', w, fname, tag, desc);
end

fprintf('\n');
if ok
    fprintf('  All %d files present. Ready to run.\n', n);
    fprintf('  >> pendulum_launcher    (interactive GUI — does everything)\n');
    fprintf('  >> run_all              (silent batch baseline)\n');
    fprintf('  >> run_disturbance      (after a baseline run)\n');
else
    fprintf('  %d file(s) missing:\n', numel(missing));
    for i = 1:numel(missing)
        fprintf('    - %s\n', missing{i});
    end
    fprintf('\n  Make sure you are in the project root folder:\n');
    fprintf('  >> cd path/to/inverted-pendulum-fuzzy-control\n');
end
fprintf('\n');

if nargout == 0, clear ok; end
end