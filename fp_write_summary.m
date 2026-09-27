function fp_write_summary(run_dir, run_num, results, cfg)
% FP_WRITE_SUMMARY
% Writes run_summary.json, README.txt, and prints a console summary.
% Called by both pendulum_launch and run_all after fp_run_analysis.

% ---- Console output ----------------------------------------------------
fprintf('\n==========================================================\n');
fprintf('  RUN SUMMARY  —  Pendulum_Control_Run_%03d\n', run_num);
fprintf('==========================================================\n');
fprintf('  Preset:   %s\n', cfg.p.name);
fprintf('  Fmax:     %.1f N  |  l=%.3f m  |  mc=%.3f kg  |  m=%.3f kg\n', ...
        cfg.p.Fmax, cfg.p.l, cfg.p.mc, cfg.p.m);
fprintf('  Swing-up: %s\n', cfg.swingup);
fprintf('  Fuzzy:    %d sets / %s MF / %s inference\n', ...
        cfg.fcfg.n_sets, cfg.fcfg.mf_type, cfg.fcfg.inference);
if cfg.fcfg.type2
    fprintf('            Type-2 FOU width = %.2f\n', cfg.fcfg.fou_width);
end
fprintf('\n');
fprintf('  %-20s  %-10s  %-10s  %-12s  %-12s  %-12s  %s\n', ...
        'Controller','Captured','t_capture','t_settle','Overshoot','xc_err(m)','Effort(N*s)');
fprintf('  %-20s  %-10s  %-10s  %-12s  %-12s  %-12s  %s\n', ...
        '--------------------','----------','----------', ...
        '------------','------------','------------','----------');

keys = fieldnames(results.runs);
for ki = 1:numel(keys)
    k   = keys{ki};
    r   = results.runs.(k);
    m   = r.metrics;
    cap_s  = bool2str(r.captured);
    tc_s   = fmt_val(m.t_capture,      '%.2f s');
    ts_s   = fmt_val(m.t_settle,       '%.2f s');
    ov_s   = fmt_val(m.overshoot_deg,  '%.1f deg');
    xce_s  = fmt_val(m.xc_final_error, '%.3f');
    eff_s  = fmt_val(m.total_effort,   '%.1f');
    fprintf('  %-20s  %-10s  %-10s  %-12s  %-12s  %-12s  %s\n', ...
            k, cap_s, tc_s, ts_s, ov_s, xce_s, eff_s);
end
fprintf('==========================================================\n\n');

% ---- Write README.txt --------------------------------------------------
fid = fopen(fullfile(run_dir, 'README.txt'), 'w');
fprintf(fid, 'Fuzzy Inverted Pendulum on Cart -- Run %03d\n', run_num);
fprintf(fid, 'Generated: %s\n\n', ...
        char(datetime('now','Format','yyyy-MM-dd HH:mm:ss')));

fprintf(fid, 'PHYSICAL PARAMETERS\n');
fprintf(fid, '  Preset:              %s\n',      cfg.p.name);
fprintf(fid, '  Cart mass mc:        %.3f kg\n', cfg.p.mc);
fprintf(fid, '  Bob mass m:          %.3f kg\n', cfg.p.m);
fprintf(fid, '  Rod length l:        %.3f m\n',  cfg.p.l);
fprintf(fid, '  Max force Fmax:      %.1f N\n',  cfg.p.Fmax);
fprintf(fid, '  Natural frequency:   %.3f rad/s\n', cfg.p.omega_n);
fprintf(fid, '  Energy target:       %.4f J\n',  cfg.p.E_target);
fprintf(fid, '\nINITIAL CONDITIONS\n');
fprintf(fid, '  xc0:         %.4f m\n',    cfg.x0(1));
fprintf(fid, '  xc_dot0:     %.4f m/s\n',  cfg.x0(2));
fprintf(fid, '  theta0:      %.2f deg\n',  rad2deg(cfg.x0(3)));
fprintf(fid, '  theta_dot0:  %.2f deg/s\n', rad2deg(cfg.x0(4)));

fprintf(fid, '\nFUZZY SYSTEM DESIGN\n');
fprintf(fid, '  Rule base preset:  %s\n', cfg.rule_base.preset);
fprintf(fid, '  N membership sets: %d\n', cfg.fcfg.n_sets);
fprintf(fid, '  MF type:           %s\n', cfg.fcfg.mf_type);
fprintf(fid, '  Partition:         %s\n', cfg.fcfg.partition);
fprintf(fid, '  Inference:         %s\n', cfg.fcfg.inference);
fprintf(fid, '  AND operator:      %s\n', cfg.fcfg.and_op);
fprintf(fid, '  Defuzzification:   %s\n', cfg.fcfg.defuzz);
fprintf(fid, '  Type-2 FLS:        %s\n', bool2str(cfg.fcfg.type2));
if cfg.fcfg.type2
    fprintf(fid, '  FOU width:         %.2f\n', cfg.fcfg.fou_width);
end

fprintf(fid, '\nSWING-UP STRATEGY\n');
fprintf(fid, '  Strategy:  %s\n', cfg.swingup);
fprintf(fid, '  Threshold: %.1f deg\n', cfg.p.capture_deg);

fprintf(fid, '\nCONTROLLER RESULTS\n');
for ki = 1:numel(keys)
    k  = keys{ki};
    r  = results.runs.(k);
    m  = r.metrics;
    fprintf(fid, '  [%s]\n', k);
    fprintf(fid, '    Captured:       %s\n', bool2str(r.captured));
    if ~isnan(m.t_capture)
        fprintf(fid, '    Capture time:   %.3f s\n', m.t_capture);
    end
    if ~isnan(m.t_settle)
        fprintf(fid, '    Settle time:    %.3f s\n', m.t_settle);
    end
    if ~isnan(m.overshoot_deg)
        fprintf(fid, '    Overshoot:      %.2f deg\n', m.overshoot_deg);
    end
    if ~isnan(m.rms_theta_deg)
        fprintf(fid, '    RMS theta:      %.3f deg\n', m.rms_theta_deg);
    end
    fprintf(fid, '    xc_ref:         %.3f m\n', m.xc_ref);
    fprintf(fid, '    xc_final:       %.3f m\n', m.xc_final);
    fprintf(fid, '    xc_error:       %.3f m\n', m.xc_final_error);
    if ~isnan(m.t_xc_settle)
        fprintf(fid, '    xc settle time: %.3f s\n', m.t_xc_settle);
    end
    fprintf(fid, '    xc settled:     %s\n', bool2str(m.xc_settled));
    fprintf(fid, '    Control effort: %.2f N*s\n', m.total_effort);
    fprintf(fid, '    Lyapunov:       %s\n', r.lyapunov.assessment);
    fprintf(fid, '\n');
end

% Disturbances
dcfg = cfg.disturbance;
active = {};
if isfield(dcfg,'kick_enabled')     && dcfg.kick_enabled,     active{end+1}='kick';     end
if isfield(dcfg,'wind_enabled')     && dcfg.wind_enabled,     active{end+1}='wind';     end
if isfield(dcfg,'friction_enabled') && dcfg.friction_enabled, active{end+1}='friction'; end
if isfield(dcfg,'mass_enabled')     && dcfg.mass_enabled,     active{end+1}='mass';     end
if isfield(dcfg,'tilt_enabled')     && dcfg.tilt_enabled,     active{end+1}='tilt';     end
if ~isempty(active)
    fprintf(fid, 'DISTURBANCES ACTIVE\n');
    fprintf(fid, '  %s\n', strjoin(active, ', '));
end

fclose(fid);

% ---- Write run_summary.json --------------------------------------------
S = struct();
S.schema     = 'fuzzy-pendulum-control/1.0';
S.run_number = run_num;
S.run_folder = sprintf('Pendulum_Control_Run_%03d', run_num);
S.generated  = char(datetime('now','Format','yyyy-MM-dd HH:mm:ss'));

S.params = struct('mc',cfg.p.mc,'m',cfg.p.m,'l',cfg.p.l, ...
                  'Fmax',cfg.p.Fmax,'g',cfg.p.g,'name',cfg.p.name);
S.fuzzy  = struct('n_sets',cfg.fcfg.n_sets,'mf_type',cfg.fcfg.mf_type, ...
                  'inference',cfg.fcfg.inference,'type2',cfg.fcfg.type2, ...
                  'rule_preset',cfg.rule_base.preset);
S.swingup = cfg.swingup;

ctrl_out = struct();
for ki = 1:numel(keys)
    k  = keys{ki};
    m  = results.runs.(k).metrics;
    ctrl_out.(k) = struct( ...
        'captured',        results.runs.(k).captured, ...
        't_capture',       m.t_capture, ...
        't_settle',        m.t_settle, ...
        'overshoot_deg',   m.overshoot_deg, ...
        'rms_theta_deg',   m.rms_theta_deg, ...
        'xc_ref',          m.xc_ref, ...
        'xc_final',        m.xc_final, ...
        'xc_final_error',  m.xc_final_error, ...
        'xc_rms',          m.xc_rms, ...
        't_xc_settle',     m.t_xc_settle, ...
        'xc_settled',      m.xc_settled, ...
        'total_effort',    m.total_effort, ...
        'rms_F',           m.rms_F, ...
        'F_sat_fraction',  m.F_sat_fraction);
end
S.controllers = ctrl_out;
S.disturbances = active;

fid = fopen(fullfile(run_dir,'run_summary.json'),'w');
fwrite(fid, jsonencode(S,'PrettyPrint',true), 'char');
fclose(fid);

fprintf('[fp_write_summary] Written to %s\n', run_dir);
end

function s = fmt_val(v, fmt)
    if isnan(v) || isempty(v), s = '—'; else, s = sprintf(fmt, v); end
end
function s = bool2str(b)
    if b, s = 'YES'; else, s = 'no'; end
end
