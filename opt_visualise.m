% OPT_VISUALISE
% Visualises the results from opt_run.
%
% Produces four figures:
%   1. Metrics summary bar chart — settle time, overshoot, xc_rms, effort
%      for every optimised controller side by side
%   2. Pareto front — settling time vs control effort, one point per
%      controller; shows the efficiency frontier and where each controller
%      sits on the speed vs effort trade-off
%   3. Gain comparison — optimised parameter values per controller,
%      normalised to [0,1] within each parameter's bounds so controllers
%      can be compared on the same axes regardless of parameter units
%   4. Warm-start vs optimised improvement — percentage change in each
%      metric from the warm-start point to the optimised point
%
% Usage:
%   >> opt_visualise              % reads opt_results.mat in current folder
%   >> opt_visualise('path/to/opt_results.mat')

function opt_visualise(results_file)

if nargin < 1 || isempty(results_file)
    results_file = 'opt_results.mat';
end

if ~isfile(results_file)
    error('opt_visualise: file not found: %s\n  Run opt_run first.', results_file);
end

fprintf('\n[opt_visualise] Loading %s ...\n', results_file);
S = load(results_file, 'opt_results');
res = S.opt_results;

ctrls = fieldnames(res.controllers);
n     = numel(ctrls);

if n == 0
    error('No controller results found in opt_results.');
end

% ---- Colour palette (one colour per controller) ------------------------
pal = [0.85 0.25 0.20;   % red
       0.20 0.50 0.85;   % blue
       0.20 0.72 0.45;   % green
       0.90 0.65 0.15;   % amber
       0.65 0.25 0.85;   % purple
       0.20 0.75 0.80;   % cyan
       0.90 0.40 0.65];  % pink
pal = pal(1:n,:);

% ---- Collect metrics ---------------------------------------------------
t_settle   = nan(n,1);
overshoot  = nan(n,1);
xc_rms_v   = nan(n,1);
effort_v   = nan(n,1);
cost_v     = nan(n,1);
captured_v = false(n,1);

for ci = 1:n
    r = res.controllers.(ctrls{ci});
    m = r.best_metrics;
    cost_v(ci)     = r.best_cost;
    captured_v(ci) = isfield(m,'captured') && m.captured;
    if isfield(m,'t_settle')   && isfinite(m.t_settle),      t_settle(ci)  = m.t_settle;  end
    if isfield(m,'overshoot_deg')&&isfinite(m.overshoot_deg),overshoot(ci) = m.overshoot_deg; end
    if isfield(m,'xc_rms')    && isfinite(m.xc_rms),         xc_rms_v(ci) = m.xc_rms;    end
    if isfield(m,'total_effort')&&isfinite(m.total_effort),   effort_v(ci) = m.total_effort; end
end

short_names = cellfun(@(c) strrep(c,'FuzzyPID_','FPD-'), ctrls, 'UniformOutput',false);
short_names = cellfun(@(c) strrep(c,'Fuzzy','F'),        short_names,'UniformOutput',false);
short_names = cellfun(@(c) strrep(c,'Classical','Cls'),  short_names,'UniformOutput',false);
short_names = cellfun(@(c) strrep(c,'Adaptive','Adp'),   short_names,'UniformOutput',false);

% =========================================================================
%  FIGURE 1 — METRICS SUMMARY
% =========================================================================
fig1 = figure('Name','Optimised Metrics Summary','Color','w', ...
               'Position',[40 40 1100 480]);
metrics_data = {t_settle, 'Settle time (s)', 5;
                overshoot, 'Overshoot (deg)', 30;
                xc_rms_v,  'Cart RMS (m)',    0.5;
                effort_v,  'Effort (N*s)',    100};

for mi = 1:4
    data  = metrics_data{mi,1};
    label = metrics_data{mi,2};
    ref   = metrics_data{mi,3};

    ax = subplot(1,4,mi);
    hold on;
    for ci = 1:n
        if isnan(data(ci))
            b = bar(ci, ref*2, 'FaceColor', [0.85 0.85 0.85], ...
                    'EdgeColor','none');
            text(ci, ref*2*1.02, 'N/A','HorizontalAlignment','center', ...
                 'FontSize',7,'Color',[0.5 0.5 0.5]);
        else
            b = bar(ci, data(ci), 'FaceColor', pal(ci,:), 'EdgeColor','none');
        end
    end
    yline(ref,'r--','LineWidth',1.2,'Label','ref');
    set(ax,'XTick',1:n,'XTickLabel',short_names,'XTickLabelRotation',35, ...
           'FontSize',8);
    ylabel(label); grid on; box on;
    title(label,'FontSize',9);

    % Star on best
    [~,best] = min(data);
    if isfinite(data(best))
        text(best, data(best)*1.05,'★','FontSize',14,'Color',[0.1 0.6 0.2], ...
             'HorizontalAlignment','center');
    end
end
sgtitle(sprintf('Optimised Controller Metrics  |  Preset: %s  |  Swing-up: %s', ...
                res.preset, res.swingup), 'FontSize',11,'FontWeight','bold');
drawnow;

% =========================================================================
%  FIGURE 2 — PARETO FRONT (settle time vs effort)
% =========================================================================
fig2 = figure('Name','Pareto Front — Settle vs Effort','Color','w', ...
               'Position',[80 80 700 520]);
ax2  = axes(fig2); hold(ax2,'on'); grid(ax2,'on'); box(ax2,'on');

% Draw Pareto-efficient front
valid = ~isnan(t_settle) & ~isnan(effort_v);
if sum(valid) >= 2
    pareto_front(ax2, t_settle(valid), effort_v(valid));
end

% Plot each controller
for ci = 1:n
    if isnan(t_settle(ci)) || isnan(effort_v(ci))
        % Failed controller — plot at high values with X marker
        scatter(ax2, 20, max(effort_v(valid))*1.2, 120, pal(ci,:), 'x', ...
                'LineWidth',2.5, 'DisplayName',[short_names{ci} ' (failed)']);
    else
        scatter(ax2, t_settle(ci), effort_v(ci), 160, pal(ci,:), 'o', ...
                'filled','MarkerEdgeColor','k','LineWidth',0.8, ...
                'DisplayName', short_names{ci});
        text(ax2, t_settle(ci)+0.05, effort_v(ci)+1, short_names{ci}, ...
             'FontSize',8,'Color',pal(ci,:)*0.7);
    end
end

xlabel(ax2,'Settling time (s)','FontSize',11);
ylabel(ax2,'Control effort (N·s)','FontSize',11);
title(ax2,sprintf('Pareto Front — Speed vs Effort  |  %s preset', res.preset), ...
     'FontSize',11,'FontWeight','bold');
legend(ax2,'Location','northeast','FontSize',8);

% Annotate LQR as baseline if present
if isfield(res.controllers,'LQR')
    r_lqr = res.controllers.LQR;
    m_lqr = r_lqr.best_metrics;
    if isfield(m_lqr,'t_settle') && isfinite(m_lqr.t_settle)
        xline(ax2, m_lqr.t_settle,'k--','LineWidth',1,'Label','LQR settle');
        yline(ax2, m_lqr.total_effort,'k:','LineWidth',1,'Label','LQR effort');
    end
end
drawnow;

% =========================================================================
%  FIGURE 3 — NORMALISED GAIN COMPARISON
% =========================================================================
fig3 = figure('Name','Optimised Gains (normalised)','Color','w', ...
               'Position',[120 120 1100 420]);

% Find the longest parameter vector
max_params = 0;
for ci = 1:n
    r = res.controllers.(ctrls{ci});
    max_params = max(max_params, numel(r.best_params));
end

ax3 = axes(fig3);
hold(ax3,'on'); grid(ax3,'on'); box(ax3,'on');

% Build normalised matrix: rows = parameters, cols = controllers
% (transposed from before so that grouped bar colours one bar per controller)
bar_data = zeros(max_params, n);
for ci = 1:n
    r     = res.controllers.(ctrls{ci});
    p_opt = r.best_params(:);
    lb    = r.lb(:);
    ub    = r.ub(:);
    np    = numel(p_opt);
    p_norm = (p_opt - lb) ./ max(ub - lb, 1e-9);
    bar_data(1:np, ci) = p_norm;
end

% One grouped bar call — rows are param indices (x-axis), cols are controllers
bh = bar(ax3, bar_data, 'grouped');
% bh has one element per controller (column) — apply colour correctly
for ci = 1:min(n, numel(bh))
    bh(ci).FaceColor = pal(ci,:);
    bh(ci).EdgeColor = 'none';
    bh(ci).DisplayName = short_names{ci};
end

% Build param name labels from the controller with the most parameters
all_pnames = {};
for ci = 1:n
    r = res.controllers.(ctrls{ci});
    if numel(r.param_names) > numel(all_pnames)
        all_pnames = r.param_names;
    end
end
set(ax3,'XTick',1:max_params,'XTickLabel',all_pnames, ...
        'XTickLabelRotation',25,'FontSize',8);
ylabel(ax3,'Normalised value (0 = lower bound, 1 = upper bound)');
title(ax3,'Optimised Parameters — Normalised to Bounds', ...
     'FontSize',11,'FontWeight','bold');
legend(ax3,'Location','northeast','FontSize',8);
drawnow;

% =========================================================================
%  FIGURE 4 — WARM-START VS OPTIMISED COST
% =========================================================================
fig4 = figure('Name','Cost Improvement from Warm Start','Color','w', ...
               'Position',[160 160 800 420]);
ax4 = axes(fig4); hold(ax4,'on'); grid(ax4,'on'); box(ax4,'on');

cost_ws_all  = nan(n,1);
cost_opt_all = nan(n,1);

for ci = 1:n
    r = res.controllers.(ctrls{ci});
    % opt_run records the warm-start cost, so no re-simulation is needed.
    % Fall back to evaluating it only for results saved before that change.
    if isfield(r,'cost_warm') && isfinite(r.cost_warm)
        c_ws = r.cost_warm;
    else
        try
            [c_ws,~] = opt_cost(r.warm_start, ctrls{ci}, ...
                                build_base_cfg_from_res(res), res.cost_weights);
        catch
            c_ws = nan;
        end
    end
    cost_ws_all(ci)  = c_ws;
    cost_opt_all(ci) = r.best_cost;
end

x_pos = 1:n;
bar(ax4, x_pos - 0.2, cost_ws_all,  0.35, 'FaceColor',[0.75 0.75 0.78], ...
    'EdgeColor','none','DisplayName','Warm start');
for ci = 1:n
    bar(ax4, ci+0.2, cost_opt_all(ci), 0.35, 'FaceColor', pal(ci,:), ...
        'EdgeColor','none');
end
% Improvement arrows
for ci = 1:n
    if isfinite(cost_ws_all(ci)) && isfinite(cost_opt_all(ci))
        pct = (cost_ws_all(ci)-cost_opt_all(ci))/max(cost_ws_all(ci),1)*100;
        if pct > 0
            text(ci, max(cost_ws_all(ci),cost_opt_all(ci))*1.05, ...
                 sprintf('%.0f%%↓',pct),'FontSize',8, ...
                 'HorizontalAlignment','center','Color',[0.1 0.5 0.2]);
        end
    end
end

set(ax4,'XTick',1:n,'XTickLabel',short_names,'XTickLabelRotation',25,'FontSize',8);
ylabel(ax4,'Cost (weighted sum)');
title(ax4,'Cost: Warm Start vs Optimised','FontSize',11,'FontWeight','bold');
legend(ax4,{'Warm start','Optimised'},'Location','northeast');
drawnow;

% =========================================================================
%  PRINT SUMMARY TABLE
% =========================================================================
fprintf('\n=== OPTIMISATION RESULTS SUMMARY ===\n');
fprintf('  Preset: %s  |  Swing-up: %s\n', res.preset, res.swingup);
fprintf('  Cost weights: settle=%.1f  overshoot=%.1f  xc_rms=%.1f  effort=%.1f\n\n', ...
        res.cost_weights);
fprintf('  %-20s  %8s  %10s  %10s  %8s  %8s\n', ...
        'Controller','Cost','Settle(s)','Over(deg)','XcRMS(m)','Effort');
fprintf('  %-20s  %8s  %10s  %10s  %8s  %8s\n', ...
        repmat('-',1,20),repmat('-',1,8),repmat('-',1,10), ...
        repmat('-',1,10),repmat('-',1,8),repmat('-',1,8));
for ci = 1:n
    r = res.controllers.(ctrls{ci});
    m = r.best_metrics;
    ts_s  = fmtv(m,'t_settle','%.2f');
    ov_s  = fmtv(m,'overshoot_deg','%.1f');
    xc_s  = fmtv(m,'xc_rms','%.3f');
    ef_s  = fmtv(m,'total_effort','%.1f');
    flag  = '';
    if isfield(m,'captured') && ~m.captured, flag = ' [NO CAPTURE]'; end
    fprintf('  %-20s  %8.3f  %10s  %10s  %8s  %8s%s\n', ...
            ctrls{ci}, r.best_cost, ts_s, ov_s, xc_s, ef_s, flag);
end
fprintf('\n  ★ = best in column\n');
fprintf('\n[opt_visualise] Done.\n\n');

end   % function opt_visualise


% =========================================================================
%  HELPERS
% =========================================================================
function pareto_front(ax, t_settle, effort)
% Draw a staircase Pareto front on the scatter plot
% Pareto-efficient points: no other point dominates on BOTH axes

n  = numel(t_settle);
dominated = false(n,1);
for i = 1:n
    for j = 1:n
        if i ~= j && t_settle(j) <= t_settle(i) && effort(j) <= effort(i) && ...
           (t_settle(j) < t_settle(i) || effort(j) < effort(i))
            dominated(i) = true;
            break;
        end
    end
end
pts = ~dominated;
if sum(pts) < 2, return; end

% Sort Pareto points by settle time
[ts_sorted, idx] = sort(t_settle(pts));
ef_sorted = effort(pts); ef_sorted = ef_sorted(idx);

% Draw staircase
xs = [ts_sorted(1)]; ys = [ef_sorted(1)];
for i = 2:numel(ts_sorted)
    xs(end+1) = ts_sorted(i); ys(end+1) = ys(end);
    xs(end+1) = ts_sorted(i); ys(end+1) = ef_sorted(i);
end
plot(ax, xs, ys, '--', 'Color',[0.6 0.6 0.6],'LineWidth',1.2, ...
     'DisplayName','Pareto front');
end


function s = fmtv(m, field, fmt)
if isfield(m,field) && isfinite(m.(field))
    s = sprintf(fmt, m.(field));
else
    s = '—';
end
end


function cfg = build_base_cfg_from_res(res)
% Reconstruct a minimal base_cfg from saved results for warm-start re-eval
p   = pendulum_params(res.preset);
cfg.p          = p;
cfg.swingup    = res.swingup;
cfg.t_end      = 20.0;
cfg.dt         = 0.005;
cfg.x0         = [0;0;pi;0.05];
cfg.run_dir    = '';
cfg.chk_interval = 1e9;
cfg.sucfg      = struct('k_su',10,'k1',4,'k2',10,'E_thresh',0.05, ...
                        'omega',0.8*p.omega_n,'F_amp',0.85*p.Fmax, ...
                        'phase_aware',true);
cfg.fcfg       = struct('n_sets',7,'range',[-180 180],'mf_type','triangular', ...
                        'partition','uniform','inference','sugeno','and_op','min', ...
                        'defuzz','weighted_avg','type2',false,'fou_width',0.20);
cfg.rule_base  = fp_rule_base('expert',7);
cfg.gcfg       = struct('kp_max',300,'kd_max',50,'ki_max',0.1, ...
                        'kp_min',0,'kd_min',0,'ki_min',0,'use_lpf',false, ...
                        'kp_xc',1.5,'kd_xc',2.0);
cfg.pidcfg     = struct('kp',80,'kd',15,'ki',0.02,'auto_tune',false, ...
                        'i_clamp',500,'kp_xc',1.5,'kd_xc',2.0);
cfg.smccfg     = struct('lambda',5,'phi_boundary',0.05,'K_min',2,'K_max',15, ...
                        'kp_xc',1.5,'kd_xc',2.0);
cfg.adapt_cfg  = struct('eta_kp',0.01,'eta_kd',0.005,'eta_ki',0.001, ...
                        'freeze',false,'kp_bounds',[0 500], ...
                        'kd_bounds',[0 100],'ki_bounds',[0 0.5]);
cfg.lqr_Q      = [0.1,0.01,200,20];
cfg.lqr_R      = 0.01;
cfg.disturbance= struct('kick_enabled',false,'wind_enabled',false, ...
                        'friction_enabled',false,'mass_enabled',false, ...
                        'tilt_enabled',false);
cfg.quiet      = true;
end
