function fp_export_figures(run_dir, results, cfg, opts)
% FP_EXPORT_FIGURES
% Exports all figures and GIFs for one pendulum control run.
%
% Figure inventory per run:
%   00_parameters.png         — system params + open-loop analysis
%   fuzzy_mf.png              — membership functions used (all types)
%   control_surface_kp.png    — 3D Kp(e, edot) surface
%   control_surface_kd.png    — 3D Kd(e, edot) surface
%   rulebase_heatmap.png      — static heatmap of kp/kd/ki tables
%   response_<ctrl>.png/.gif  — theta, phi, F, energy per controller
%   phase_plane_<ctrl>.png/.gif — phase plane with separatrix
%   gains_<ctrl>.png          — kp/kd/ki over time (fuzzy controllers only)
%   lyapunov_<ctrl>.png       — V(x) and V_dot(x) for both phases
%   metrics_<ctrl>.png        — time-history metrics per controller
%   comparison_all.png        — all controllers on one plot
%   metrics_comparison.png    — bar chart of scalar metrics

if nargin < 4 || isempty(opts), opts = struct(); end
opts = def(opts,'dpi',         150);
opts = def(opts,'make_gifs',   true);
opts = def(opts,'gif_fps',     10);
opts = def(opts,'gif_frames',  120);
opts = def(opts,'gif_px',      [1200 600]);

fig_dir = fullfile(run_dir, 'figures');
if ~exist(fig_dir,'dir'), mkdir(fig_dir); end

p    = results.p;
fcfg = results.fcfg;
rb   = results.rule_base;
keys = fieldnames(results.runs);
n_ctrl = numel(keys);

% Colour palette — one colour per controller
palette = lines(max(n_ctrl,2));
col = containers.Map(keys, num2cell(palette(1:n_ctrl,:),2));

fprintf('[fp_export_figures] Exporting %d controllers to %s\n', n_ctrl, run_dir);

% =========================================================================
%  00 — Parameters overview
% =========================================================================
fig = figure('Visible','off','Color','w','Position',[0 0 1000 560]);
subplot(1,2,1); axis off;
pdata = {
    'Preset',           p.name
    'Cart mass mc',     sprintf('%.3f kg', p.mc)
    'Bob mass m',       sprintf('%.3f kg', p.m)
    'Rod length l',     sprintf('%.3f m',  p.l)
    'Fmax',             sprintf('%.1f N',  p.Fmax)
    'Capture zone',     sprintf('%.1f deg', p.capture_deg)
    'omega_n',          sprintf('%.3f rad/s', p.omega_n)
    'E_target',         sprintf('%.4f J',  p.E_target)
    'MF type',          fcfg.mf_type
    'N sets',           num2str(fcfg.n_sets)
    'Inference',        fcfg.inference
    'Rule preset',      rb.preset
    'Type-2',           mat2str(fcfg.type2)
    'Swing-up',         cfg.swingup
};
for ri = 1:size(pdata,1)
    text(0.02, 1-ri*0.065, pdata{ri,1}, 'FontSize',10,'FontWeight','bold', ...
         'Units','normalized','Color',[0.3 0.3 0.3],'Parent',gca);
    text(0.52, 1-ri*0.065, pdata{ri,2}, 'FontSize',10, ...
         'Units','normalized','Parent',gca);
end
title('Run Parameters','FontSize',12,'FontWeight','bold');

subplot(1,2,2);
% Open-loop eigenvalue analysis
[A_lin,~,~,~,lin_info] = pendulum_linearise(p);
ev = lin_info.eig_ol;
plot(real(ev), imag(ev), 'rx','MarkerSize',12,'LineWidth',2.5); hold on;
xline(0,'k--','LineWidth',1);
grid on; xlabel('Re(\lambda)'); ylabel('Im(\lambda)');
title('Open-loop eigenvalues (linearised)');
text(0.05,0.92,sprintf('Controllability rank: %d/4',lin_info.rank_co), ...
     'Units','normalized','FontSize',9,'Color',[0.1 0.5 0.1]);
text(0.05,0.82,sprintf('Observability rank: %d/4',lin_info.rank_obs), ...
     'Units','normalized','FontSize',9,'Color',[0.1 0.5 0.1]);
text(0.05,0.72,lin_info.note, ...
     'Units','normalized','FontSize',7,'Color',[0.5 0.5 0.5], ...
     'Interpreter','none');

save_fig(fig, fullfile(fig_dir,'00_parameters.png'), opts.dpi);

% =========================================================================
%  Membership function plot
% =========================================================================
fig = figure('Visible','off','Color','w','Position',[0 0 900 360]);
x_vals = linspace(fcfg.range(1), fcfg.range(2), 300);
mu_all = zeros(numel(x_vals), fcfg.n_sets);
fcfg_t1 = fcfg; fcfg_t1.type2 = false;   % always plot Type-1 MFs for clarity
for xi = 1:numel(x_vals)
    mu_all(xi,:) = fp_membership(x_vals(xi), fcfg_t1)';
end
plot(x_vals, mu_all, 'LineWidth',1.8);
xlabel('Input (deg or deg/s)'); ylabel('\mu(x)');
title(sprintf('Membership Functions: %s, N=%d, %s partition', ...
              fcfg.mf_type, fcfg.n_sets, fcfg.partition));
ylim([0 1.05]); grid on;
% Label centres
[~,centres] = fp_membership(0, fcfg_t1);
for ni = 1:fcfg.n_sets
    text(centres(ni), 1.02, sprintf('MF%d',ni), 'FontSize',7, ...
         'HorizontalAlignment','center');
end
save_fig(fig, fullfile(fig_dir,'fuzzy_mf.png'), opts.dpi);

% =========================================================================
%  Control surface plots (kp and kd)
% =========================================================================
e_ax    = linspace(fcfg.range(1), fcfg.range(2), 40);
edot_ax = linspace(fcfg.range(1), fcfg.range(2), 40);
[EE, EDOT] = meshgrid(e_ax, edot_ax);
KP_surf = zeros(size(EE));
KD_surf = zeros(size(EE));
for ii = 1:numel(EE)
    [kp_i,kd_i,~,~] = fp_inference(EE(ii), EDOT(ii), rb, fcfg);
    KP_surf(ii) = kp_i;
    KD_surf(ii) = kd_i;
end

fig = figure('Visible','off','Color','w','Position',[0 0 960 420]);
subplot(1,2,1);
surf(EE, EDOT, KP_surf,'EdgeColor','none'); colorbar;
xlabel('Error (deg)'); ylabel('Error rate (deg/s)'); zlabel('Kp');
title('Fuzzy control surface — Kp'); view(-35,25);
subplot(1,2,2);
surf(EE, EDOT, KD_surf,'EdgeColor','none'); colorbar;
xlabel('Error (deg)'); ylabel('Error rate (deg/s)'); zlabel('Kd');
title('Fuzzy control surface — Kd'); view(-35,25);
save_fig(fig, fullfile(fig_dir,'control_surface.png'), opts.dpi);

% =========================================================================
%  Rule base heatmaps (static)
% =========================================================================
fig = figure('Visible','off','Color','w','Position',[0 0 1100 360]);
subplot(1,3,1);
imagesc(rb.kp_rule); colorbar; axis square;
title('kp rule table'); xlabel('Error\_dot MF'); ylabel('Error MF');
subplot(1,3,2);
imagesc(rb.kd_rule); colorbar; axis square;
title('kd rule table'); xlabel('Error\_dot MF'); ylabel('Error MF');
subplot(1,3,3);
imagesc(rb.ki_rule); colorbar; axis square;
title('ki rule table'); xlabel('Error\_dot MF'); ylabel('Error MF');
sgtitle(sprintf('Rule Base: %s (%d sets)', rb.preset, rb.n_sets));
save_fig(fig, fullfile(fig_dir,'rulebase_heatmap.png'), opts.dpi);

% =========================================================================
%  Per-controller figures
% =========================================================================
for ki = 1:n_ctrl
    key = keys{ki};
    r   = results.runs.(key);
    m   = r.metrics;
    clr = col(key);
    t   = r.t;
    theta_deg = rad2deg(r.x(:,3));
    phi_deg   = rad2deg(r.x(:,1));

    fprintf('  [%d/%d] %s ...\n', ki, n_ctrl, key);

    % ---- Response figure (4-panel) ----------------------------------------
    fig = figure('Visible','off','Color','w','Position',[0 0 opts.gif_px]);

    subplot(2,2,1);
    plot(t, theta_deg, 'Color',clr,'LineWidth',1.5); hold on;
    yline(p.capture_deg,'r--','LineWidth',0.9);
    yline(-p.capture_deg,'r--','LineWidth',0.9);
    yline(0,'k:','LineWidth',0.8);
    if ~isnan(m.t_capture), xline(m.t_capture,'g--','capture'); end
    xlabel('t (s)'); ylabel('\theta (deg)'); title('Pendulum angle');
    grid on;

    subplot(2,2,2);
    plot(t, r.E, 'Color',clr,'LineWidth',1.5); hold on;
    yline(p.E_target,'r--','LineWidth',1.0,'Label','E_{target}');
    xlabel('t (s)'); ylabel('Energy (J)'); title('Pendulum energy');
    grid on;

    subplot(2,2,3);
    plot(t, r.F, 'Color',clr*0.7,'LineWidth',1.3);
    yline(p.Fmax,'r:'); yline(-p.Fmax,'r:');
    xlabel('t (s)'); ylabel('F (N)'); title('Control force');
    grid on;

    subplot(2,2,4);
    plot(t, phi_deg,'Color',[0.4 0.6 0.8],'LineWidth',1.3);
    xlabel('t (s)'); ylabel('\phi (deg)'); title('Cart angle on ring');
    grid on;

    plain = regexprep(key,'_',' ');
    sgtitle(sprintf('%s | %s swing-up | settle=%.2fs', ...
            plain, cfg.swingup, fv(m.t_settle)), 'Interpreter','none');

    save_fig(fig, fullfile(fig_dir,[key '_response.png']), opts.dpi);
    if opts.make_gifs
        make_gif(fig, fullfile(fig_dir,[key '_response.gif']), ...
                 t, r, theta_deg, clr, p, opts);
    end

    % ---- Phase plane figure -----------------------------------------------
    fig = figure('Visible','off','Color','w','Position',[0 0 600 550]);
    plot(theta_deg, rad2deg(r.x(:,4)), 'Color',clr,'LineWidth',1.3); hold on;
    % Separatrix: draw line through origin with slope = real unstable eigenvalue
    % eig_ol is already the vector of eigenvalues from pendulum_linearise
    sep_th  = linspace(-deg2rad(30), deg2rad(30), 100);
    unstable_eig = real(max(results.lin_info.eig_ol));   % +4.4 rad/s
    sep_thd = unstable_eig * sep_th;
    plot(rad2deg(sep_th), rad2deg(sep_thd), 'k--','LineWidth',1.2,...
         'DisplayName','Linear separatrix (approx.)');
    yline(0,'k:'); xline(0,'k:');
    plot(rad2deg(r.x(1,3)), rad2deg(r.x(1,4)), 'ko','MarkerSize',8,...
         'DisplayName','Start');
    plot(rad2deg(r.x(end,3)), rad2deg(r.x(end,4)), 'k^','MarkerSize',8,...
         'DisplayName','End');
    xlabel('\theta (deg)'); ylabel('\dot\theta (deg/s)');
    title(sprintf('Phase plane — %s', regexprep(key,'_',' ')), ...
          'Interpreter','none');
    legend('Location','best'); grid on;
    save_fig(fig, fullfile(fig_dir,[key '_phase_plane.png']), opts.dpi);

    % ---- Gains over time (fuzzy controllers only) -------------------------
    if any(r.kp ~= 0)
        fig = figure('Visible','off','Color','w','Position',[0 0 900 360]);
        plot(t, r.kp, 'LineWidth',1.4,'DisplayName','kp'); hold on;
        plot(t, r.kd, 'LineWidth',1.4,'DisplayName','kd');
        plot(t, r.ki*500, '--','LineWidth',1.0,'DisplayName','ki x500');
        if ~isnan(m.t_capture), xline(m.t_capture,'k--','capture'); end
        xlabel('t (s)'); ylabel('Gain value');
        title(sprintf('Adaptive gains — %s', regexprep(key,'_',' ')), ...
              'Interpreter','none');
        legend('Location','best'); grid on;
        save_fig(fig, fullfile(fig_dir,[key '_gains.png']), opts.dpi);
    end

    % ---- Lyapunov figure --------------------------------------------------
    fig = figure('Visible','off','Color','w','Position',[0 0 900 440]);
    lyap = r.lyapunov;
    subplot(1,2,1);
    yyaxis left;
    plot(t, lyap.V_su,'LineWidth',1.5,'DisplayName','V_{su}'); hold on;
    ylabel('V_{su} = ½(E-E_t)²');
    yyaxis right;
    plot(t, lyap.V_su_dot,'--','LineWidth',1.2,'DisplayName','dV/dt');
    yline(0,'k:');
    ylabel('dV_{su}/dt');
    xlabel('t (s)');
    title(sprintf('Swing-up Lyapunov (%.1f%% steps: dV≤0)', lyap.pct_ok_su));
    grid on; legend('Location','best');

    subplot(1,2,2);
    t_stab = t(lyap.k_stab:end);
    if ~isempty(lyap.V_stab_stab)
        yyaxis left;
        plot(t_stab, lyap.V_stab_stab,'LineWidth',1.5,'Color',clr);
        ylabel('V_{stab} = x^T P x');
        yyaxis right;
        plot(t_stab, lyap.V_stab_dot_stab,'--','LineWidth',1.2,'Color',clr*0.7);
        yline(0,'k:');
        ylabel('dV_{stab}/dt');
    end
    xlabel('t (s)');
    title(sprintf('Stabilisation Lyapunov (%.1f%% steps: dV<0)', ...
                  fv(lyap.pct_ok_stab)));
    grid on;
    sgtitle(regexprep(key,'_',' '), 'Interpreter','none');
    save_fig(fig, fullfile(fig_dir,[key '_lyapunov.png']), opts.dpi);
end

% =========================================================================
%  Combined comparison figure
% =========================================================================
fig = figure('Visible','off','Color','w','Position',[0 0 opts.gif_px]);
subplot(2,1,1); hold on;
for ki = 1:n_ctrl
    key = keys{ki}; r = results.runs.(key);
    plot(r.t, rad2deg(r.x(:,3)), 'Color',col(key),'LineWidth',1.5, ...
         'DisplayName', regexprep(key,'_',' '));
end
yline(0,'k:'); yline(p.capture_deg,'r--'); yline(-p.capture_deg,'r--');
xlabel('t (s)'); ylabel('\theta (deg)');
title('All controllers — pendulum angle'); legend('Location','best'); grid on;

subplot(2,1,2); hold on;
for ki = 1:n_ctrl
    key = keys{ki}; r = results.runs.(key);
    plot(r.t, r.F, 'Color',col(key),'LineWidth',1.3, ...
         'DisplayName', regexprep(key,'_',' '));
end
yline(p.Fmax,'r:'); yline(-p.Fmax,'r:'); yline(0,'k:');
xlabel('t (s)'); ylabel('F (N)');
title('All controllers — control force'); legend('Location','best'); grid on;

sgtitle(sprintf('Comparison: %s swing-up | %d MF %s %s', ...
        cfg.swingup, fcfg.n_sets, fcfg.mf_type, fcfg.inference));
save_fig(fig, fullfile(fig_dir,'comparison_all.png'), opts.dpi);

% =========================================================================
%  Metrics bar chart
% =========================================================================
metrics_names = {'t\_settle (s)','Overshoot (deg)','RMS \theta (deg)','Effort (N*s)'};
n_m = numel(metrics_names);
M_data = zeros(n_ctrl, n_m);
for ki = 1:n_ctrl
    m = results.runs.(keys{ki}).metrics;
    M_data(ki,:) = [fv(m.t_settle,0), fv(m.overshoot_deg,0), ...
                    fv(m.rms_theta_deg,0), fv(m.total_effort,0)];
end

fig = figure('Visible','off','Color','w','Position',[0 0 1000 400]);
for mi = 1:n_m
    subplot(1,n_m,mi);
    b = bar(M_data(:,mi));
    b.FaceColor = 'flat';
    for ki = 1:n_ctrl, b.CData(ki,:) = col(keys{ki}); end
    set(gca,'XTickLabel',cellfun(@(k) regexprep(k,'_',' '), keys, ...
            'UniformOutput',false),'XTickLabelRotation',35,'FontSize',7);
    title(metrics_names{mi},'Interpreter','tex'); grid on;
    % Log scale if range > 100x
    vals = M_data(:,mi); vals = vals(vals>0);
    if ~isempty(vals) && max(vals)/min(vals) > 100
        set(gca,'YScale','log');
        ylabel('log scale');
    end
end
sgtitle('Performance metrics comparison');
save_fig(fig, fullfile(fig_dir,'metrics_comparison.png'), opts.dpi);

fprintf('[fp_export_figures] Done.\n\n');
end

% =========================================================================
%  HELPERS
% =========================================================================
function save_fig(fig, path, dpi)
    exportgraphics(fig, path, 'Resolution', dpi);
    close(fig);
    fprintf('    %s\n', path);
end

function make_gif(fig_tmpl, gif_path, t, r, theta_deg, clr, p, opts)
% Animate the response figure frame by frame
    n     = numel(t);
    skip  = max(1, floor(n/opts.gif_frames));
    frames= 1:skip:n;
    ref_h = []; ref_w = [];

    gfig = figure('Visible','off','Color','w', ...
                  'Position',[0 0 opts.gif_px(1) opts.gif_px(2)]);
    for fi = 1:numel(frames)
        k = frames(fi);
        clf(gfig);

        subplot(2,2,1);
        plot(t(1:k), theta_deg(1:k),'Color',clr,'LineWidth',1.4); hold on;
        yline(p.capture_deg,'r--'); yline(-p.capture_deg,'r--');
        yline(0,'k:'); xlim([t(1) t(end)]); ylim([-200 200]);
        xlabel('t(s)'); ylabel('\theta(deg)'); grid on; title('Angle');

        subplot(2,2,2);
        plot(t(1:k), r.E(1:k),'Color',clr,'LineWidth',1.4); hold on;
        yline(p.E_target,'r--');
        xlim([t(1) t(end)]); xlabel('t(s)'); ylabel('E(J)');
        grid on; title('Energy');

        subplot(2,2,3);
        plot(t(1:k), r.F(1:k),'Color',clr*0.7,'LineWidth',1.3);
        yline(p.Fmax,'r:'); yline(-p.Fmax,'r:');
        xlim([t(1) t(end)]); xlabel('t(s)'); ylabel('F(N)');
        grid on; title('Force');

        subplot(2,2,4);
        plot(theta_deg(1:k), rad2deg(r.x(1:k,4)), ...
             'Color',clr,'LineWidth',1.0);
        xlabel('\theta(deg)'); ylabel('\theta_{dot}(deg/s)');
        grid on; title('Phase plane');

        sgtitle(gfig, sprintf('t = %.2f s', t(k)), 'Interpreter','none');

        cd = getframe(gfig); cd = cd.cdata;
        if isempty(ref_h), ref_h=size(cd,1); ref_w=size(cd,2); end
        cd = lock_frame(cd, ref_h, ref_w);
        [im,cm] = rgb2ind(cd,256,'nodither');
        write_gif(gif_path, im, cm, fi==1, opts.gif_fps);
    end
    close(gfig);
end

function write_gif(path, im, cm, first, fps)
    if first, imwrite(im,cm,path,'gif','Loopcount',inf,'DelayTime',1/fps);
    else,     imwrite(im,cm,path,'gif','WriteMode','append','DelayTime',1/fps); end
end

function out = lock_frame(cd, rh, rw)
    [h,w,~] = size(cd);
    out = cd(1:min(h,rh), 1:min(w,rw), :);
    if size(out,1)<rh, out=cat(1,out,repmat(out(end,:,:),rh-size(out,1),1,1)); end
    if size(out,2)<rw, out=cat(2,out,repmat(out(:,end,:),1,rw-size(out,2),1)); end
end

function v = fv(x, default)
    if nargin < 2, default = 0; end
    if isnan(x) || isempty(x), v = default; else, v = x; end
end

function o = def(o, f, v)
    if ~isfield(o,f) || isempty(o.(f)), o.(f) = v; end
end
