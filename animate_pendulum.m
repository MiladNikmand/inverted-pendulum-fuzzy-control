function animate_pendulum(run_dir, ctrl_name, opts)
% ANIMATE_PENDULUM
% Animation of the inverted pendulum on a cart.
%
% OUTPUTS PER CONTROLLER:
%   <ctrl>_pendulum.gif  — 3D scene only
%   <ctrl>_graphs.gif    — 6-panel time-series graphs
%   Live window          — combined 4-panel view (interactive)
%
% GRAPH PANELS (graphs GIF):
%   phase plane | theta trace | xc trace (with xc_ref marker)
%   energy      | control force | governor theta_ref
%
% GEOMETRY (unchanged):
%   Cart slides around a horizontal ring of display radius R_display.
%   phi_display = xc / R_display
%   Cart: [R*cos(phi), R*sin(phi), 0]
%   Bob:  cart + l*sin(theta)*[cos(phi),sin(phi),0] + [0,0,l*cos(theta)]
%
% Usage:
%   animate_pendulum
%   animate_pendulum('Pendulum_Control_Run_015', 'FuzzyPID_Sugeno')
%   animate_pendulum('...', 'all')

% ---- Find run folder ---------------------------------------------------
% No prompt: always use the highest-numbered run folder that contains a
% readable results.mat. Pass run_dir explicitly to target a specific run.
if nargin < 1 || isempty(run_dir)
    run_dir = latest_run_dir();
    fprintf('[animate_pendulum] Using latest run: %s\n', run_dir);
end

if nargin < 2 || isempty(ctrl_name), ctrl_name = 'all'; end
if nargin < 3 || isempty(opts),      opts = struct();    end

opts = apd(opts,'anim_speed',   1.5);
opts = apd(opts,'max_frames',   600);
opts = apd(opts,'gif_fps',      10);
opts = apd(opts,'gif_max_frames',400);
opts = apd(opts,'gif_px',       [1400 720]);   % live + graphs GIF
opts = apd(opts,'gif_px_pend',  [760 700]);    % pendulum GIF
opts = apd(opts,'save_gifs',    true);
opts = apd(opts,'view_az',      40);
opts = apd(opts,'view_el',      20);
opts = apd(opts,'R_display',    0.35);
% Web-sized copies, written in the same pass as the full-resolution GIFs.
% Costs one resize per kept frame; no re-reading, no re-simulation.
opts = apd(opts,'web_gif',      false);              % master switch
opts = apd(opts,'web_width',    480);                % px
opts = apd(opts,'web_frames',   60);                 % frames kept
opts = apd(opts,'web_dir',      fullfile('docs','assets','anim'));
opts = apd(opts,'web_keys',     {});                 % {} = every controller

% ---- Load results ------------------------------------------------------
mat_file = fullfile(run_dir,'results.mat');
if ~isfile(mat_file)
    error('results.mat not found in %s', run_dir);
end
S = load(mat_file,'results','cfg');
results = S.results;
cfg_run = S.cfg;
p       = results.p;

% ---- Disturbance / tilt info -------------------------------------------
tilt_deg = 0;
if isfield(cfg_run,'disturbance')
    d = cfg_run.disturbance;
    if isfield(d,'tilt_enabled') && d.tilt_enabled && isfield(d,'tilt_angle_deg')
        tilt_deg = d.tilt_angle_deg;
    end
end

xc_ref = 0;
if isfield(p,'xc_ref'), xc_ref = p.xc_ref; end

% ---- Controllers to animate --------------------------------------------
all_keys = fieldnames(results.runs);
if strcmp(ctrl_name,'all')
    anim_keys = all_keys;
else
    if ~isfield(results.runs, ctrl_name)
        error('Controller "%s" not found.', ctrl_name);
    end
    anim_keys = {ctrl_name};
end

% ---- Output directory --------------------------------------------------
% No prompt: GIFs always go to <run_dir>/animation/.
% Set opts.save_gifs = false to skip GIF writing entirely.
anim_dir = fullfile(run_dir,'animation');
if ~exist(anim_dir,'dir'), mkdir(anim_dir); end
if opts.save_gifs
    fprintf('[animate_pendulum] GIFs -> %s\n', anim_dir);
end

% ---- Ring geometry constants (tilted if disturbance active) ------------
R        = opts.R_display;
l_rod    = p.l;
n_ring   = 120;
th_ring  = linspace(0, 2*pi, n_ring);
ring_x   = R * cos(th_ring);
ring_y0  = R * sin(th_ring);
ring_z0  = zeros(1, n_ring);
[ring_y, ring_z] = tiltYZ(ring_y0, ring_z0, tilt_deg);
gnd_ext  = R * 1.6;

% ---- Animate each controller -------------------------------------------
for ai = 1:numel(anim_keys)
    key = anim_keys{ai};
    r   = results.runs.(key);
    t   = r.t;
    x   = r.x;   % [xc, xc_dot, theta, theta_dot]
    n   = numel(t);

    clr       = apd_color(ai);
    plain_lbl = strrep(key,'_',' ');

    theta_deg = rad2deg(x(:,3));
    E_hist    = 0.5*p.m*p.l^2*x(:,4).^2 + p.m*p.g*p.l*cos(x(:,3));

    F_hist = zeros(n,1);
    if isfield(r,'F'),      F_hist = r.F(:);
    elseif isfield(r,'F_hist'), F_hist = r.F_hist(:); end

    tref_hist = zeros(n,1);
    if isfield(r,'theta_ref_hist'), tref_hist = r.theta_ref_hist(:); end

    phi_disp  = x(:,1) / R;

    % GIF frame count: fps * duration, capped
    gif_n    = min(round(opts.gif_fps * t(end)), opts.gif_max_frames);
    skip_gif = max(1, floor(n / max(gif_n,1)));
    gif_idx  = 1:skip_gif:n;

    skip_live = max(1, floor(n/opts.max_frames));
    dt_live   = (t(end)-t(1))/n * skip_live / opts.anim_speed;

    % Does this controller get a web-sized copy?
    web_on = opts.web_gif && (isempty(opts.web_keys) || ...
                              any(strcmp(key, opts.web_keys)));
    if web_on && ~exist(opts.web_dir,'dir'), mkdir(opts.web_dir); end
    web_step = max(1, ceil(numel(gif_idx) / opts.web_frames));

    % ============================================================
    %  PASS A1 — PENDULUM GIF (3D only)
    % ============================================================
    if opts.save_gifs
        gp = fullfile(anim_dir, [key '_pendulum.gif']);
        fprintf('  [%s] pendulum GIF (%d frames)...\n', key, numel(gif_idx));

        f1 = figure('Visible','off','Color','k', ...
                    'Position',[0 0 opts.gif_px_pend(1) opts.gif_px_pend(2)]);
        [a3, rod1, bob1, trl1, crt1] = build_3d(f1, [0.05 0.05 0.90 0.88], ...
            ring_x, ring_y, ring_z, gnd_ext, R, l_rod, p, clr, opts, ...
            xc_ref, tilt_deg);

        rh=[]; rw=[]; first_web_p = true;
        for fi = 1:numel(gif_idx)
            k = gif_idx(fi);
            update_3d(rod1,bob1,trl1,crt1, phi_disp, x, k, R, l_rod, tilt_deg);
            title(a3, sprintf('%s   t = %.2f s', plain_lbl, t(k)), ...
                  'Color','w','FontSize',10,'Interpreter','none');
            drawnow;
            cdta = getframe(f1); cdta = cdta.cdata;
            if isempty(rh), rh=size(cdta,1); rw=size(cdta,2); end
            cdta = apd_lock(cdta, rh, rw);
            [im,cm] = rgb2ind(cdta,256,'nodither');
            apd_gif(gp, im, cm, fi==1, opts.gif_fps);

            if web_on && mod(fi-1, web_step)==0
                wp = fullfile(opts.web_dir,[key '_pendulum.gif']);
                web_frame(cdta, wp, first_web_p, opts);
                first_web_p = false;
            end
        end
        close(f1);
        fprintf('    -> %s\n', gp);
        if web_on
            fprintf('    -> %s  (web)\n', ...
                    fullfile(opts.web_dir,[key '_pendulum.gif']));
        end
    end

    % ============================================================
    %  PASS A2 — GRAPHS GIF (6 panels)
    % ============================================================
    if opts.save_gifs
        gg = fullfile(anim_dir, [key '_graphs.gif']);
        fprintf('  [%s] graphs GIF   (%d frames)...\n', key, numel(gif_idx));

        f2 = figure('Visible','off','Color','k', ...
                    'Position',[0 0 opts.gif_px(1) opts.gif_px(2)]);
        G = build_graphs(f2, t, theta_deg, x, E_hist, F_hist, tref_hist, ...
                         xc_ref, p, clr);

        rh=[]; rw=[]; first_web_g = true;
        for fi = 1:numel(gif_idx)
            k = gif_idx(fi);
            update_graphs(G, theta_deg, x, t, E_hist, F_hist, tref_hist, k);
            sgtitle(f2, sprintf('%s  |  %s  |  t = %.2f s', ...
                    run_dir, plain_lbl, t(k)), ...
                    'Color','w','FontSize',10,'Interpreter','none');
            drawnow;
            cdta = getframe(f2); cdta = cdta.cdata;
            if isempty(rh), rh=size(cdta,1); rw=size(cdta,2); end
            cdta = apd_lock(cdta, rh, rw);
            [im,cm] = rgb2ind(cdta,256,'nodither');
            apd_gif(gg, im, cm, fi==1, opts.gif_fps);

            if web_on && mod(fi-1, web_step)==0
                wg = fullfile(opts.web_dir,[key '_graphs.gif']);
                web_frame(cdta, wg, first_web_g, opts);
                first_web_g = false;
            end
        end
        close(f2);
        fprintf('    -> %s\n', gg);
        if web_on
            fprintf('    -> %s  (web)\n', ...
                    fullfile(opts.web_dir,[key '_graphs.gif']));
        end
    end

    % ============================================================
    %  PASS B — Live playback (combined 4-panel, unchanged layout)
    % ============================================================
    fig = figure('Name', sprintf('%s — %s', run_dir, plain_lbl), ...
                 'Color','k', ...
                 'Position',[40 40 opts.gif_px(1) opts.gif_px(2)]);

    [ax3d, ph_line, th_line, E_line, rod_h, bob_h, trail_h, cart_h] = ...
        build_axes(fig, ring_x, ring_y, ring_z, gnd_ext, ...
                   R, l_rod, p, clr, opts, xc_ref, tilt_deg);

    sgtitle(fig, sprintf('%s  |  %s', run_dir, plain_lbl), ...
            'Color','w','FontSize',11,'Interpreter','none');

    for k = 1:skip_live:n
        if ~ishandle(fig), break; end
        update_scene(rod_h, bob_h, trail_h, cart_h, ...
                     ph_line, th_line, E_line, phi_disp, x, k, t, ...
                     theta_deg, E_hist, R, l_rod, tilt_deg);
        drawnow limitrate;
        pause(dt_live);
    end

    if ai < numel(anim_keys)
        pause(1.5);
    end
end

fprintf('\nAnimation complete.\n\n');
end


% =========================================================================
%  BUILD_3D  — 3D scene only (used by pendulum GIF)
% =========================================================================
function [ax3d, rod_h, bob_h, trail_h, cart_h] = build_3d( ...
        fig, pos, ring_x, ring_y, ring_z, gnd_ext, R, l, p, clr, opts, ...
        xc_ref, tilt_deg)

dark   = [0.10 0.10 0.12];
lite   = [0.75 0.80 0.85];
grid_c = [0.25 0.28 0.32];

ax3d = axes('Parent',fig,'Position',pos);
set(ax3d,'Color',dark,'GridColor',grid_c,'XColor',lite,'YColor',lite, ...
         'ZColor',lite,'FontSize',8);
hold(ax3d,'on'); grid(ax3d,'on'); axis(ax3d,'equal');
view(ax3d, opts.view_az, opts.view_el);
xlabel(ax3d,'X (m)','Color',lite);
ylabel(ax3d,'Y (m)','Color',lite);
zlabel(ax3d,'Z (m)','Color',lite);

lim = R + l + 0.05;
xlim(ax3d,[-lim lim]); ylim(ax3d,[-lim lim]); zlim(ax3d,[-l-0.05 l+0.05]);

draw_static_3d(ax3d, ring_x, ring_y, ring_z, gnd_ext, R, l, ...
               xc_ref, tilt_deg);

[rod_h, bob_h, trail_h, cart_h] = make_parts(ax3d, clr);
end


% =========================================================================
%  BUILD_AXES  — combined 4-panel (live view), 3D block unchanged
% =========================================================================
function [ax3d, ph_line, th_line, E_line, rod_h, bob_h, trail_h, cart_h] = ...
    build_axes(fig, ring_x, ring_y, ring_z, gnd_ext, R, l, p, clr, opts, ...
               xc_ref, tilt_deg)

dark   = [0.10 0.10 0.12];
lite   = [0.75 0.80 0.85];
grid_c = [0.25 0.28 0.32];

% LEFT: 3D view
ax3d = subplot('Position',[0.02 0.04 0.52 0.88],'Parent',fig);
set(ax3d,'Color',dark,'GridColor',grid_c,'XColor',lite,'YColor',lite, ...
         'ZColor',lite,'FontSize',8);
hold(ax3d,'on'); grid(ax3d,'on'); axis(ax3d,'equal');
view(ax3d, opts.view_az, opts.view_el);
xlabel(ax3d,'X (m)','Color',lite);
ylabel(ax3d,'Y (m)','Color',lite);
zlabel(ax3d,'Z (m)','Color',lite);
title(ax3d,'Inverted Pendulum on Cart — 3D View','Color','w','FontSize',9);

lim = R + l + 0.05;
xlim(ax3d,[-lim lim]); ylim(ax3d,[-lim lim]); zlim(ax3d,[-l-0.05 l+0.05]);

draw_static_3d(ax3d, ring_x, ring_y, ring_z, gnd_ext, R, l, ...
               xc_ref, tilt_deg);

[rod_h, bob_h, trail_h, cart_h] = make_parts(ax3d, clr);

% TOP-RIGHT: Phase plane
ax_ph = subplot('Position',[0.57 0.68 0.40 0.26],'Parent',fig);
set(ax_ph,'Color',dark,'GridColor',grid_c,'XColor',lite,'YColor',lite,'FontSize',8);
hold(ax_ph,'on'); grid(ax_ph,'on');
xlabel(ax_ph,'\theta (deg)','Color',lite);
ylabel(ax_ph,'\theta-dot (deg/s)','Color',lite);
title(ax_ph,'Phase plane','Color','w','FontSize',9);
plot(ax_ph,0,0,'x','Color',[1 0.3 0.3],'MarkerSize',10,'LineWidth',2);
yline(ax_ph,0,'Color',grid_c,'LineWidth',0.8);
xline(ax_ph,0,'Color',grid_c,'LineWidth',0.8);
ph_line = plot(ax_ph,nan,nan,'Color',clr,'LineWidth',1.6);

% MID-RIGHT: theta trace
ax_th = subplot('Position',[0.57 0.37 0.40 0.26],'Parent',fig);
set(ax_th,'Color',dark,'GridColor',grid_c,'XColor',lite,'YColor',lite,'FontSize',8);
hold(ax_th,'on'); grid(ax_th,'on');
xlabel(ax_th,'t (s)','Color',lite);
ylabel(ax_th,'\theta (deg)','Color',lite);
title(ax_th,'Pendulum angle','Color','w','FontSize',9);
yline(ax_th, p.capture_deg,'--','Color',[1 0.6 0.2],'LineWidth',1);
yline(ax_th,-p.capture_deg,'--','Color',[1 0.6 0.2],'LineWidth',1);
yline(ax_th,0,'Color',grid_c,'LineWidth',0.8);
ylim(ax_th,[-185 185]);
th_line = plot(ax_th,nan,nan,'Color',clr,'LineWidth',1.8);

% BOT-RIGHT: Energy trace
ax_E = subplot('Position',[0.57 0.06 0.40 0.26],'Parent',fig);
set(ax_E,'Color',dark,'GridColor',grid_c,'XColor',lite,'YColor',lite,'FontSize',8);
hold(ax_E,'on'); grid(ax_E,'on');
xlabel(ax_E,'t (s)','Color',lite);
ylabel(ax_E,'E (J)','Color',lite);
title(ax_E,'Pendulum energy','Color','w','FontSize',9);
yline(ax_E,p.E_target,'--','Color',[1 0.3 0.3],'LineWidth',1);
E_line = plot(ax_E,nan,nan,'Color',clr,'LineWidth',1.8);

end


% =========================================================================
%  STATIC 3D ELEMENTS (ground, ring, target, reference, light)
% =========================================================================
function draw_static_3d(ax3d, ring_x, ring_y, ring_z, gnd_ext, R, l, ...
                        xc_ref, tilt_deg)

% Ground plane (tilted with the track when tilt disturbance is active)
[gx,gy0] = meshgrid(linspace(-gnd_ext,gnd_ext,30));
gz0 = zeros(size(gx));
[gy,gz] = tiltYZ(gy0, gz0, tilt_deg);
surf(ax3d, gx, gy, gz-0.002, ...
     'FaceColor',[0.18 0.20 0.22],'EdgeColor','none','FaceAlpha',0.6);

% Ring (cart track)
plot3(ax3d, ring_x, ring_y, ring_z, ...
      'Color',[0.55 0.60 0.65],'LineWidth',2.0,'LineStyle','--');

% Target cart position marker on the ring
phi_ref = xc_ref / R;
tx  = R*cos(phi_ref);
ty0 = R*sin(phi_ref);
[ty,tz] = tiltYZ(ty0, 0, tilt_deg);
plot3(ax3d, tx, ty, tz, 'x', 'Color',[1 0.30 0.30], ...
      'MarkerSize',15,'LineWidth',2.5);
text(ax3d, tx, ty, tz+0.06, 'target', 'Color',[1 0.45 0.45], ...
     'FontSize',8,'HorizontalAlignment','center');

% Upright reference line at ring centre
plot3(ax3d,[0 0],[0 0],[-l l],'--','Color',[0.35 0.38 0.40],'LineWidth',0.8);

% Drive hub at centre (motor that moves the cart)
plot3(ax3d,0,0,0,'s','MarkerSize',9, ...
      'MarkerFaceColor',[0.45 0.48 0.52],'MarkerEdgeColor','w');

% Tilt annotation
if tilt_deg ~= 0
    text(ax3d, -(R+l)*0.85, -(R+l)*0.85, l*0.85, ...
         sprintf('track tilt: %.1f\\circ', tilt_deg), ...
         'Color',[1 0.80 0.35],'FontSize',9,'FontWeight','bold');
end

% Lighting
light(ax3d,'Position',[1 1 2],'Style','local');
material(ax3d,'dull');
end


function [rod_h, bob_h, trail_h, cart_h] = make_parts(ax3d, clr)
cart_h  = plot3(ax3d,nan,nan,nan,'o','MarkerSize',14, ...
                'MarkerFaceColor',[0.65 0.68 0.72],'MarkerEdgeColor','w', ...
                'LineWidth',1.5);
rod_h   = plot3(ax3d,[nan nan],[nan nan],[nan nan],'-', ...
                'Color',[0.90 0.90 0.92],'LineWidth',3);
bob_h   = plot3(ax3d,nan,nan,nan,'o','MarkerSize',18, ...
                'MarkerFaceColor',clr,'MarkerEdgeColor','w','LineWidth',1.5);
trail_h = plot3(ax3d,nan(1,200),nan(1,200),nan(1,200),'-', ...
                'Color',[clr 0.28],'LineWidth',1.2);
end


% =========================================================================
%  UPDATE 3D PARTS ONLY
% =========================================================================
function update_3d(rod_h, bob_h, trail_h, cart_h, phi_disp, x, k, R, l, tilt_deg)

phi_k   = phi_disp(k);
theta_k = x(k,3);

% Cart on ring (untilted coords, then tilt)
cx  = R * cos(phi_k);
cy0 = R * sin(phi_k);
[cy,cz] = tiltYZ(cy0, 0, tilt_deg);

% Bob (untilted coords, then tilt)
bx  = cx + l * sin(theta_k) * cos(phi_k);
by0 = cy0 + l * sin(theta_k) * sin(phi_k);
bz0 =        l * cos(theta_k);
[by,bz] = tiltYZ(by0, bz0, tilt_deg);

set(cart_h,'XData',cx,'YData',cy,'ZData',cz);
set(rod_h, 'XData',[cx bx],'YData',[cy by],'ZData',[cz bz]);
set(bob_h, 'XData',bx,'YData',by,'ZData',bz);

% Trail — last 60 points
k0 = max(1, k-60);
phi_t = phi_disp(k0:k);
th_t  = x(k0:k,3);
bx_t  = R*cos(phi_t) + l*sin(th_t).*cos(phi_t);
by0_t = R*sin(phi_t) + l*sin(th_t).*sin(phi_t);
bz0_t = l*cos(th_t);
[by_t,bz_t] = tiltYZ(by0_t, bz0_t, tilt_deg);
set(trail_h,'XData',bx_t','YData',by_t','ZData',bz_t');
end


% =========================================================================
%  UPDATE_SCENE  — 3D + the three live panels
% =========================================================================
function update_scene(rod_h, bob_h, trail_h, cart_h, ...
                      ph_line, th_line, E_line, phi_disp, x, k, t, ...
                      theta_deg, E_hist, R, l, tilt_deg)

update_3d(rod_h, bob_h, trail_h, cart_h, phi_disp, x, k, R, l, tilt_deg);

k0p = max(1,k-80);
set(ph_line,'XData',theta_deg(k0p:k),'YData',rad2deg(x(k0p:k,4)));
set(th_line,'XData',t(1:k),'YData',theta_deg(1:k));
set(E_line, 'XData',t(1:k),'YData',E_hist(1:k));
end


% =========================================================================
%  BUILD_GRAPHS  — 6-panel figure for the graphs GIF
% =========================================================================
function G = build_graphs(fig, t, theta_deg, x, E_hist, F_hist, ...
                          tref_hist, xc_ref, p, clr)

dark   = [0.10 0.10 0.12];
lite   = [0.75 0.80 0.85];
grid_c = [0.25 0.28 0.32];

pos = {[0.055 0.565 0.265 0.345];
       [0.375 0.565 0.265 0.345];
       [0.695 0.565 0.265 0.345];
       [0.055 0.085 0.265 0.345];
       [0.375 0.085 0.265 0.345];
       [0.695 0.085 0.265 0.345]};

mk = @(i) styled_axes(fig, pos{i}, dark, lite, grid_c);

% 1 — phase plane
a1 = mk(1);
xlabel(a1,'\theta (deg)','Color',lite); ylabel(a1,'\theta-dot (deg/s)','Color',lite);
title(a1,'Phase plane','Color','w','FontSize',9);
plot(a1,0,0,'x','Color',[1 0.3 0.3],'MarkerSize',10,'LineWidth',2);
yline(a1,0,'Color',grid_c,'LineWidth',0.8);
xline(a1,0,'Color',grid_c,'LineWidth',0.8);
G.ph = plot(a1,nan,nan,'Color',clr,'LineWidth',1.6);

% 2 — theta trace
a2 = mk(2);
xlabel(a2,'t (s)','Color',lite); ylabel(a2,'\theta (deg)','Color',lite);
title(a2,'Pendulum angle','Color','w','FontSize',9);
yline(a2, p.capture_deg,'--','Color',[1 0.6 0.2],'LineWidth',1);
yline(a2,-p.capture_deg,'--','Color',[1 0.6 0.2],'LineWidth',1);
yline(a2,0,'Color',grid_c,'LineWidth',0.8);
xlim(a2,[t(1) t(end)]); ylim(a2,[-185 185]);
G.th = plot(a2,nan,nan,'Color',clr,'LineWidth',1.8);

% 3 — cart position with xc_ref marker
a3 = mk(3);
xlabel(a3,'t (s)','Color',lite); ylabel(a3,'x_c (m)','Color',lite);
title(a3,'Cart position','Color','w','FontSize',9);
yline(a3, xc_ref,'--','Color',[1 0.30 0.30],'LineWidth',1.5);
text(a3, t(1)+0.02*(t(end)-t(1)), xc_ref, sprintf('  x_{ref} = %.2f m',xc_ref), ...
     'Color',[1 0.45 0.45],'FontSize',8,'VerticalAlignment','bottom');
xlim(a3,[t(1) t(end)]);
xpad = 0.1 + 0.05*max(abs(x(:,1)));
ylim(a3,[min(min(x(:,1)),xc_ref)-xpad, max(max(x(:,1)),xc_ref)+xpad]);
G.xc  = plot(a3,nan,nan,'Color',clr,'LineWidth',1.8);
G.xcm = plot(a3,nan,nan,'o','Color',[1 0.30 0.30],'MarkerSize',7, ...
             'MarkerFaceColor',[1 0.30 0.30]);

% 4 — energy
a4 = mk(4);
xlabel(a4,'t (s)','Color',lite); ylabel(a4,'E (J)','Color',lite);
title(a4,'Pendulum energy','Color','w','FontSize',9);
yline(a4,p.E_target,'--','Color',[1 0.3 0.3],'LineWidth',1);
xlim(a4,[t(1) t(end)]);
G.E = plot(a4,nan,nan,'Color',clr,'LineWidth',1.8);

% 5 — control force
a5 = mk(5);
xlabel(a5,'t (s)','Color',lite); ylabel(a5,'F (N)','Color',lite);
title(a5,'Control force','Color','w','FontSize',9);
yline(a5, p.Fmax,'--','Color',[0.60 0.62 0.66],'LineWidth',0.8);
yline(a5,-p.Fmax,'--','Color',[0.60 0.62 0.66],'LineWidth',0.8);
yline(a5,0,'Color',grid_c,'LineWidth',0.8);
xlim(a5,[t(1) t(end)]); ylim(a5,[-p.Fmax*1.15 p.Fmax*1.15]);
G.F = plot(a5,nan,nan,'Color',[0.95 0.70 0.25],'LineWidth',1.5);

% 6 — governor reference
a6 = mk(6);
xlabel(a6,'t (s)','Color',lite); ylabel(a6,'\theta_{ref} (deg)','Color',lite);
title(a6,'Governor reference','Color','w','FontSize',9);
yline(a6,0,'Color',grid_c,'LineWidth',0.8);
xlim(a6,[t(1) t(end)]);
tr_max = max(2, max(abs(tref_hist))*1.3);
ylim(a6,[-tr_max tr_max]);
G.gov = plot(a6,nan,nan,'Color',[0.45 0.90 0.60],'LineWidth',1.6);
end


function ax = styled_axes(fig, pos, dark, lite, grid_c)
ax = axes('Parent',fig,'Position',pos);
set(ax,'Color',dark,'GridColor',grid_c,'XColor',lite,'YColor',lite,'FontSize',8);
hold(ax,'on'); grid(ax,'on');
end


function update_graphs(G, theta_deg, x, t, E_hist, F_hist, tref_hist, k)
k0 = max(1,k-100);
set(G.ph, 'XData',theta_deg(k0:k),'YData',rad2deg(x(k0:k,4)));
set(G.th, 'XData',t(1:k),'YData',theta_deg(1:k));
set(G.xc, 'XData',t(1:k),'YData',x(1:k,1));
set(G.xcm,'XData',t(k),  'YData',x(k,1));
set(G.E,  'XData',t(1:k),'YData',E_hist(1:k));
set(G.F,  'XData',t(1:k),'YData',F_hist(1:k));
set(G.gov,'XData',t(1:k),'YData',tref_hist(1:k));
end


% =========================================================================
%  LATEST RUN FOLDER
% =========================================================================
function rd = latest_run_dir()
% Returns the highest-numbered Pendulum_Control_Run_NNN folder that
% actually contains a readable results.mat. Folders that are empty or
% still being written are skipped rather than causing an error.

dirs = dir('Pendulum_Control_Run_*');
dirs = dirs([dirs.isdir]);
if isempty(dirs)
    error(['No Pendulum_Control_Run_NNN folders found in:\n  %s\n' ...
           'Run run_all (or pendulum_launcher) first.'], pwd);
end

% Extract run numbers and sort descending
nums = nan(1,numel(dirs));
for di = 1:numel(dirs)
    tok = regexp(dirs(di).name,'^Pendulum_Control_Run_(\d+)$','tokens','once');
    if ~isempty(tok), nums(di) = str2double(tok{1}); end
end
keep = ~isnan(nums);
dirs = dirs(keep);  nums = nums(keep);
if isempty(dirs)
    error('No correctly named run folders found.');
end
[~, order] = sort(nums,'descend');
dirs = dirs(order);  nums = nums(order);

% Walk from newest to oldest, take the first with a usable results.mat
skipped = {};
for di = 1:numel(dirs)
    mf = fullfile(dirs(di).name,'results.mat');
    if ~isfile(mf)
        skipped{end+1} = sprintf('%s (no results.mat)', dirs(di).name); %#ok
        continue;
    end
    vars = who('-file', mf);
    if ~ismember('results', vars)
        skipped{end+1} = sprintf('%s (results.mat has no "results")', ...
                                 dirs(di).name); %#ok
        continue;
    end
    rd = dirs(di).name;
    for si = 1:numel(skipped)
        fprintf('[animate_pendulum] Skipped %s\n', skipped{si});
    end
    return;
end

error(['Found %d run folder(s) but none contain a usable results.mat.\n' ...
       'Newest checked: %s'], numel(dirs), dirs(1).name);
end


% =========================================================================
%  TILT HELPER — rotates (y,z) about the X axis
% =========================================================================
function [y2, z2] = tiltYZ(y, z, tilt_deg)
if tilt_deg == 0
    y2 = y; z2 = z; return;
end
a  = deg2rad(tilt_deg);
y2 = y*cos(a) - z*sin(a);
z2 = y*sin(a) + z*cos(a);
end


% =========================================================================
%  UTILITIES
% =========================================================================
function web_frame(cdta, path, first, opts)
% Writes one downscaled frame of a web GIF. The frame is already in memory
% from getframe, so this costs a resize and a palette quantise -- no file
% re-read, no extra rendering, no second simulation.
try
    sc = opts.web_width / size(cdta,2);
    if sc < 1, cdta = imresize(cdta, sc); end
    [im, cm] = rgb2ind(cdta, 128, 'nodither');
    if first
        imwrite(im,cm,path,'gif','Loopcount',inf,'DelayTime',1/opts.gif_fps);
    else
        imwrite(im,cm,path,'gif','WriteMode','append', ...
                'DelayTime',1/opts.gif_fps);
    end
catch
end
end


function apd_gif(path, im, cm, first, fps)
    if first
        imwrite(im,cm,path,'gif','Loopcount',inf,'DelayTime',1/fps);
    else
        imwrite(im,cm,path,'gif','WriteMode','append','DelayTime',1/fps);
    end
end

function out = apd_lock(cd, rh, rw)
    [h,w,~] = size(cd);
    out = cd(1:min(h,rh), 1:min(w,rw), :);
    if size(out,1)<rh
        out = cat(1,out,repmat(out(end,:,:),rh-size(out,1),1,1));
    end
    if size(out,2)<rw
        out = cat(2,out,repmat(out(:,end,:),1,rw-size(out,2),1));
    end
end

function c = apd_color(idx)
    pal = [0.95 0.35 0.25;
           0.25 0.65 0.95;
           0.30 0.85 0.55;
           0.95 0.75 0.20;
           0.75 0.35 0.95;
           0.25 0.85 0.85;
           0.95 0.50 0.75];
    c = pal(mod(idx-1,size(pal,1))+1,:);
end

function o = apd(o,f,v)
    if ~isfield(o,f)||isempty(o.(f)), o.(f)=v; end
end
