function pendulum_launcher()
% PENDULUM_LAUNCHER
% Main GUI for the Fuzzy Inverted Pendulum project.
%
% Six tabs: Physical Setup | Fuzzy System | Controllers | Simulation |
%           Disturbances   | Run & Results
%
% Usage:  >> pendulum_launcher

% ---- Layout constants (shared by every tab) ----------------------------
L = layout();
C = colors();

% =========================================================================
%  MAIN WINDOW
% =========================================================================
fig = uifigure('Name','Pendulum Control Launcher', ...
               'Position',[60 50 1010 740], ...
               'Color',C.bg,'Resize','on');

hdr = uipanel(fig,'Position',[0 700 1010 40], ...
              'BackgroundColor',C.header,'BorderType','none');
uilabel(hdr,'Text','Fuzzy Inverted Pendulum on Cart — Control Launcher', ...
        'Position',[16 9 620 22],'FontSize',14,'FontWeight','bold','FontColor','w');
uilabel(hdr,'Text','cascade governor  ·  x_c regulation  ·  disturbance suite', ...
        'Position',[640 10 355 20],'FontSize',10, ...
        'FontColor',[0.76 0.88 1.0],'HorizontalAlignment','right');

tg = uitabgroup(fig,'Position',[6 48 998 648]);
t1 = uitab(tg,'Title','  Physical Setup  ','BackgroundColor',C.tab);
t2 = uitab(tg,'Title','  Fuzzy System  ',  'BackgroundColor',C.tab);
t3 = uitab(tg,'Title','  Controllers  ',   'BackgroundColor',C.tab);
t4 = uitab(tg,'Title','  Simulation  ',    'BackgroundColor',C.tab);
t5 = uitab(tg,'Title','  Disturbances  ',  'BackgroundColor',C.tab);
t6 = uitab(tg,'Title','  Run & Results  ', 'BackgroundColor',C.tab);

ftr = uipanel(fig,'Position',[0 0 1010 44], ...
              'BackgroundColor',C.footer,'BorderType','none');
s.status_lbl = uilabel(ftr,'Text','Ready.', ...
    'Position',[16 11 800 22],'FontColor',[0.55 0.85 0.60],'FontSize',11);

% =========================================================================
%  BUILD TABS
% =========================================================================
build_tab1(t1, L, C);
build_tab2(t2, L, C);
build_tab3(t3, L, C);
build_tab4(t4, L, C); 
build_tab5(t5, L, C);
s = build_tab6(t6, s, fig, L, C);

fig.UserData = s;
end


% =========================================================================
%  TAB 1 — PHYSICAL SETUP
% =========================================================================
function build_tab1(parent, L, C)

% ---- Preset ------------------------------------------------------------
pn = mkpanel(parent,[L.m 520 470 82],'Scale Preset',C);
mklbl(pn,'Preset',[L.p 26 90 22],C);
dd = uidropdown(pn,'Items',{'small','medium','large','custom'}, ...
    'Value','medium','Position',[L.p+100 24 150 26],'Tag','preset_dd', ...
    'BackgroundColor',C.input,'FontColor',C.txt);
mklbl(pn,'small · tabletop     medium · lab     large · industrial', ...
    [L.p+264 27 190 20],C,9);

% ---- Parameters --------------------------------------------------------
rows = {'Cart mass  m_c  (kg)','p_mc','2.5';
        'Bob mass  m  (kg)',   'p_m','0.20';
        'Rod length  l  (m)',  'p_l','0.50';
        'Max force  F_max (N)','p_Fmax','15.0';
        'Cart friction  b',    'p_b','0.05';
        'Joint friction  b_p', 'p_bp','0.002'};
ph = panel_h(numel(rows), L) + 24;
pp = mkpanel(parent,[L.m 520-ph-L.gap 470 ph],'Parameters',C);
for i = 1:size(rows,1)
    addrow(pp, i, rows{i,1}, rows{i,2}, rows{i,3}, L, C, numel(rows));
end
mklbl(pp,'Editable only when preset = custom',[L.p 8 400 18],C,9);

% ---- Position target ---------------------------------------------------
yb = 520-ph-L.gap;
pt = mkpanel(parent,[L.m yb-104-L.gap 470 104],'Position Target',C);
mklbl(pt,'x_ref  —  desired cart position (m)',[L.p 52 260 22],C);
uieditfield(pt,'numeric','Value',0.0,'Position',[L.p+270 50 110 26], ...
    'Tag','xc_ref','BackgroundColor',C.input,'FontColor',C.txt);
mklbl(pt,['After balancing, the governor drives the cart here while ' ...
          'holding the pendulum upright.'],[L.p 18 430 20],C,9);

% ---- Derived quantities ------------------------------------------------
drows = {'omega_n','natural frequency  \omega_n  (rad/s)';
         'E_target','energy target  E_t  (J)';
         'M11','M_{11}  (kg)';
         'M12','M_{12}  (kg·m)';
         'M22','M_{22}  (kg·m²)';
         'det_M','det(M)';
         'Mi21','M^{-1}_{21}   (rad/s²)/N'};
pd = mkpanel(parent,[L.m+480 520-ph-L.gap 490 ph+82+L.gap],'Derived Quantities',C);
ptop = ph+82+L.gap;
for i = 1:size(drows,1)
    y = ptop - 44 - (i-1)*L.row;
    mklbl(pd, drows{i,2}, [L.p y 250 22], C);
    uilabel(pd,'Text','—','Tag',drows{i,1}, ...
        'Position',[L.p+260 y 200 22],'FontColor',C.accent,'FontSize',11);
end

dd.ValueChangedFcn = @(src,~) update_derived(parent, src.Value);
update_derived(parent,'medium');
end


function update_derived(parent, preset)
try
    p = pendulum_params(preset);
    setlbl(parent,'omega_n', sprintf('%.4f',  p.omega_n));
    setlbl(parent,'E_target',sprintf('%.4f J',p.E_target));
    setlbl(parent,'M11',     sprintf('%.4f',  p.M11_eq));
    setlbl(parent,'M12',     sprintf('%.4f',  p.M12_eq));
    setlbl(parent,'M22',     sprintf('%.4f',  p.M22_eq));
    setlbl(parent,'det_M',   sprintf('%.6f',  p.M11_eq*p.M22_eq-p.M12_eq^2));
    setlbl(parent,'Mi21',    sprintf('%.4f',  p.Minv_eq(2,1)));

    custom = strcmp(preset,'custom');
    map = {'p_mc','mc';'p_m','m';'p_l','l';'p_Fmax','Fmax';'p_b','b';'p_bp','bp'};
    for i = 1:size(map,1)
        ef = findobj(parent,'Tag',map{i,1});
        if isempty(ef), continue; end
        if custom
            ef.Editable = 'on';
            ef.BackgroundColor = [0.24 0.26 0.31];
        else
            if isfield(p, map{i,2}), ef.Value = p.(map{i,2}); end
            ef.Editable = 'off';
            ef.BackgroundColor = [0.17 0.18 0.21];
        end
    end
catch
end
end


% =========================================================================
%  TAB 2 — FUZZY SYSTEM
% =========================================================================
function build_tab2(parent, L, C)

% ---- Inference engine --------------------------------------------------
rows = {'Membership function','mf_type','uidropdown', ...
            {'triangular','gaussian','trapezoidal'},'triangular';
        'Number of sets','n_sets','uidropdown',{'5','7','9'},'7';
        'Partition','partition','uidropdown',{'uniform','concentrated'},'uniform';
        'Inference type','inference','uidropdown',{'sugeno','mamdani'},'sugeno';
        'AND operator','and_op','uidropdown',{'min','product'},'min';
        'Defuzzification','defuzz','uidropdown', ...
            {'weighted_avg','centroid','mom'},'weighted_avg'};
ph = panel_h(size(rows,1), L);
pi_ = mkpanel(parent,[L.m 602-ph 470 ph],'Inference Engine',C);
for i = 1:size(rows,1)
    y = rowy(ph, i, L);
    mklbl(pi_, rows{i,1}, [L.p y L.lbl 22], C);
    uidropdown(pi_,'Items',rows{i,4},'Value',rows{i,5}, ...
        'Position',[L.p+L.lbl+10 y-2 170 26],'Tag',rows{i,2}, ...
        'BackgroundColor',C.input,'FontColor',C.txt, ...
        'ValueChangedFcn',@(~,~) draw_mf_preview(parent));
end

% ---- Type-2 ------------------------------------------------------------
yb = 602-ph-L.gap;
p2 = mkpanel(parent,[L.m yb-116 470 116],'Type-2 Fuzzy Logic',C);
mklbl(p2,'Enable Type-2 sets',[L.p 70 L.lbl 22],C);
uicheckbox(p2,'Text','','Value',false,'Tag','type2', ...
    'Position',[L.p+L.lbl+10 69 30 24], ...
    'ValueChangedFcn',@(~,~) draw_mf_preview(parent));
mklbl(p2,'FOU half-width',[L.p 28 L.lbl 22],C);
sl = uislider(p2,'Limits',[0 0.5],'Value',0.20,'Tag','fou_width', ...
    'Position',[L.p+L.lbl+10 40 200 3], ...
    'MajorTicks',[0 0.25 0.5],'MinorTicks',[]);
lb = uilabel(p2,'Text','0.20','Tag','fou_lbl', ...
    'Position',[L.p+L.lbl+225 26 60 22],'FontColor',C.accent,'FontSize',11);
sl.ValueChangedFcn = @(src,~) fou_changed(parent, src, lb);

% ---- Rule base ---------------------------------------------------------
yb2 = yb-116-L.gap;
pr = mkpanel(parent,[L.m yb2-92 470 92],'Rule Base',C);
mklbl(pr,'Rule preset',[L.p 42 L.lbl 22],C);
uidropdown(pr,'Items',{'expert','aggressive','conservative','symmetric'}, ...
    'Value','expert','Position',[L.p+L.lbl+10 40 170 26],'Tag','rule_preset', ...
    'BackgroundColor',C.input,'FontColor',C.txt);
mklbl(pr,'expert · balanced     aggressive · fast     conservative · gentle', ...
    [L.p 12 430 18],C,9);

% ---- MF preview --------------------------------------------------------
pv = mkpanel(parent,[L.m+480 yb2-92 490 602-(yb2-92)],'Membership Preview',C);
pvh = 602-(yb2-92);
ax = uiaxes(pv,'Position',[14 44 462 pvh-90]);
ax.Tag = 'mf_axes';
style_uiaxes(ax);
uilabel(pv,'Text','Updates live as you change the settings on the left.', ...
    'Position',[14 14 460 20],'FontColor',C.dim,'FontSize',9);

draw_mf_preview(parent);
end


function fou_changed(parent, src, lb)
lb.Text = sprintf('%.2f', src.Value);
draw_mf_preview(parent);
end


function draw_mf_preview(parent)
% Draws every membership function across the universe of discourse.
% For Type-2, the upper and lower MFs are drawn as a shaded FOU band.
ax = findobj(parent,'Tag','mf_axes');
if isempty(ax), return; end
cla(ax);

mf_dd = findobj(parent,'Tag','mf_type');
ns_dd = findobj(parent,'Tag','n_sets');
pt_dd = findobj(parent,'Tag','partition');
t2_cb = findobj(parent,'Tag','type2');
fw_sl = findobj(parent,'Tag','fou_width');
if isempty(mf_dd) || isempty(ns_dd), return; end

n_sets    = str2double(ns_dd.Value);
partition = 'uniform';
if ~isempty(pt_dd), partition = pt_dd.Value; end
is_t2 = ~isempty(t2_cb) && t2_cb.Value;
fou_w = 0.20;
if ~isempty(fw_sl), fou_w = fw_sl.Value; end

fcfg = struct('n_sets',n_sets,'range',[-180 180], ...
              'mf_type',mf_dd.Value,'partition',partition, ...
              'type2',false,'fou_width',fou_w);

e = linspace(-180,180,600);
pal = mf_palette(n_sets);
hold(ax,'on');

ok = false;
for si = 1:n_sets
    try
        if is_t2
            f_hi = fcfg; f_hi.type2 = true;  f_hi.fou_width = fou_w;
            [mu_lo, mu_hi] = fp_membership(e, si, f_hi);
            mu_lo = row(mu_lo); mu_hi = row(mu_hi);
            fill(ax,[e fliplr(e)],[mu_hi fliplr(mu_lo)], pal(si,:), ...
                 'FaceAlpha',0.25,'EdgeColor','none');
            plot(ax,e,mu_hi,'Color',pal(si,:),'LineWidth',1.4);
            plot(ax,e,mu_lo,'Color',pal(si,:),'LineWidth',0.9,'LineStyle',':');
        else
            mu = row(fp_membership(e, si, fcfg));
            plot(ax,e,mu,'Color',pal(si,:),'LineWidth',1.6);
        end
        ok = true;
    catch
    end
end
hold(ax,'off');

if ~ok
    text(ax,0,0.5,'preview unavailable','Color',[0.8 0.5 0.5], ...
         'HorizontalAlignment','center');
    return;
end

xlim(ax,[-180 180]); ylim(ax,[0 1.06]);
xticks(ax,-180:60:180);
xlabel(ax,'error  e_\theta  (deg)');
ylabel(ax,'membership  \mu');
if is_t2
    title(ax,sprintf('%s · %d sets · Type-2 (FOU %.2f)', ...
          fcfg.mf_type, n_sets, fou_w));
else
    title(ax,sprintf('%s · %d sets · %s', ...
          fcfg.mf_type, n_sets, partition));
end
end


function v = row(v)
v = reshape(double(v),1,[]);
end

function pal = mf_palette(n)
base = [0.95 0.35 0.30; 0.95 0.62 0.25; 0.92 0.85 0.30;
        0.45 0.85 0.45; 0.30 0.75 0.95; 0.45 0.50 0.95;
        0.75 0.45 0.95; 0.95 0.45 0.70; 0.60 0.85 0.75];
if n <= size(base,1)
    idx = round(linspace(1,size(base,1),n));
    pal = base(idx,:);
else
    pal = jet(n);
end
end


% =========================================================================
%  TAB 3 — CONTROLLERS
% =========================================================================
function build_tab3(parent, L, C)

mklbl(parent,'Tick a controller to include it in the run. Settings apply to that controller only.', ...
    [L.m 608 700 22],C,11);

ctrls = {'FuzzyPID_Sugeno', 'Fuzzy PID — Sugeno';
         'FuzzyPID_Mamdani','Fuzzy PID — Mamdani';
         'FuzzyPID_Type2',  'Fuzzy PID — Type-2 (KM)';
         'FuzzySMC',        'Fuzzy Sliding Mode';
         'AdaptiveFuzzy',   'Adaptive Fuzzy';
         'LQR',             'LQR — augmented, integral x_c';
         'ClassicalPID',    'Classical PID'};

RH = 62;                      % row height per controller
top = 588;
for i = 1:size(ctrls,1)
    y = top - i*RH;
    bx = uipanel(parent,'Position',[L.m y 970 RH-8], ...
        'BackgroundColor',C.panel,'BorderType','line','HighlightColor',C.border);
    uicheckbox(bx,'Text',ctrls{i,2},'Value',true, ...
        'Position',[12 (RH-8)/2-11 230 22],'Tag',['cb_' ctrls{i,1}], ...
        'FontColor',C.txt,'FontSize',10.5);
    sub_controls(bx, ctrls{i,1}, RH-8, C);
end

% ---- Governor ----------------------------------------------------------
gy = top - size(ctrls,1)*RH - 96;
pg = mkpanel(parent,[L.m gy 970 88],'Cart Position Governor (outer loop)',C);
mklbl(pg,'Gain source',[L.p 34 110 22],C);
uidropdown(pg,'Items',{'Auto — derived from LQR K5','Manual'}, ...
    'Value','Auto — derived from LQR K5','Position',[L.p+115 32 230 26], ...
    'Tag','gov_mode','BackgroundColor',C.input,'FontColor',C.txt);
gp = {'k_p  (deg/m)','gov_kp','auto';
      'k_d  (deg·s/m)','gov_kd','auto';
      'max tilt (deg)','gov_max_tilt','auto'};
for i = 1:3
    x = 380 + (i-1)*200;
    mklbl(pg, gp{i,1}, [x 40 130 18], C, 9);
    uieditfield(pg,'text','Value',gp{i,3},'Position',[x 14 130 24], ...
        'Tag',gp{i,2},'BackgroundColor',C.input,'FontColor',C.dim, ...
        'Editable','off','FontSize',9);
end
end


function sub_controls(bx, name, h, C)
% Lays out a controller's parameters on one row, evenly spaced.
yc = h/2 - 11;
switch name
    case {'FuzzyPID_Sugeno','FuzzyPID_Mamdani','FuzzyPID_Type2'}
        f = {'kp_max','300';'kd_max','50';'ki_max','0.1'};
        pref = [name '_'];
    case 'FuzzySMC'
        f = {'lambda','5.0';'phi_b','0.05';'K_min','2.0';'K_max','15.0'};
        pref = 'smc_';
    case 'AdaptiveFuzzy'
        f = {'eta_kp','0.01';'eta_kd','0.005';'eta_ki','0.001'};
        pref = 'adapt_';
    case 'LQR'
        f = {'Q1','0.1';'Q2','0.02';'Q3','200';'Q4','20';'qi','0.1';'R','0.01'};
        pref = 'lqr_';
    case 'ClassicalPID'
        f = {'kp','80';'kd','15';'ki','0.02'};
        pref = 'pid_';
    otherwise
        return;
end

x0 = 258; avail = 700;
nf = size(f,1);
cell_w = floor(avail/nf);
lw = min(70, cell_w-96);
for i = 1:nf
    x = x0 + (i-1)*cell_w;
    uilabel(bx,'Text',f{i,1},'Position',[x yc+1 lw 20], ...
        'FontColor',C.dim,'FontSize',9,'HorizontalAlignment','right');
    uieditfield(bx,'text','Value',f{i,2}, ...
        'Position',[x+lw+8 yc 78 24],'Tag',[pref f{i,1}], ...
        'BackgroundColor',C.input,'FontColor',C.txt,'FontSize',9);
end

% Extra toggles
if strcmp(name,'AdaptiveFuzzy')
    uicheckbox(bx,'Text','freeze rules','Value',false,'Tag','adapt_freeze', ...
        'Position',[x0+avail+6 yc 120 24],'FontColor',C.dim,'FontSize',9);
elseif strcmp(name,'ClassicalPID')
    uicheckbox(bx,'Text','auto-tune (Z-N)','Value',false,'Tag','pid_auto_tune', ...
        'Position',[x0+avail+6 yc 130 24],'FontColor',C.dim,'FontSize',9);
end
end


% =========================================================================
%  TAB 4 — SIMULATION
% =========================================================================
function build_tab4(parent, L, C)

% ---- Swing-up ----------------------------------------------------------
rows = {'Energy gain  k_su','su_k_su','10.0';
        'Backstepping  k_1','su_k1','4.0';
        'Backstepping  k_2','su_k2','10.0';
        'Capture zone (deg)','cap_deg','25.0'};
ph = panel_h(numel(rows),L) + L.row;
ps = mkpanel(parent,[L.m 602-ph 470 ph],'Swing-Up',C);
y0 = rowy(ph,1,L);
mklbl(ps,'Strategy',[L.p y0 L.lbl 22],C);
uidropdown(ps,'Items',{'energy','backstepping','sinusoidal','fuzzy'}, ...
    'Value','energy','Position',[L.p+L.lbl+10 y0-2 170 26],'Tag','swingup', ...
    'BackgroundColor',C.input,'FontColor',C.txt);
for i = 1:size(rows,1)
    addrow(ps, i+1, rows{i,1}, rows{i,2}, rows{i,3}, L, C, numel(rows)+1);
end

% ---- Timing ------------------------------------------------------------
yb = 602-ph-L.gap;
rows2 = {'Duration  t_end  (s)','t_end','25.0';
         'Time step  dt  (s)','dt','0.005'};
ph2 = panel_h(numel(rows2),L);
pt = mkpanel(parent,[L.m yb-ph2 470 ph2],'Timing',C);
for i = 1:size(rows2,1)
    addrow(pt, i, rows2{i,1}, rows2{i,2}, rows2{i,3}, L, C, numel(rows2));
end

% ---- Initial conditions ------------------------------------------------
yb2 = yb-ph2-L.gap;
rows3 = {'x_c0  cart position (m)','xc0','0.0';
         'theta_0  pendulum (deg)','theta0','-178';
         'theta-dot_0  nudge (rad/s)','thetad0','0.05'};
ph3 = panel_h(numel(rows3),L);
pic = mkpanel(parent,[L.m yb2-ph3 470 ph3],'Initial Conditions',C);
for i = 1:size(rows3,1)
    addrow(pic, i, rows3{i,1}, rows3{i,2}, rows3{i,3}, L, C, numel(rows3));
end

% ---- Export ------------------------------------------------------------
rows4 = {'GIF frames per second','gif_fps','10';
         'Max GIF frames','gif_max_frames','400';
         'Figure DPI','dpi','150'};
ph4 = panel_h(numel(rows4),L) + 70;
pe = mkpanel(parent,[L.m+480 602-ph4 490 ph4],'Export',C);
for i = 1:size(rows4,1)
    addrow(pe, i, rows4{i,1}, rows4{i,2}, rows4{i,3}, L, C, numel(rows4)+2);
end
yck = rowy(ph4, numel(rows4)+1, L);
uicheckbox(pe,'Text','Write GIF animations','Value',true,'Tag','make_gifs', ...
    'Position',[L.p yck 250 24],'FontColor',C.txt);
uicheckbox(pe,'Text','Run sensitivity sweep (slow)','Value',false, ...
    'Tag','run_sens','Position',[L.p yck-L.row 280 24],'FontColor',C.txt);

% ---- Rule designer -----------------------------------------------------
pr = mkpanel(parent,[L.m+480 602-ph4-L.gap-80 490 80],'Rule Designer',C);
uicheckbox(pr,'Text','Generate data-driven rule base before running', ...
    'Value',false,'Tag','run_rule_designer', ...
    'Position',[L.p 30 420 24],'FontColor',C.txt);
mklbl(pr,'Overrides the rule preset chosen on the Fuzzy System tab.', ...
    [L.p 8 440 18],C,9);
end


% =========================================================================
%  TAB 5 — DISTURBANCES
% =========================================================================
function build_tab5(parent, L, C)

% ---- Mode --------------------------------------------------------------
pm = mkpanel(parent,[L.m 512 970 90],'Experiment Mode',C);
bg = uibuttongroup(pm,'Position',[L.p 8 560 56], ...
    'BackgroundColor',C.panel,'BorderType','none','Tag','dist_mode');
uiradiobutton(bg,'Text','Warm start — resume from the settled state of the baseline run', ...
    'Position',[6 30 540 22],'FontColor',C.txt,'Value',true);
uiradiobutton(bg,'Text','Fresh start — swing up from hanging, then apply the disturbance', ...
    'Position',[6 6 540 22],'FontColor',C.txt,'Value',false);
mklbl(pm,'Run duration (s)',[600 38 120 20],C,9);
uieditfield(pm,'text','Value','20.0','Position',[600 12 110 24], ...
    'Tag','dist_t_end','BackgroundColor',C.input,'FontColor',C.txt);
mklbl(pm,'Baseline run',[730 38 120 20],C,9);
uieditfield(pm,'text','Value','(latest)','Position',[730 12 150 24], ...
    'Tag','warm_run_dir','BackgroundColor',C.input, ...
    'FontColor',C.txt,'FontSize',9);
uibutton(pm,'Text','…','Position',[886 12 32 24], ...
    'BackgroundColor',C.accent,'FontColor','w', ...
    'ButtonPushedFcn',@(~,~) browse_run_dir(parent));

% ---- Disturbance blocks ------------------------------------------------
defs = { 'kick','Impulse Kick',true, ...
            {'Time (s)','kick_time','3.0';'Magnitude (N)','kick_mag','5.0'; ...
             'Duration (s)','kick_dur','0.1'};
         'wind','Wind Gust',true, ...
            {'Start (s)','wind_start','2.0';'Speed (m/s)','wind_speed','5.0'; ...
             'Hold (s)','wind_hold','3.0'};
         'friction','Track Friction',false, ...
            {'Start (s)','fric_start','2.0';'End (s)','fric_end','4.0'; ...
             'Coulomb (N)','fric_mag','2.0'};
         'mass','Added Bob Mass',false, ...
            {'Time (s)','mass_time','2.0';'Delta m (kg)','mass_delta','0.1'};
         'tilt','Track Inclination',false, ...
            {'Angle (deg)','tilt_angle','2.0';'Phase (rad)','tilt_phase','0'} };

BH = 78;
for i = 1:size(defs,1)
    y = 500 - i*(BH+L.gap);
    bx = uipanel(parent,'Position',[L.m y 970 BH], ...
        'BackgroundColor',C.panel,'BorderType','line','HighlightColor',C.border);
    uicheckbox(bx,'Text',defs{i,2},'Value',defs{i,3},'Tag',['d_' defs{i,1}], ...
        'Position',[14 BH/2-12 200 24],'FontColor',C.txt, ...
        'FontSize',10.5,'FontWeight','bold');
    prm = defs{i,4};
    for j = 1:size(prm,1)
        x = 240 + (j-1)*240;
        uilabel(bx,'Text',prm{j,1},'Position',[x BH-30 130 20], ...
            'FontColor',C.dim,'FontSize',9);
        uieditfield(bx,'text','Value',prm{j,3}, ...
            'Position',[x BH-56 150 24],'Tag',prm{j,2}, ...
            'BackgroundColor',C.input,'FontColor',C.txt,'FontSize',9);
    end
end

mklbl(parent,['Each ticked disturbance becomes a subfolder under ' ...
    '<baseline>/disturbance/ when you press Run Disturbance.'], ...
    [L.m 14 900 20],C,9);
end


% =========================================================================
%  TAB 6 — RUN & RESULTS
% =========================================================================
function s = build_tab6(parent, s, fig, L, C)

% ---- Main run ----------------------------------------------------------
s.run_btn = uibutton(parent,'Text','▶   RUN SIMULATION', ...
    'Position',[L.m 552 250 50], ...
    'BackgroundColor',[0.16 0.55 0.30],'FontColor','w', ...
    'FontSize',14,'FontWeight','bold', ...
    'ButtonPushedFcn',@(~,~) run_simulation(fig));

btns = {'Animate baseline','animate_btn',[0.18 0.40 0.64];
        'Export figures',  'export_btn', [0.36 0.31 0.56];
        'Build report',    'report_btn', [0.20 0.46 0.46]};
for i = 1:size(btns,1)
    x = L.m + 264 + (i-1)*238;
    s.(btns{i,2}) = uibutton(parent,'Text',btns{i,1}, ...
        'Position',[x 552 228 50],'BackgroundColor',btns{i,3}, ...
        'FontColor','w','FontSize',11,'Enable','off','Tag',btns{i,2});
end
s.animate_btn.ButtonPushedFcn = @(~,~) run_post(fig,'animate');
s.export_btn.ButtonPushedFcn  = @(~,~) run_post(fig,'export');
s.report_btn.ButtonPushedFcn  = @(~,~) run_post(fig,'report');

% ---- Disturbance section ------------------------------------------------
pd = mkpanel(parent,[L.m 406 970 134],'Disturbances',C);

s.disturbance_btn = uibutton(pd,'Text','Run disturbance suite', ...
    'Position',[L.p 62 210 40],'BackgroundColor',[0.56 0.36 0.18], ...
    'FontColor','w','FontSize',11,'Enable','off', ...
    'ButtonPushedFcn',@(~,~) run_post(fig,'disturbance'));
mklbl(pd,'Uses the settings on the Disturbances tab.',[L.p 40 230 18],C,9);

% Selector — shared by the animate and export buttons
mklbl(pd,'Select disturbance runs',[250 88 240 18],C,9);
dtypes = {'kick','wind','friction','mass','tilt'};
for i = 1:numel(dtypes)
    x = 250 + (i-1)*104;
    s.(['anim_' dtypes{i}]) = uicheckbox(pd,'Text',dtypes{i},'Value',false, ...
        'Position',[x 60 100 24],'Tag',['anim_' dtypes{i}], ...
        'FontColor',C.dim,'FontSize',9,'Enable','off');
end
s.anim_status = uilabel(pd,'Text','No disturbance runs found yet.', ...
    'Position',[250 34 500 18],'FontColor',C.dim,'FontSize',9);

s.refresh_btn = uibutton(pd,'Text','Refresh list', ...
    'Position',[782 78 170 28],'BackgroundColor',[0.30 0.33 0.40], ...
    'FontColor','w','FontSize',10, ...
    'ButtonPushedFcn',@(~,~) refresh_dist_list(fig));
s.anim_dist_btn = uibutton(pd,'Text','Animate selected', ...
    'Position',[782 46 170 28],'BackgroundColor',[0.18 0.40 0.64], ...
    'FontColor','w','FontSize',10,'Enable','off', ...
    'ButtonPushedFcn',@(~,~) animate_selected(fig));
s.export_dist_btn = uibutton(pd,'Text','Export selected', ...
    'Position',[782 14 170 28],'BackgroundColor',[0.36 0.31 0.56], ...
    'FontColor','w','FontSize',10,'Enable','off', ...
    'ButtonPushedFcn',@(~,~) export_selected(fig));

% ---- Log ----------------------------------------------------------------
s.log_area = uitextarea(parent,'Position',[L.m 206 970 188], ...
    'Value',{'Ready. Configure the tabs, then press Run Simulation.'}, ...
    'Editable','off','BackgroundColor',[0.10 0.11 0.13], ...
    'FontColor',[0.74 0.90 0.76],'FontSize',9,'FontName','Courier New');

% ---- Results table ------------------------------------------------------
s.results_table = uitable(parent,'Position',[L.m 14 970 180], ...
    'ColumnName',{'Controller','Captured','t_capture','t_settle', ...
                  'Overshoot','x_c error (m)','Effort (N·s)','x_c settled'}, ...
    'ColumnWidth',{160,80,90,90,90,100,100,90}, ...
    'BackgroundColor',[0.15 0.16 0.19; 0.17 0.18 0.22], ...
    'FontSize',9,'Data',{});
end


% =========================================================================
%  DISTURBANCE ANIMATION SELECTOR
% =========================================================================
function refresh_dist_list(fig)
% Scans <baseline>/disturbance/ and enables a checkbox for each run found.
s = fig.UserData;
dtypes = {'kick','wind','friction','mass','tilt'};

base = '';
if isfield(s,'last_run_dir') && ~isempty(s.last_run_dir)
    base = s.last_run_dir;
else
    try, base = newest_baseline(); catch, end
end

if isempty(base)
    s.anim_status.Text = 'No baseline run found. Run a simulation first.';
    for i = 1:numel(dtypes)
        s.(['anim_' dtypes{i}]).Enable = 'off';
        s.(['anim_' dtypes{i}]).Value  = false;
    end
    s.anim_dist_btn.Enable   = 'off';
    s.export_dist_btn.Enable = 'off';
    fig.UserData = s;
    return;
end

found = {};
for i = 1:numel(dtypes)
    cb = s.(['anim_' dtypes{i}]);
    mf = fullfile(base,'disturbance',dtypes{i},'results.mat');
    if isfile(mf)
        cb.Enable    = 'on';
        cb.FontColor = [0.88 0.90 0.94];
        found{end+1} = dtypes{i}; %#ok
    else
        cb.Enable    = 'off';
        cb.Value     = false;
        cb.FontColor = [0.45 0.47 0.52];
    end
end

if isempty(found)
    s.anim_status.Text = sprintf('No disturbance runs in %s', ...
        fullfile(base,'disturbance'));
    s.anim_dist_btn.Enable   = 'off';
    s.export_dist_btn.Enable = 'off';
else
    s.anim_status.Text = sprintf('Available in %s:  %s', ...
        base, strjoin(found,', '));
    s.anim_dist_btn.Enable   = 'on';
    s.export_dist_btn.Enable = 'on';
end
s.dist_base = base;
fig.UserData = s;
end


% =========================================================================
%  WHICH DISTURBANCES ARE TICKED (and actually available)
% =========================================================================
function [sel, base] = selected_dist(fig)
s = fig.UserData;
dtypes = {'kick','wind','friction','mass','tilt'};
sel  = {};
base = '';
if isfield(s,'dist_base'), base = s.dist_base; end
for i = 1:numel(dtypes)
    cb = s.(['anim_' dtypes{i}]);
    if strcmp(cb.Enable,'on') && cb.Value
        sel{end+1} = dtypes{i}; %#ok
    end
end
end


% =========================================================================
%  EXPORT FIGURES FOR SELECTED DISTURBANCE RUNS
% =========================================================================
function export_selected(fig)
s = fig.UserData;
[sel, base] = selected_dist(fig);

if isempty(base)
    set_status(fig,'Press Refresh list first.','red'); return;
end
if isempty(sel)
    set_status(fig,'Tick at least one disturbance to export.','red'); return;
end

eopts = struct('dpi',           getnum(fig,'dpi',150), ...
               'make_gifs',     false, ...
               'gif_fps',       getnum(fig,'gif_fps',10), ...
               'gif_max_frames',getnum(fig,'gif_max_frames',400));

done = {};
for i = 1:numel(sel)
    rd = fullfile(base,'disturbance',sel{i});
    set_status(fig, sprintf('Exporting figures — %s ...', sel{i}),'yellow');
    append_log(s, sprintf('Export figures: %s', rd));
    drawnow;
    try
        D = load(fullfile(rd,'results.mat'),'results','cfg');
        if ~isfield(D,'results') || ~isfield(D,'cfg')
            error('results.mat missing "results"/"cfg"');
        end
        fp_export_figures(rd, D.results, D.cfg, eopts);
        done{end+1} = sel{i}; %#ok
        append_log(s, sprintf('  -> %s', fullfile(rd,'figures')));
    catch ME
        append_log(s, sprintf('  ERROR (%s): %s', sel{i}, ME.message));
    end
end

if isempty(done)
    set_status(fig,'Export failed — see log.','red');
else
    set_status(fig, sprintf('Figures exported: %s', strjoin(done,', ')),'green');
end
end


function animate_selected(fig)
s = fig.UserData;
[sel, base] = selected_dist(fig);

if isempty(base)
    set_status(fig,'Press Refresh list first.','red'); return;
end
if isempty(sel)
    set_status(fig,'Tick at least one disturbance to animate.','red'); return;
end

done = {};
for i = 1:numel(sel)
    rd = fullfile(base,'disturbance',sel{i});
    set_status(fig, sprintf('Animating — %s ...', sel{i}),'yellow');
    append_log(s, sprintf('Animate: %s', rd));
    drawnow;
    try
        aopts = struct('web_gif',true, ...
                       'web_dir',fullfile('docs','assets','anim'));
        animate_pendulum(rd, 'all', aopts);
        done{end+1} = sel{i}; %#ok
    catch ME
        append_log(s, sprintf('  ERROR (%s): %s', sel{i}, ME.message));
    end
end

if isempty(done)
    set_status(fig,'Animation failed — see log.','red');
else
    set_status(fig, sprintf('Animated: %s', strjoin(done,', ')),'green');
end
end


function b = newest_baseline()
dirs = dir('Pendulum_Control_Run_*');
dirs = dirs([dirs.isdir]);
nums = nan(1,numel(dirs));
for i = 1:numel(dirs)
    tok = regexp(dirs(i).name,'^Pendulum_Control_Run_(\d+)$','tokens','once');
    if ~isempty(tok), nums(i) = str2double(tok{1}); end
end
keep = ~isnan(nums); dirs = dirs(keep); nums = nums(keep);
if isempty(dirs), error('none'); end
[~,o] = sort(nums,'descend'); dirs = dirs(o);
for i = 1:numel(dirs)
    mf = fullfile(dirs(i).name,'results.mat');
    if isfile(mf) && ismember('results', who('-file',mf))
        b = dirs(i).name; return;
    end
end
error('none');
end


% =========================================================================
%  RUN CALLBACKS
% =========================================================================
function run_simulation(fig)
s = fig.UserData;
set_status(fig,'Building configuration...','yellow');
append_log(s,'=== Run started ===');

try
    cfg = collect_config(fig);
catch ME
    set_status(fig,['Config error: ' ME.message],'red');
    append_log(s,['CONFIG ERROR: ' ME.message]);
    return;
end

run_num = next_run_number();
run_dir = sprintf('Pendulum_Control_Run_%03d', run_num);
mkdir(run_dir);
mkdir(fullfile(run_dir,'figures'));
mkdir(fullfile(run_dir,'animation'));
cfg.run_dir = run_dir;
append_log(s,['Run folder: ' run_dir]);

set_status(fig,'Simulating...','yellow'); drawnow;
try
    results = fp_run_analysis(cfg);
catch ME
    set_status(fig,['Simulation error: ' ME.message],'red');
    append_log(s,['SIM ERROR: ' ME.message]);
    return;
end

save(fullfile(run_dir,'results.mat'),'results','cfg','-v7.3');
append_log(s,'Results saved.');

try
    fp_write_summary(run_dir, run_num, results, cfg);
    append_log(s,'Summary written.');
catch ME
    append_log(s,['Summary warning: ' ME.message]);
end

populate_table(s, results);

s.animate_btn.Enable     = 'on';
s.export_btn.Enable      = 'on';
s.disturbance_btn.Enable = 'on';
s.report_btn.Enable      = 'on';
s.last_run_dir = run_dir;
s.last_results = results;
s.last_cfg     = cfg;
fig.UserData   = s;

refresh_dist_list(fig);

set_status(fig,['Complete — ' run_dir],'green');
append_log(s,'=== Run complete ===');
end


function run_post(fig, action)
s = fig.UserData;
if ~isfield(s,'last_run_dir')
    set_status(fig,'Run a simulation first.','red'); return;
end
set_status(fig,[action ' running...'],'yellow'); drawnow;
try
    switch action
        case 'animate'
            % Also emit web-sized copies for the two controllers the report
            % publishes, so fp_export_web has nothing left to re-encode.
            aopts = struct('web_gif',true, ...
                           'web_dir',fullfile('docs','assets','anim'));
            animate_pendulum(s.last_run_dir,'all',aopts);
        case 'export'
            eopts = struct('dpi',getnum(fig,'dpi',150), ...
                           'make_gifs',getchk(fig,'make_gifs'), ...
                           'gif_fps',getnum(fig,'gif_fps',10), ...
                           'gif_max_frames',getnum(fig,'gif_max_frames',400));
            fp_export_figures(s.last_run_dir, s.last_results, s.last_cfg, eopts);
        case 'disturbance'
            dopts = collect_disturbance_opts(fig);
            append_log(s, sprintf('Disturbance suite: %s', ...
                strjoin(enabled_list(dopts), ', ')));
            run_disturbance(dopts);
            refresh_dist_list(fig);
        case 'report'
            fp_build_report();
    end
    set_status(fig,[action ' complete.'],'green');
catch ME
    set_status(fig,[action ' error: ' ME.message],'red');
    append_log(s,['ERROR (' action '): ' ME.message]);
end
end


function names = enabled_list(d)
names = {};
if d.test_kick,     names{end+1} = 'kick';     end
if d.test_wind,     names{end+1} = 'wind';     end
if d.test_friction, names{end+1} = 'friction'; end
if d.test_mass,     names{end+1} = 'mass';     end
if d.test_tilt,     names{end+1} = 'tilt';     end
if isempty(names),  names = {'(none selected)'}; end
end


% =========================================================================
%  DISTURBANCE OPTIONS FROM THE GUI
% =========================================================================
function d = collect_disturbance_opts(fig)
% Reads the Disturbances tab and returns the opts struct run_disturbance
% expects. This is what makes the GUI actually drive the suite.

d = struct();

d.test_kick     = getchk(fig,'d_kick');
d.test_wind     = getchk(fig,'d_wind');
d.test_friction = getchk(fig,'d_friction');
d.test_mass     = getchk(fig,'d_mass');
d.test_tilt     = getchk(fig,'d_tilt');

d.kick_time      = getnum(fig,'kick_time',3.0);
d.kick_mag       = getnum(fig,'kick_mag',5.0);
d.kick_duration  = getnum(fig,'kick_dur',0.1);

d.wind_start     = getnum(fig,'wind_start',2.0);
d.wind_speed     = getnum(fig,'wind_speed',5.0);
d.wind_hold      = getnum(fig,'wind_hold',3.0);
d.wind_ramp      = 0.5;

d.fric_start     = getnum(fig,'fric_start',2.0);
d.fric_end       = getnum(fig,'fric_end',4.0);
d.fric_mag       = getnum(fig,'fric_mag',2.0);
d.fric_visc      = 0.5;

d.mass_time      = getnum(fig,'mass_time',2.0);
d.mass_delta     = getnum(fig,'mass_delta',0.1);

d.tilt_angle_deg = getnum(fig,'tilt_angle',2.0);
d.tilt_phase     = getnum(fig,'tilt_phase',0);

d.t_disturbance  = getnum(fig,'dist_t_end',20.0);
d.export_figures = false;

% Warm vs fresh start
d.warm_start = true;
bg = findobj(fig,'Tag','dist_mode');
if ~isempty(bg)
    kids = bg.Children;
    for i = 1:numel(kids)
        if isa(kids(i),'matlab.ui.control.RadioButton') && kids(i).Value
            d.warm_start = contains(lower(kids(i).Text),'warm');
        end
    end
end

% Baseline folder
wd = getstr(fig,'warm_run_dir');
if isempty(wd) || strcmpi(strtrim(wd),'(latest)')
    d.base_run_dir = '';
else
    d.base_run_dir = strtrim(wd);
end
end


% =========================================================================
%  MAIN CONFIG FROM THE GUI
% =========================================================================
function cfg = collect_config(fig)

preset = getstr(fig,'preset_dd');
p = pendulum_params(preset);
if strcmp(preset,'custom')
    p.mc   = getnum(fig,'p_mc',2.5);
    p.m    = getnum(fig,'p_m',0.20);
    p.l    = getnum(fig,'p_l',0.50);
    p.Fmax = getnum(fig,'p_Fmax',15.0);
    p.b    = getnum(fig,'p_b',0.05);
    p.bp   = getnum(fig,'p_bp',0.002);
    p.omega_n  = sqrt(p.g/p.l);
    p.E_target = p.m*p.g*p.l;
    M11 = p.mc+p.m; M12 = p.m*p.l; M22 = p.m*p.l^2;
    dM  = M11*M22 - M12^2;
    p.M11_eq = M11; p.M12_eq = M12; p.M22_eq = M22;
    p.Minv_eq = [M22,-M12;-M12,M11]/dM;
end
p.xc_ref      = getnum(fig,'xc_ref',0.0);
p.capture_deg = getnum(fig,'cap_deg',25.0);
p.capture_vel = 2.0;

fcfg = struct('n_sets',    str2double(getstr(fig,'n_sets')), ...
              'range',     [-180 180], ...
              'mf_type',   getstr(fig,'mf_type'), ...
              'partition', getstr(fig,'partition'), ...
              'inference', getstr(fig,'inference'), ...
              'and_op',    getstr(fig,'and_op'), ...
              'defuzz',    getstr(fig,'defuzz'), ...
              'type2',     getchk(fig,'type2'), ...
              'fou_width', getnum(fig,'fou_width',0.20));

rb = fp_rule_base(getstr(fig,'rule_preset'), fcfg.n_sets);

all_c = {'FuzzyPID_Sugeno','FuzzyPID_Mamdani','FuzzyPID_Type2', ...
         'FuzzySMC','AdaptiveFuzzy','LQR','ClassicalPID'};
controllers = {};
for i = 1:numel(all_c)
    if getchk(fig,['cb_' all_c{i}]), controllers{end+1} = all_c{i}; end %#ok
end
if isempty(controllers), error('Select at least one controller.'); end

gcfg = struct('kp_max',getnum(fig,'FuzzyPID_Sugeno_kp_max',300), ...
              'kd_max',getnum(fig,'FuzzyPID_Sugeno_kd_max',50), ...
              'ki_max',getnum(fig,'FuzzyPID_Sugeno_ki_max',0.1), ...
              'kp_min',0,'kd_min',0,'ki_min',0,'use_lpf',false);

lqr_Q  = [getnum(fig,'lqr_Q1',0.1), getnum(fig,'lqr_Q2',0.02), ...
          getnum(fig,'lqr_Q3',200), getnum(fig,'lqr_Q4',20)];
lqr_qi = getnum(fig,'lqr_qi',0.1);
lqr_R  = getnum(fig,'lqr_R',0.01);

pidcfg = struct('kp',getnum(fig,'pid_kp',80), ...
                'kd',getnum(fig,'pid_kd',15), ...
                'ki',getnum(fig,'pid_ki',0.02), ...
                'auto_tune',getchk(fig,'pid_auto_tune'),'i_clamp',500);

smccfg = struct('lambda',      getnum(fig,'smc_lambda',5.0), ...
                'phi_boundary',getnum(fig,'smc_phi_b',0.05), ...
                'K_min',       getnum(fig,'smc_K_min',2.0), ...
                'K_max',       getnum(fig,'smc_K_max',15.0));

adapt_cfg = struct('eta_kp',getnum(fig,'adapt_eta_kp',0.01), ...
                   'eta_kd',getnum(fig,'adapt_eta_kd',0.005), ...
                   'eta_ki',getnum(fig,'adapt_eta_ki',0.001), ...
                   'freeze',getchk(fig,'adapt_freeze'), ...
                   'kp_bounds',[0 500],'kd_bounds',[0 100],'ki_bounds',[0 0.5]);

govcfg = struct();
if strcmpi(getstr(fig,'gov_mode'),'Manual')
    v = getnum(fig,'gov_kp',NaN);        if ~isnan(v), govcfg.kp_gov   = v; end
    v = getnum(fig,'gov_kd',NaN);        if ~isnan(v), govcfg.kd_gov   = v; end
    v = getnum(fig,'gov_max_tilt',NaN);  if ~isnan(v), govcfg.max_tilt = v; end
end

sucfg = struct('k_su',getnum(fig,'su_k_su',10.0), ...
               'k1',  getnum(fig,'su_k1',4.0), ...
               'k2',  getnum(fig,'su_k2',10.0), ...
               'E_thresh',0.05,'omega',0.8*p.omega_n, ...
               'F_amp',0.85*p.Fmax,'phase_aware',true);

x0 = [getnum(fig,'xc0',0); 0; ...
      deg2rad(getnum(fig,'theta0',-178)); getnum(fig,'thetad0',0.05)];

% Baseline run: no disturbances (those belong to the disturbance suite)
disturbance = struct( ...
    'kick_enabled',false,'kick_time',8,'kick_mag',5,'kick_duration',0.1, ...
    'wind_enabled',false,'wind_start',10,'wind_ramp',0.5, ...
    'wind_hold',3,'wind_speed',5, ...
    'friction_enabled',false,'friction_start',5,'friction_end',7, ...
    'friction_mag',2,'friction_visc',0.5, ...
    'mass_enabled',false,'mass_time',12,'mass_delta',0.1, ...
    'tilt_enabled',false,'tilt_angle_deg',2,'tilt_phase',0);

cfg = struct();
cfg.p            = p;
cfg.fcfg         = fcfg;
cfg.rule_base    = rb;
cfg.swingup      = getstr(fig,'swingup');
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
cfg.t_end        = getnum(fig,'t_end',25.0);
cfg.dt           = getnum(fig,'dt',0.005);
cfg.x0           = x0;
cfg.chk_interval = 1000;
cfg.run_dir      = '';
end


% =========================================================================
%  LAYOUT / STYLE HELPERS
% =========================================================================
function L = layout()
L.m    = 14;    % outer margin
L.p    = 14;    % panel padding
L.gap  = 12;    % gap between panels
L.row  = 34;    % row pitch
L.lbl  = 210;   % label width
L.fld  = 130;   % field width
L.fldh = 26;    % field height
end

function C = colors()
C.bg     = [0.13 0.14 0.17];
C.tab    = [0.155 0.165 0.195];
C.panel  = [0.185 0.195 0.225];
C.input  = [0.235 0.250 0.285];
C.txt    = [0.88 0.90 0.94];
C.dim    = [0.62 0.66 0.72];
C.accent = [0.38 0.74 1.00];
C.border = [0.30 0.32 0.37];
C.header = [0.17 0.37 0.60];
C.footer = [0.11 0.12 0.14];
end

function h = panel_h(n_rows, L)
h = n_rows*L.row + 44;
end

function y = rowy(panel_height, i, L)
y = panel_height - 40 - (i-1)*L.row;
end

function addrow(parent, i, label, tag, value, L, C, n_rows)
h = panel_h(n_rows, L);
y = rowy(h, i, L);
uilabel(parent,'Text',label,'Position',[L.p y L.lbl 22], ...
    'FontColor',C.txt,'FontSize',10);
uieditfield(parent,'text','Value',value, ...
    'Position',[L.p+L.lbl+10 y-2 L.fld L.fldh],'Tag',tag, ...
    'BackgroundColor',C.input,'FontColor',C.txt);
end

function p = mkpanel(parent, pos, title_str, C)
p = uipanel(parent,'Position',pos,'Title',['  ' title_str '  '], ...
    'BackgroundColor',C.panel,'ForegroundColor',C.accent, ...
    'BorderType','line','HighlightColor',C.border, ...
    'FontSize',10.5,'FontWeight','bold');
end

function lbl = mklbl(parent, txt, pos, C, fsz)
if nargin < 5, fsz = 10; end
col = C.txt; if fsz < 10, col = C.dim; end
lbl = uilabel(parent,'Text',txt,'Position',pos,'FontColor',col,'FontSize',fsz);
end

function style_uiaxes(ax)
ax.Color           = [0.11 0.12 0.14];
ax.XColor          = [0.70 0.74 0.80];
ax.YColor          = [0.70 0.74 0.80];
ax.GridColor       = [0.28 0.30 0.35];
ax.Title.Color     = [0.92 0.94 0.97];
ax.XGrid = 'on'; ax.YGrid = 'on';
ax.FontSize = 9;
ax.Box = 'on';
end


% =========================================================================
%  VALUE ACCESS HELPERS
% =========================================================================
function v = getstr(fig, tag)
o = findobj(fig,'Tag',tag);
if isempty(o), v = ''; return; end
if isprop(o,'Value'), v = o.Value; else, v = ''; end
if ~ischar(v) && ~isstring(v), v = ''; end
v = char(v);
end

function v = getnum(fig, tag, default)
if nargin < 3, default = 0; end
o = findobj(fig,'Tag',tag);
if isempty(o), v = default; return; end
raw = o.Value;
if isnumeric(raw), v = double(raw); else, v = str2double(raw); end
if isnan(v), v = default; end
end

function v = getchk(fig, tag)
o = findobj(fig,'Tag',tag);
if isempty(o), v = false; return; end
v = logical(o.Value);
end

function setlbl(parent, tag, txt)
o = findobj(parent,'Tag',tag);
if ~isempty(o), o.Text = txt; end
end

function n = next_run_number()
d = dir('Pendulum_Control_Run_*');
d = d([d.isdir]);
n = 1;
if ~isempty(d)
    nums = zeros(1,numel(d));
    for i = 1:numel(d)
        tok = regexp(d(i).name,'Pendulum_Control_Run_(\d+)','tokens');
        if ~isempty(tok), nums(i) = str2double(tok{1}{1}); end
    end
    n = max(nums) + 1;
end
end

function browse_run_dir(parent)
d = uigetdir(pwd,'Select baseline run folder');
if ischar(d) && ~isempty(d)
    ef = findobj(parent,'Tag','warm_run_dir');
    if ~isempty(ef)
        [~,nm] = fileparts(d);
        ef.Value = nm;
    end
end
end

function populate_table(s, results)
keys = fieldnames(results.runs);
data = cell(numel(keys),8);
for i = 1:numel(keys)
    k = keys{i}; r = results.runs.(k); m = r.metrics;
    data(i,:) = {k, tf(r.captured), num(m,'t_capture','%.2f'), ...
                 num(m,'t_settle','%.2f'), num(m,'overshoot_deg','%.1f'), ...
                 num(m,'xc_final_error','%.3f'), ...
                 num(m,'total_effort','%.1f'), tfq(m,'xc_settled')};
end
s.results_table.Data = data;
end

function out = num(m, f, fmt)
if isfield(m,f) && ~isnan(m.(f)), out = sprintf(fmt, m.(f)); else, out = '—'; end
end
function out = tf(v)
if v, out = 'yes'; else, out = 'no'; end
end
function out = tfq(m, f)
if isfield(m,f) && m.(f), out = 'yes'; else, out = 'no'; end
end

function set_status(fig, msg, col)
s = fig.UserData;
if ~isfield(s,'status_lbl'), return; end
s.status_lbl.Text = msg;
switch col
    case 'green',  s.status_lbl.FontColor = [0.42 0.88 0.50];
    case 'yellow', s.status_lbl.FontColor = [1.00 0.84 0.25];
    case 'red',    s.status_lbl.FontColor = [0.92 0.38 0.35];
    otherwise,     s.status_lbl.FontColor = [0.70 0.80 0.90];
end
drawnow;
end

function append_log(s, msg)
if ~isfield(s,'log_area'), return; end
cur = s.log_area.Value;
if ischar(cur), cur = {cur}; end
s.log_area.Value = [cur; {msg}];
scroll(s.log_area,'bottom');
drawnow;
end
