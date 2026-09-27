function [theta_ref_deg, info] = ctrl_xc_governor(x, govcfg, p)
% CTRL_XC_GOVERNOR
% Fuzzy reference governor — outer slow loop for cart position regulation.
%
% CONTROL LAW:
%   theta_ref = -kp_gov*(xc - xc_ref) - kd_gov*xc_dot
%   theta_ref = clamp(theta_ref, -max_tilt, +max_tilt)
%
% INTEGRATOR:
%   No integral term in the governor itself. The integrator for LQR
%   is maintained in fp_run_analysis (p.lqr_xi). The governor is a
%   pure PD outer loop — adding integral here caused cart rotation
%   (wind-up during the balance phase before the governor activated).
%
% The PD law is sufficient because:
%   - At steady state xc_dot=0, so theta_ref = -kp_gov*(xc-xc_ref)
%   - A nonzero theta_ref tilts the pendulum, which via gravity creates
%     a horizontal force that continues to drive the cart toward xc_ref
%   - Once xc=xc_ref: theta_ref=0, controller returns to pure balancing
%   - Residual steady-state error is eliminated by the LQR xi integrator

xc     = x(1);
xc_dot = x(2);

xc_ref = 0;
if isfield(p,'xc_ref'), xc_ref = p.xc_ref; end

% ---- Governor gains (from govcfg, with fallback defaults) --------------
% Defaults computed from Minv_eq to give a reasonable starting point.
% fp_run_analysis overrides these with K5-derived values.
kp_gov_def = abs(p.Minv_eq(2,1)) * 6.5;    % ~5.2 * Minv ratio
kd_gov_def = abs(p.Minv_eq(2,1)) * 11.8;   % ~9.4 * Minv ratio
max_tilt_def = 10.0;                         % deg

kp_gov   = kp_gov_def;   if isfield(govcfg,'kp_gov'),   kp_gov   = govcfg.kp_gov;   end
kd_gov   = kd_gov_def;   if isfield(govcfg,'kd_gov'),   kd_gov   = govcfg.kd_gov;   end
max_tilt = max_tilt_def; if isfield(govcfg,'max_tilt'), max_tilt = govcfg.max_tilt; end

% ---- Governor PD law ---------------------------------------------------
xc_err       = xc - xc_ref;
theta_ref_raw = -kp_gov * xc_err - kd_gov * xc_dot;
theta_ref_deg = max(-max_tilt, min(max_tilt, theta_ref_raw));

% ---- Info struct -------------------------------------------------------
info = struct();
info.xc_err       = xc_err;
info.theta_ref_raw= theta_ref_raw;
info.theta_ref_deg= theta_ref_deg;
info.kp_gov       = kp_gov;
info.kd_gov       = kd_gov;
info.max_tilt     = max_tilt;
info.xc_ref       = xc_ref;
info.label        = 'XC Reference Governor (PD outer loop)';

end
