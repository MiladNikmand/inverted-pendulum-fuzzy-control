function metrics = fp_metrics(t, x, F_hist, p, cfg)
% FP_METRICS
% Computes performance metrics for one controller run.
%
% Inputs:
%   t      — Nx1 time vector (s)
%   x      — Nx4 state matrix [xc, xc_dot, theta, theta_dot]
%   F_hist — Nx1 control force history (N)
%   p      — pendulum_params struct (p.xc_ref = desired cart position)
%   cfg    — simulation config
%
% THETA METRICS (stabilisation quality):
%   t_capture            time to enter capture zone (s)
%   t_settle             time from capture until |theta| < 1 deg sustained (s)
%   overshoot_deg        max |theta| after capture (deg)
%   steady_state_err_deg mean |theta| over last 20% of run (deg)
%   rms_theta_deg        RMS theta over stabilisation phase (deg)
%
% POSITION METRICS (cart regulation quality):
%   xc_final             cart position at end of run (m)
%   xc_final_error       |xc_final - xc_ref| (m)
%   xc_rms               RMS of (xc - xc_ref) over stabilisation phase (m)
%   t_xc_settle          time from capture until |xc - xc_ref| < 0.05m sustained (s)
%   xc_settled           logical — cart reached and held xc_ref
%
% EFFORT METRICS:
%   total_effort         integral |F| dt (N*s)
%   rms_F                RMS force (N)
%   max_F                peak force (N)
%   F_sat_fraction       fraction of steps at saturation

theta_deg = rad2deg(x(:,3));
theta_dot = x(:,4);
xc        = x(:,1);
cap_deg   = p.capture_deg;
xc_ref    = 0;
if isfield(p,'xc_ref'), xc_ref = p.xc_ref; end

% =========================================================================
%  CAPTURE TIME
% =========================================================================
t_capture = NaN; captured = false; k_capture = NaN;
for k = 1:numel(t)
    if abs(theta_deg(k)) < cap_deg
        t_capture = t(k); k_capture = k; captured = true; break;
    end
end

% =========================================================================
%  SWING-UP METRICS
% =========================================================================
if captured && k_capture > 1
    theta_pre           = x(1:k_capture, 3);
    zc                  = sum(diff(sign(theta_pre)) ~= 0);
    n_swingup_cycles    = floor(zc / 2);
    max_xcdot_swingup   = max(abs(x(1:k_capture, 2)));
else
    n_swingup_cycles  = NaN;
    max_xcdot_swingup = NaN;
end
t_swingup = t_capture;

% =========================================================================
%  THETA STABILISATION METRICS
% =========================================================================
if captured
    k_stab      = k_capture:numel(t);
    theta_stab  = theta_deg(k_stab);
    thetad_stab = theta_dot(k_stab);
    t_stab      = t(k_stab);

    overshoot_deg = max(abs(theta_stab));

    % Settling: |theta| < 1 deg and never exceeds 1 deg again
    t_settle = NaN;
    for k = 1:numel(t_stab)
        if all(abs(theta_stab(k:end)) < 1.0)
            t_settle = t_stab(k) - t_capture;
            break;
        end
    end

    n_last               = max(1, floor(0.2 * numel(t_stab)));
    steady_state_err_deg = mean(abs(theta_stab(end-n_last+1:end)));
    rms_theta_deg        = sqrt(mean(theta_stab.^2));
    rms_thetadot         = sqrt(mean(thetad_stab.^2));
else
    overshoot_deg        = NaN;
    t_settle             = NaN;
    steady_state_err_deg = NaN;
    rms_theta_deg        = NaN;
    rms_thetadot         = NaN;
end

% =========================================================================
%  POSITION REGULATION METRICS
% =========================================================================
xc_err = xc - xc_ref;

xc_final       = xc(end);
xc_final_error = abs(xc_final - xc_ref);

if captured
    xc_stab  = xc_err(k_stab);
    t_stab_v = t(k_stab);

    % RMS cart position error over stabilisation phase
    xc_rms = sqrt(mean(xc_stab.^2));

    % Cart settling: |xc - xc_ref| < 0.05m and never exceeds 0.05m again
    % Only computed from halfway through the run to avoid penalising
    % the initial drift during the balance phase (before governor activates)
    xc_settle_thresh = 0.05;   % 5 cm
    t_xc_settle = NaN;
    xc_settled  = false;

    % Only look for xc settling in second half of stabilisation
    k_half = max(1, floor(numel(k_stab)/2));
    for k = k_half:numel(t_stab_v)
        if all(abs(xc_stab(k:end)) < xc_settle_thresh)
            t_xc_settle = t_stab_v(k) - t_capture;
            xc_settled  = true;
            break;
        end
    end
else
    xc_rms      = NaN;
    t_xc_settle = NaN;
    xc_settled  = false;
end

% =========================================================================
%  CONTROL EFFORT METRICS
% =========================================================================
rms_F          = sqrt(mean(F_hist.^2));
total_effort   = trapz(t, abs(F_hist));
max_F          = max(abs(F_hist));
F_sat_fraction = mean(abs(F_hist) >= 0.99 * p.Fmax);

% =========================================================================
%  ENERGY HISTORY
% =========================================================================
E_hist = 0.5 * p.m * p.l^2 * x(:,4).^2 + p.m * p.g * p.l * cos(x(:,3));

% =========================================================================
%  PACK METRICS STRUCT
% =========================================================================
metrics = struct();

% Timing
metrics.t_capture           = t_capture;
metrics.captured            = captured;
metrics.t_swingup           = t_swingup;
metrics.n_swingup_cycles    = n_swingup_cycles;
metrics.max_xcdot_swingup   = max_xcdot_swingup;

% Theta stabilisation
metrics.t_settle             = t_settle;
metrics.overshoot_deg        = overshoot_deg;
metrics.steady_state_err_deg = steady_state_err_deg;
metrics.rms_theta_deg        = rms_theta_deg;
metrics.rms_thetadot         = rms_thetadot;

% Position regulation
metrics.xc_ref          = xc_ref;
metrics.xc_final        = xc_final;
metrics.xc_final_error  = xc_final_error;
metrics.xc_rms          = xc_rms;
metrics.t_xc_settle     = t_xc_settle;
metrics.xc_settled      = xc_settled;

% Control effort
metrics.rms_F          = rms_F;
metrics.total_effort   = total_effort;
metrics.max_F          = max_F;
metrics.F_sat_fraction = F_sat_fraction;

% Time histories (for figures)
metrics.E_hist         = E_hist;
metrics.E_target       = p.E_target;
metrics.E_final        = E_hist(end);
metrics.theta_hist     = theta_deg;
metrics.theta_dot_hist = theta_dot;
metrics.xc_hist        = xc;
metrics.xc_err_hist    = xc_err;

end
