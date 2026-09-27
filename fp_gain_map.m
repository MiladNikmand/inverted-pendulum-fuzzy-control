function [kp, kd, ki] = fp_gain_map(kp_raw, kd_raw, ki_raw, gcfg)
% FP_GAIN_MAP
% Maps raw fuzzy output (k_prime consequents) to actual PID gains.
%
% The fuzzy system outputs "raw" gain factors which are then scaled,
% saturated, and optionally low-pass filtered to produce smooth, physically
% bounded gains for the PID controller.
%
% The mapping follows the original project convention:
%   kp_actual = kp_base * kp_raw
%   kd_actual = kd_base * kd_raw
%   ki_actual = ki_base * ki_raw
%
% An optional low-pass filter smooths gain transitions to avoid
% high-frequency chattering when the pendulum crosses MF boundaries.
%
% Inputs:
%   kp_raw, kd_raw, ki_raw — raw fuzzy outputs from fp_inference
%   gcfg  — gain config struct:
%     .kp_max     max proportional gain (N / deg)   default 300
%     .kd_max     max derivative gain   (N*s / deg) default 50
%     .ki_max     max integral gain     (N / (deg*s)) default 0.1
%     .kp_min     min kp (>= 0)        default 0
%     .kd_min     min kd (>= 0)        default 0
%     .ki_min     min ki (>= 0)        default 0
%     .use_lpf    logical — apply low-pass filter  default false
%     .alpha_lpf  LPF coefficient [0,1]: 0=no filter, 0.9=heavy filter
%                 y_n = alpha*y_{n-1} + (1-alpha)*u_n
%     .kp_prev, .kd_prev, .ki_prev — previous filtered values (for LPF)
%
% Outputs:
%   kp, kd, ki — final PID gains (same units as kp_raw but bounded)

if nargin < 4 || isempty(gcfg)
    gcfg = struct();
end

% Defaults
if ~isfield(gcfg,'kp_max'),    gcfg.kp_max   = 300;   end
if ~isfield(gcfg,'kd_max'),    gcfg.kd_max   = 50;    end
if ~isfield(gcfg,'ki_max'),    gcfg.ki_max   = 0.1;   end
if ~isfield(gcfg,'kp_min'),    gcfg.kp_min   = 0;     end
if ~isfield(gcfg,'kd_min'),    gcfg.kd_min   = 0;     end
if ~isfield(gcfg,'ki_min'),    gcfg.ki_min   = 0;     end
if ~isfield(gcfg,'use_lpf'),   gcfg.use_lpf  = false; end
if ~isfield(gcfg,'alpha_lpf'), gcfg.alpha_lpf= 0.7;   end
if ~isfield(gcfg,'kp_prev'),   gcfg.kp_prev  = kp_raw;end
if ~isfield(gcfg,'kd_prev'),   gcfg.kd_prev  = kd_raw;end
if ~isfield(gcfg,'ki_prev'),   gcfg.ki_prev  = ki_raw;end

% ---- Clamp to physical limits ------------------------------------------
kp = max(gcfg.kp_min, min(gcfg.kp_max, kp_raw));
kd = max(gcfg.kd_min, min(gcfg.kd_max, kd_raw));
ki = max(gcfg.ki_min, min(gcfg.ki_max, ki_raw));

% ---- Optional low-pass filter ------------------------------------------
if gcfg.use_lpf
    a  = gcfg.alpha_lpf;
    kp = a * gcfg.kp_prev + (1-a) * kp;
    kd = a * gcfg.kd_prev + (1-a) * kd;
    ki = a * gcfg.ki_prev + (1-a) * ki;
end

end
