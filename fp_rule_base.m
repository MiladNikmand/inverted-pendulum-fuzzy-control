function rb = fp_rule_base(preset, n_sets)
% FP_RULE_BASE
% Returns a fuzzy rule base (kp, kd, ki consequent tables) for N sets.
%
% The rule tables are N x N matrices where:
%   rows    -> error linguistic values   (NL, NM, NS, ZE, PS, PM, PL for N=7)
%   columns -> error_dot linguistic values
%   entry   -> crisp consequent for that rule
%
% Presets:
%   'expert'       — hand-designed rules encoding pendulum domain knowledge
%   'aggressive'   — fast response, higher gains, accepts more overshoot
%   'conservative' — slow, minimal overshoot, smooth control
%   'symmetric'    — exactly symmetric kp/kd about ZE (equal left/right)
%   'data_driven'  — placeholder, overwritten by fp_rule_designer output
%
% Usage:
%   rb = fp_rule_base('expert', 7)
%   rb = fp_rule_base('aggressive', 5)

if nargin < 1, preset  = 'expert'; end
if nargin < 2, n_sets  = 7;        end

if n_sets == 7
    rb = build_7(preset);
elseif n_sets == 5
    rb = build_5(preset);
elseif n_sets == 9
    rb = build_9(preset);
else
    warning('n_sets=%d not supported, using 7.', n_sets);
    rb = build_7(preset);
end

rb.preset = preset;
rb.n_sets = n_sets;
rb.description = describe(preset);
end

% =========================================================================
function rb = build_7(preset)
% 7x7 rule tables
% Rows: error     NL NM NS ZE PS PM PL
% Cols: error_dot NL NM NS ZE PS PM PL
n = 7;

switch preset
    case 'expert'
        % Kp rules: large kp when far from target, small near zero
        % Based on the original project rules (verified working)
        rb.kp_rule = [
            180  150  120   80   60   40   20;
            150  130  100   70   50   30   15;
            120  100   80   60   40   25   12;
             80   70   60   50   35   20   10;
             60   50   40   35   30   15    8;
             40   30   25   20   15   10    5;
             20   15   12   10    8    5    2];

        % Kd rules: high damping when moving fast, low when slow
        rb.kd_rule = [
             30   25   20   15   10    8    5;
             25   22   18   13    9    7    4;
             20   18   15   12    8    6    3;
             15   13   12   10    7    5    2;
             10    9    8    7    6    4    2;
              8    7    6    5    4    3    1;
              5    4    3    2    2    1  0.5];

        % Ki rules (alpha blending factor 0-1): small integral action
        rb.ki_rule = [
           0.05  0.05  0.04  0.03  0.02  0.01  0.005;
           0.05  0.04  0.04  0.03  0.02  0.01  0.005;
           0.04  0.04  0.03  0.03  0.02  0.01  0.004;
           0.03  0.03  0.03  0.02  0.02  0.01  0.003;
           0.02  0.02  0.02  0.02  0.01  0.008 0.002;
           0.01  0.01  0.01  0.01  0.008 0.005 0.001;
           0.005 0.005 0.004 0.003 0.002 0.001 0.0005];

    case 'aggressive'
        scale = 1.4;
        rb_base = build_7('expert');
        rb.kp_rule = rb_base.kp_rule * scale;
        rb.kd_rule = rb_base.kd_rule * 0.8;   % less damping
        rb.ki_rule = rb_base.ki_rule * 2.0;   % more integral

    case 'conservative'
        scale = 0.6;
        rb_base = build_7('expert');
        rb.kp_rule = rb_base.kp_rule * scale;
        rb.kd_rule = rb_base.kd_rule * 1.4;   % more damping
        rb.ki_rule = rb_base.ki_rule * 0.5;   % less integral

    case 'symmetric'
        % Force exact symmetry: rb(i,j) = rb(n+1-i, n+1-j)
        rb_base = build_7('expert');
        kp = rb_base.kp_rule;
        kp_sym = (kp + rot90(kp,2)) / 2;
        rb.kp_rule = kp_sym;
        kd = rb_base.kd_rule;
        kd_sym = (kd + rot90(kd,2)) / 2;
        rb.kd_rule = kd_sym;
        rb.ki_rule = rb_base.ki_rule;

    case 'data_driven'
        % Placeholder — will be overwritten by fp_rule_designer
        rb = build_7('expert');
        rb.preset = 'data_driven';
        return;

    otherwise
        warning('Unknown preset "%s", using expert.', preset);
        rb = build_7('expert');
end
end

% =========================================================================
function rb = build_5(preset)
% 5x5 rule table (downsampled from 7x7 expert)
rb7 = build_7(preset);
idx = [1, 2, 4, 6, 7];   % sample rows/cols from 7-set
rb.kp_rule = rb7.kp_rule(idx, idx);
rb.kd_rule = rb7.kd_rule(idx, idx);
rb.ki_rule = rb7.ki_rule(idx, idx);
end

% =========================================================================
function rb = build_9(preset)
% 9x9: extend expert rules with finer resolution near zero
rb7 = build_7(preset);
% Interpolate to 9x9
[X7,Y7] = meshgrid(1:7,1:7);
[X9,Y9] = meshgrid(linspace(1,7,9), linspace(1,7,9));
rb.kp_rule = interp2(X7,Y7,rb7.kp_rule,X9,Y9,'linear');
rb.kd_rule = interp2(X7,Y7,rb7.kd_rule,X9,Y9,'linear');
rb.ki_rule = interp2(X7,Y7,rb7.ki_rule,X9,Y9,'linear');
rb.kp_rule = max(0, rb.kp_rule);
rb.kd_rule = max(0, rb.kd_rule);
rb.ki_rule = max(0, rb.ki_rule);
end

% =========================================================================
function s = describe(preset)
switch preset
    case 'expert'
        s = 'Hand-designed rules encoding pendulum domain knowledge';
    case 'aggressive'
        s = 'High gains, fast response, accepts larger overshoot';
    case 'conservative'
        s = 'Low gains, heavy damping, minimal overshoot';
    case 'symmetric'
        s = 'Exactly symmetric about ZE — equal left/right response';
    case 'data_driven'
        s = 'Generated by fp_rule_designer from closed-loop observations';
    otherwise
        s = preset;
end
end
