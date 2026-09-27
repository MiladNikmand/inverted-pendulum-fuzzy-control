function [mu, centres] = fp_membership(x, fcfg)
% FP_MEMBERSHIP
% Evaluates fuzzy membership functions for input x.
%
% Supports three MF types: triangular, Gaussian, trapezoidal.
% Supports three partition styles: uniform, concentrated (near zero), custom.
% Supports Type-2 fuzzy (interval-valued) via a symmetric FOU band.
%
% Inputs:
%   x     — scalar input value (degrees or rad/s, same units as fcfg.range)
%   fcfg  — fuzzy config struct:
%     .n_sets      number of sets (5, 7, or 9)
%     .range       [min, max] universe of discourse
%     .mf_type     'triangular' | 'gaussian' | 'trapezoidal'
%     .partition   'uniform' | 'concentrated'
%     .type2       logical — enable Type-2 interval MFs
%     .fou_width   FOU half-width fraction (e.g. 0.15 = ±15%) [Type-2 only]
%
% Outputs:
%   mu      — N x 1 vector of membership values (or N x 2 for Type-2: [lo, hi])
%   centres — N x 1 vector of MF centres

n   = fcfg.n_sets;
lo  = fcfg.range(1);
hi  = fcfg.range(2);

% ---- Compute MF centres -------------------------------------------------
switch fcfg.partition
    case 'uniform'
        centres = linspace(lo, hi, n)';

    case 'concentrated'
        % More MFs near zero, fewer at extremes.
        % Use a sinh-spaced partition: denser near centre.
        t = linspace(-1, 1, n)';
        span = max(abs(lo), abs(hi));
        centres = span * sinh(t * 1.5) / sinh(1.5);
        % Clip to range
        centres = max(lo, min(hi, centres));

    otherwise
        centres = linspace(lo, hi, n)';
end

% ---- Width of each MF (half-base for triangular / sigma for Gaussian) ---
% For uniform: width = spacing between centres
spacing = (hi - lo) / (n - 1);

% ---- Evaluate MFs -------------------------------------------------------
mu = zeros(n, 1);

switch fcfg.mf_type

    case 'triangular'
        for i = 1:n
            c = centres(i);
            if i == 1
                w_l = spacing; w_r = spacing;
            elseif i == n
                w_l = spacing; w_r = spacing;
            else
                w_l = c - centres(i-1);
                w_r = centres(i+1) - c;
            end
            if x <= c
                mu(i) = max(0, 1 - (c - x) / w_l);
            else
                mu(i) = max(0, 1 - (x - c) / w_r);
            end
        end

    case 'gaussian'
        sigma = spacing * 0.6;   % sigma chosen so adjacent MFs cross at ~0.5
        for i = 1:n
            mu(i) = exp(-0.5 * ((x - centres(i)) / sigma)^2);
        end

    case 'trapezoidal'
        % Flat top width = 0.4 * spacing, slopes on either side
        flat  = 0.2 * spacing;
        slope = 0.4 * spacing;
        for i = 1:n
            c  = centres(i);
            lo_flat = c - flat;
            hi_flat = c + flat;
            lo_slp  = lo_flat - slope;
            hi_slp  = hi_flat + slope;
            if x < lo_slp || x > hi_slp
                mu(i) = 0;
            elseif x <= lo_flat
                mu(i) = (x - lo_slp) / slope;
            elseif x <= hi_flat
                mu(i) = 1;
            else
                mu(i) = (hi_slp - x) / slope;
            end
            mu(i) = max(0, min(1, mu(i)));
        end

    otherwise
        error('Unknown mf_type: %s', fcfg.mf_type);
end

% ---- Type-2: convert to interval [mu_lo, mu_hi] -------------------------
if fcfg.type2
    fw  = fcfg.fou_width;   % symmetric FOU half-width
    mu_lo = max(0, mu - fw * mu);
    mu_hi = min(1, mu + fw * mu);
    mu = [mu_lo, mu_hi];   % N x 2
end

end
