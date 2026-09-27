function [kp, kd, ki, debug] = fp_inference(e, edot, rule_base, fcfg)
% FP_INFERENCE
% Fuzzy inference engine: maps (error, error_dot) -> PID gains.
%
% Supports:
%   Sugeno   — crisp consequents, weighted average defuzzification (fast)
%   Mamdani  — fuzzy consequents, centroid / MOM defuzzification (richer)
%   Type-2   — interval-valued antecedent MFs with Karnik-Mendel reduction
%
% Inputs:
%   e          — angle error (deg):  e = theta_ref - theta = 0 - theta
%   edot       — error rate (deg/s): edot = -theta_dot
%   rule_base  — struct with fields:
%     .kp_rule   N x N matrix of kp consequents  (N = n_sets)
%     .kd_rule   N x N matrix of kd consequents
%     .ki_rule   N x N matrix of ki consequents (or alpha blending)
%   fcfg       — fuzzy config struct (see fp_membership.m)
%     .inference    'sugeno' | 'mamdani'
%     .and_op       'min' | 'product'
%     .defuzz       'weighted_avg' | 'centroid' | 'mom' (mean of maxima)
%
% Outputs:
%   kp, kd, ki — scalar crisp PID gain outputs
%   debug      — struct with firing strengths and intermediate values

n = fcfg.n_sets;

% ---- Fuzzify both inputs ------------------------------------------------
mu_e    = fp_membership(e,    fcfg);   % N x 1 (or N x 2 for Type-2)
mu_edot = fp_membership(edot, fcfg);

% ---- Type-2 reduction (Karnik-Mendel simplified) ------------------------
is_t2 = fcfg.type2 && size(mu_e, 2) == 2;
if is_t2
    % Use average of lower and upper membership for centroid-of-sets
    mu_e    = mean(mu_e,    2);   % N x 1
    mu_edot = mean(mu_edot, 2);   % N x 1
end

% ---- Compute firing strengths -------------------------------------------
% w(i,j) = strength of rule (i,j): if E is A_i AND Edot is B_j
w = zeros(n, n);
for i = 1:n
    for j = 1:n
        switch fcfg.and_op
            case 'min'
                w(i,j) = min(mu_e(i), mu_edot(j));
            case 'product'
                w(i,j) = mu_e(i) * mu_edot(j);
            otherwise
                w(i,j) = min(mu_e(i), mu_edot(j));
        end
    end
end

w_total = sum(w(:));

% ---- Defuzzification ----------------------------------------------------
switch fcfg.inference

    case 'sugeno'
        % Weighted average of crisp consequents
        if w_total < 1e-12
            kp = rule_base.kp_rule(ceil(n/2), ceil(n/2));
            kd = rule_base.kd_rule(ceil(n/2), ceil(n/2));
            ki = rule_base.ki_rule(ceil(n/2), ceil(n/2));
        else
            kp = sum(sum(w .* rule_base.kp_rule)) / w_total;
            kd = sum(sum(w .* rule_base.kd_rule)) / w_total;
            ki = sum(sum(w .* rule_base.ki_rule)) / w_total;
        end

    case 'mamdani'
        % Output consequents are singletons (same tables used for fairness)
        % Defuzzify by chosen method
        switch fcfg.defuzz
            case 'centroid'
                % Centroid of area: same as weighted average for singletons
                if w_total < 1e-12
                    kp = rule_base.kp_rule(ceil(n/2), ceil(n/2));
                    kd = rule_base.kd_rule(ceil(n/2), ceil(n/2));
                    ki = rule_base.ki_rule(ceil(n/2), ceil(n/2));
                else
                    kp = sum(sum(w .* rule_base.kp_rule)) / w_total;
                    kd = sum(sum(w .* rule_base.kd_rule)) / w_total;
                    ki = sum(sum(w .* rule_base.ki_rule)) / w_total;
                end

            case 'mom'
                % Mean of maxima: find rules with highest firing, average their consequents
                [max_w, ~] = max(w(:));
                tol = max_w * 0.05;
                mask = w >= (max_w - tol);
                kp = mean(rule_base.kp_rule(mask));
                kd = mean(rule_base.kd_rule(mask));
                ki = mean(rule_base.ki_rule(mask));

            otherwise
                % Default: weighted average
                if w_total < 1e-12
                    kp = rule_base.kp_rule(ceil(n/2), ceil(n/2));
                    kd = rule_base.kd_rule(ceil(n/2), ceil(n/2));
                    ki = rule_base.ki_rule(ceil(n/2), ceil(n/2));
                else
                    kp = sum(sum(w .* rule_base.kp_rule)) / w_total;
                    kd = sum(sum(w .* rule_base.kd_rule)) / w_total;
                    ki = sum(sum(w .* rule_base.ki_rule)) / w_total;
                end
        end

    otherwise
        error('Unknown inference type: %s', fcfg.inference);
end

% ---- Clamp gains to valid range -----------------------------------------
kp = max(0, kp);
kd = max(0, kd);
ki = max(0, ki);

% ---- Debug output -------------------------------------------------------
debug = struct();
debug.mu_e       = mu_e;
debug.mu_edot    = mu_edot;
debug.w          = w;
debug.w_total    = w_total;
debug.is_t2      = is_t2;

end
