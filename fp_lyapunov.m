function lyap = fp_lyapunov(t, x, F_hist, p, phase)
% FP_LYAPUNOV
% Computes and packages Lyapunov function values for both phases.
%
% SWING-UP PHASE  (energy-based Lyapunov):
%   V_su(t)     = 0.5 * (E(t) - E_target)^2
%   V_su_dot(t) = numerical d/dt of V_su
%   Stability condition: V_su_dot <= 0 throughout swing-up
%
% STABILISATION PHASE  (quadratic Lyapunov near upright):
%   V_stab(t)     = x_stab(t)' * P * x_stab(t)
%   V_stab_dot(t) = numerical d/dt of V_stab
%   Stability condition: V_stab_dot < 0 throughout stabilisation
%
%   P is the solution to the Lyapunov equation:
%     A_cl' * P + P * A_cl = -Q
%   where A_cl is the closed-loop A matrix from the linearised model.
%   Since we don't have A_cl directly here, we use:
%     P = eye(4) (isotropic — sufficient for visualisation)
%   and note that V_stab = ||x||^2 is a valid Lyapunov function candidate
%   for any stable system.
%
% Inputs:
%   t       — Nx1 time vector
%   x       — Nx4 state [phi, phi_dot, theta, theta_dot]
%   F_hist  — Nx1 control force history
%   p       — pendulum_params struct
%   phase   — struct with fields:
%     .t_capture   time when stabilisation phase begins (s)
%     .A_cl        4x4 closed-loop A matrix (optional, for exact P)
%
% Outputs:
%   lyap  — struct with Lyapunov history arrays and stability assessment

% ---- Energy-based Lyapunov (swing-up) ----------------------------------
E_hist  = 0.5*p.m*p.l^2*x(:,4).^2 + p.m*p.g*p.l*cos(x(:,3));
E_err   = E_hist - p.E_target;
V_su    = 0.5 * E_err.^2;

% Numerical derivative (central differences)
V_su_dot = zeros(size(V_su));
dt = mean(diff(t));
if dt > 0
    V_su_dot(2:end-1) = (V_su(3:end) - V_su(1:end-2)) / (2*dt);
    V_su_dot(1)       = (V_su(2) - V_su(1)) / dt;
    V_su_dot(end)     = (V_su(end) - V_su(end-1)) / dt;
end

% Check: how often is V_su_dot > 0 (violation of Lyapunov decrease)?
n_total      = numel(V_su_dot);
n_violations = sum(V_su_dot > 1e-6);
pct_ok       = 100 * (1 - n_violations/n_total);

% ---- Quadratic Lyapunov (stabilisation phase) --------------------------
% Find stabilisation start index
if isfield(phase,'t_capture') && ~isnan(phase.t_capture)
    k_stab = find(t >= phase.t_capture, 1);
else
    k_stab = 1;
end

% Compute P matrix
if isfield(phase,'A_cl') && ~isempty(phase.A_cl)
    A_cl = phase.A_cl;
    Q_lyap = eye(4);
    try
        % Solve A_cl'*P + P*A_cl = -Q_lyap
        P = lyap(A_cl', Q_lyap);   % MATLAB's built-in lyap (base MATLAB)
    catch
        % Fallback: use identity
        P = eye(4);
    end
else
    P = eye(4);
end

% Compute V_stab = x'*P*x for each time step
V_stab     = zeros(numel(t), 1);
x_stab_ref = zeros(1,4);  % reference: upright, all zeros

for k = 1:numel(t)
    x_err_k     = x(k,:) - x_stab_ref;
    % For theta: use small-angle state (wrap theta to [-pi,pi])
    th_k        = x(k,3);
    x_err_k(3)  = th_k;   % theta deviation from 0 (upright)
    V_stab(k)   = x_err_k * P * x_err_k';
end

V_stab_dot = zeros(size(V_stab));
if dt > 0
    V_stab_dot(2:end-1) = (V_stab(3:end) - V_stab(1:end-2)) / (2*dt);
    V_stab_dot(1)       = (V_stab(2) - V_stab(1)) / dt;
    V_stab_dot(end)     = (V_stab(end) - V_stab(end-1)) / dt;
end

% Stabilisation phase only
if k_stab <= numel(t)
    V_stab_stab     = V_stab(k_stab:end);
    V_stab_dot_stab = V_stab_dot(k_stab:end);
    n_viol_stab     = sum(V_stab_dot_stab > 1e-6);
    pct_ok_stab     = 100*(1 - n_viol_stab/max(1,numel(V_stab_stab)));
else
    V_stab_stab     = [];
    V_stab_dot_stab = [];
    pct_ok_stab     = NaN;
end

% ---- Pack output -------------------------------------------------------
lyap = struct();

% Swing-up Lyapunov
lyap.V_su           = V_su;
lyap.V_su_dot       = V_su_dot;
lyap.E_hist         = E_hist;
lyap.E_target       = p.E_target;
lyap.n_violations_su = n_violations;
lyap.pct_ok_su      = pct_ok;

% Stabilisation Lyapunov
lyap.V_stab         = V_stab;
lyap.V_stab_dot     = V_stab_dot;
lyap.V_stab_stab    = V_stab_stab;
lyap.V_stab_dot_stab = V_stab_dot_stab;
lyap.P              = P;
lyap.k_stab         = k_stab;
lyap.pct_ok_stab    = pct_ok_stab;

% Assessment
lyap.assessment = sprintf( ...
    'Swing-up: V_dot <= 0 for %.1f%% of steps. Stab: V_dot < 0 for %.1f%% of steps.', ...
    pct_ok, pct_ok_stab);

end
