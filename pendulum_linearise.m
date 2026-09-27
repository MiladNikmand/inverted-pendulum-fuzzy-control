function [A, B, C, D, info] = pendulum_linearise(p, quiet)
% PENDULUM_LINEARISE
% Linearises the cart-pendulum about the upright equilibrium.
%
% Usage:
%   [A,B,C,D,info] = pendulum_linearise(p)          % verbose
%   [A,B,C,D,info] = pendulum_linearise(p, true)    % quiet (for optimiser)

if nargin < 2, quiet = false; end
%
% EQUILIBRIUM: theta=0, theta_dot=0, xc_dot=0, F=0
%
% LINEARISATION:
%   At equilibrium: sin(theta)~theta, cos(theta)~1, theta_dot^2~0
%   The mass matrix becomes constant:
%
%     M_eq = [(mc+m)   m*l]
%            [m*l      m*l^2]
%
%   Both entries of M_eq^{-1}*B are non-zero (coupling m*l != 0),
%   so the force F directly actuates BOTH xc_ddot and theta_ddot
%   in the linearised model.
%
% STATE-SPACE:  x = [xc; xc_dot; theta; theta_dot]
%   A = [0  1       0               0  ]
%       [0 -b*Minv11  mgl*Minv12    0  ]
%       [0  0       0               1  ]
%       [0 -b*Minv21  mgl*Minv22    0  ]
%
%   B = [0        ]
%       [Minv(1,1)]
%       [0        ]
%       [Minv(2,1)]
%
% NOTE: Minv(2,1) = -m*l / det(M) < 0
%   Positive F -> xc_ddot > 0  (cart moves right)
%   Positive F -> theta_ddot < 0  (pendulum tilts left — corrective when theta>0)
%   Therefore: error convention e = +theta (not -theta)
%
% CONTROLLABILITY: rank-4 (full) for all valid parameters.
%   The coupling m*l is non-zero at equilibrium, unlike some
%   rotary configurations where coupling vanishes at upright.

% ---- Mass matrix at equilibrium ----------------------------------------
M11  = p.M11_eq;    % mc + m
M12  = p.M12_eq;    % m*l
M22  = p.M22_eq;    % m*l^2
Minv = p.Minv_eq;   % 2x2 inverse

mgl  = p.m * p.g * p.l;    % gravity stiffness (N*m/rad)

% ---- State matrix A ----------------------------------------------------
A = zeros(4);
A(1,2) = 1;
A(2,2) = -Minv(1,1) * p.b;       % cart friction -> xc_dot
A(2,3) =  Minv(1,2) * mgl;       % pendulum gravity -> xc (coupling)
A(3,4) = 1;
A(4,2) = -Minv(2,1) * p.b;       % cart friction -> theta (coupling)
A(4,3) =  Minv(2,2) * mgl;       % pendulum gravity -> theta_ddot (POSITIVE: unstable)

% ---- Input matrix B ----------------------------------------------------
B = [0; Minv(1,1); 0; Minv(2,1)];
% Minv(1,1) > 0: positive F -> positive xc_ddot  (cart goes right)
% Minv(2,1) < 0: positive F -> negative theta_ddot (pendulum corrected)

C = eye(4);
D = zeros(4,1);

% ---- Controllability analysis ------------------------------------------
Co     = [B, A*B, A^2*B, A^3*B];
rank_co = rank(Co, 1e-9);

% ---- Observability analysis --------------------------------------------
Ob     = [C; C*A; C*A^2; C*A^3];
rank_obs = rank(Ob, 1e-9);

% ---- Open-loop eigenvalues --------------------------------------------
ev_ol = eig(A);

% ---- Info struct -------------------------------------------------------
info = struct();
info.M11_eq    = M11;
info.M12_eq    = M12;
info.M22_eq    = M22;
info.Minv_eq   = Minv;
info.detM      = M11*M22 - M12^2;
info.eig_ol    = ev_ol;
info.rank_co   = rank_co;
info.rank_obs  = rank_obs;
info.omega_n   = p.omega_n;
info.note = sprintf(['Cart-pendulum. Coupling M12=m*l=%.4f (non-zero). '...
    'Rank=%d/4 (full). LQR valid. '...
    'Error convention: e = +theta.'], M12, rank_co);

% ---- Augmented system (integral xc action) -----------------------------
% Augmented state: xa = [xc; xc_dot; theta; theta_dot; xi]
% xi_dot = xc - xc_ref  (integrated cart position error)
%
% Aa = [A,         zeros(4,1)]
%      [1  0  0  0     0     ]   <- xi_dot = xc (xc_ref enters as offset)
%
% Ba = [B; 0]
%
% Controllability of augmented system is rank-5 for all valid parameters.
Aa = zeros(5);
Aa(1:4,1:4) = A;
Aa(5,1)     = 1;          % xi_dot = xc
Ba          = [B; 0];

Co5    = [Ba, Aa*Ba, Aa^2*Ba, Aa^3*Ba, Aa^4*Ba];
rank_aug = rank(Co5, 1e-9);

info.Aa       = Aa;
info.Ba       = Ba;
info.rank_aug = rank_aug;

if ~quiet
    fprintf('[pendulum_linearise]\n');
    fprintf('  Configuration:  Cart-pendulum (horizontal track / ring)\n');
    fprintf('  M11=%.4f  M12=%.4f  M22=%.4f  det(M)=%.6f\n', ...
            M11, M12, M22, info.detM);
    fprintf('  M_inv(1,1)=%.4f  M_inv(2,1)=%.4f\n', Minv(1,1), Minv(2,1));
    fprintf('  Open-loop eigenvalues:  ');
    fprintf('%.4f  ', ev_ol); fprintf('\n');
    fprintf('  Controllability rank: %d / 4  ', rank_co);
    if rank_co == 4
        fprintf('[FULL — LQR valid]\n');
    else
        fprintf('[DEFICIENT]\n');
    end
    fprintf('  Augmented (5-state) rank: %d / 5\n', rank_aug);
    fprintf('  Observability rank:   %d / 4\n', rank_obs);
    fprintf('  omega_n = %.4f rad/s\n', p.omega_n);
    fprintf('  NOTE: %s\n\n', info.note);
end

end
