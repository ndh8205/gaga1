function [R, q] = quest_attitude_NR(b_obs, r_cat, weights)
% QUEST with Newton-Raphson for λ_max
% Solves for the optimal quaternion directly from λ_max,
% replacing the SVD method.
%
% Convention:
%   - Quaternion: Hamiltonian (scalar-first) q = [qw; qx; qy; qz] 
%   - Solves for R and q that rotate from r_cat (Inertial) to b_obs (Body)
%
% Inputs:
%   b_obs   - 3xN (body frame vectors)
%   r_cat   - 3xN (inertial frame vectors)
%   weights - Nx1 (optional)
%
% Outputs:
%   R - 3x3 rotation matrix (I to B)
%   q - 4x1 quaternion (I to B) [qw; qx; qy; qz]

N = size(b_obs, 2);
if nargin < 3
    weights = ones(N, 1);
end
% 정규화된 가중치 사용
weights = weights / sum(weights);

%% 1. Attitude profile matrix B
B = zeros(3, 3);
for i = 1:N
    B = B + weights(i) * (b_obs(:,i) * r_cat(:,i)');
end

%% 2. Parameters for characteristic polynomial
S = B + B';
s = trace(B);

% Z 벡터 (K 행렬의 off-diagonal block)
Z = [B(2,3) - B(3,2); 
     B(3,1) - B(1,3); 
     B(1,2) - B(2,1)];

% Characteristic polynomial coefficients
adjS = S(2,2)*S(3,3) - S(2,3)*S(3,2) + ...
       S(1,1)*S(3,3) - S(1,3)*S(3,1) + ...
       S(1,1)*S(2,2) - S(1,2)*S(2,1); % trace(adj(S))

a = s^2 - adjS;
b = s^2 + Z'*Z;
c = det(S) + Z'*S*Z;
d = Z'*S*S*Z;

%% 3. Newton-Raphson for λ_max
% 최대 고유값은 가중치의 합(1.0)을 넘을 수 없음
lambda = 1.0; 
for iter = 1:5 % 5회 반복이면 충분한 수렴성을 보임
    % f(λ) = det(K - λI)
    psi = (lambda^2 - a)*(lambda^2 - b) - c*lambda + (c*s - d);
    % f'(λ)
    psi_prime = 2*lambda*(lambda^2 - a) + 2*lambda*(lambda^2 - b) - c;
    
    if abs(psi_prime) < 1e-10
        break; % 분모가 0이 되는 것 방지
    end
    
    lambda = lambda - psi / psi_prime;
end
lambda_max = lambda;

%% 4. Optimal Quaternion Extraction (SVD 대체)
% K-matrix의 고유벡터 (q)를 lambda_max를 이용해 직접 계산합니다.
% (K - λI)q = 0
% K = [s, Z'; Z, S-sI] (Scalar-first K-matrix)
%
% [(s-λ),    Z'   ] [qw] = [0]
% [  Z,   (S-sI-λI)] [qv] = [0]
%
% (S - (s+λ)I)qv = -Z*qw
% C = (s+λ)I - S
% C*qv = Z*qw
%
% 여기서 qv = adj(C)*Z, qw = det(C) 를 (정규화 전) 쿼터니언으로 사용합니다.
% qv = inv(C)*det(C)*Z = inv(C)*qw*Z
% 따라서: qv_unnorm = C \ (Z * qw_unnorm)

C = (lambda_max + s)*eye(3) - S;

% 수치적 안정성을 위해 unnormalized qw = det(C) 사용
qw_unnorm = det(C);

% 3x3 선형 시스템 풀이: C * qv = -Z * qw
% [FIXED] -Z*qw_unnorm 를 C로 나눕니다.
qv_unnorm = C \ (-Z * qw_unnorm);

% 정규화되지 않은 쿼터니언 (스칼라-우선)
q_unnorm = [qw_unnorm; qv_unnorm];

% 정규화
q = q_unnorm / norm(q_unnorm);

% 쿼터니언의 스칼라 부분을 양수로 통일 (관례)
if q(1) < 0
    q = -q;
end

%% 5. Calculate DCM from Quaternion
% 제공된 'GetDCM_QUAT.m' 함수를 사용하여 DCM 계산
R = GetDCM_QUAT(q);

end