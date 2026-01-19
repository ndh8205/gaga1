%% RA_MM_dual 검증 스크립트
clear; clc;

%% 1. 파라미터 초기화
urdf_path = 'kari_arm.urdf';  % 경로 수정 필요
params = params_init(urdf_path);

%% 2. 초기 상태
p_base = [0; 0; 0];
q_base = [1; 0; 0; 0];
theta_L = zeros(7, 1);
theta_R = zeros(7, 1);

%% 3. FK 계산
FK_dual = RA_FK_dual(params, p_base, q_base, theta_L, theta_R);

%% 4. H 행렬 계산
H_dual = RA_MM_dual(FK_dual, params);

%% ========== 검증 1: 대칭성 ==========
sym_err = norm(H_dual - H_dual', 'fro');
fprintf('대칭성 오차: %.2e (0이어야 함)\n', sym_err);

%% ========== 검증 2: 양정치성 ==========
eig_min = min(eig(H_dual));
fprintf('최소 고유값: %.4f (>0이어야 함)\n', eig_min);

%% ========== 검증 3: 전체 질량 ==========
m_arm = 0;
for i = 1:params.arm.n_bodies
    m_arm = m_arm + params.arm.links(i).m;
end
M_expected = params.sat.m + 2 * m_arm;
M_actual = H_dual(1,1);
fprintf('전체 질량 - 예상: %.2f kg, 실제: %.2f kg, 오차: %.2e\n', ...
    M_expected, M_actual, abs(M_expected - M_actual));

%% ========== 검증 4: 싱글암 vs 듀얼암 비교 ==========
% 왼팔만 있는 것처럼 싱글암 H 계산
params_L = params;
params_L.sat.r_mount = params.sat.mount(1).pos;
params_L.sat.R_mount = params.sat.mount(1).R;
FK_L = RA_FK(params_L, p_base, q_base, theta_L);
H_single_L = RA_MM(FK_L, params_L);

% 듀얼암 H에서 왼팔 부분 추출
H_dual_L = H_dual([1:6, 7:13], [1:6, 7:13]);  % base + 왼팔

fprintf('\n싱글암(L) vs 듀얼암(L 부분) H_m 비교:\n');
H_m_single = H_single_L(7:13, 7:13);
H_m_dual = H_dual(7:13, 7:13);
fprintf('H_m 오차: %.2e\n', norm(H_m_single - H_m_dual, 'fro'));

%% ========== 검증 5: H_b 비교 ==========
fprintf('\n=== H_b (6x6) 비교 ===\n');
H_b_dual = H_dual(1:6, 1:6);
H_b_single_L = H_single_L(1:6, 1:6);

fprintf('H_b_dual(1,1) = %.2f\n', H_b_dual(1,1));
fprintf('H_b_single_L(1,1) = %.2f\n', H_b_single_L(1,1));
fprintf('차이 = %.2f (오른팔 질량만큼 차이나야 함)\n', H_b_dual(1,1) - H_b_single_L(1,1));

%% ========== 검증 6: H_bm 비교 ==========
fprintf('\n=== H_bm 비교 ===\n');
H_bm_L_dual = H_dual(1:6, 7:13);
H_bm_L_single = H_single_L(1:6, 7:13);
fprintf('H_bm_L 오차: %.2e\n', norm(H_bm_L_dual - H_bm_L_single, 'fro'));

%% ========== 검증 7: Coriolis 비교 ==========
fprintf('\n=== Coriolis 검증 ===\n');
v_base = [0; 0; 0];
omega_base = [0.1; 0; 0];  % 약간의 각속도
theta_dot_L = [0.1; 0; 0; 0; 0; 0; 0];
theta_dot_R = zeros(7, 1);

c_dual = RA_C_dual(FK_dual, params, v_base, omega_base, theta_dot_L, theta_dot_R);
c_single_L = RA_C(FK_L, params_L, v_base, omega_base, theta_dot_L);

fprintf('c_dual 크기: %d\n', length(c_dual));
fprintf('c_single_L 크기: %d\n', length(c_single_L));
fprintf('c_dual(7:13) vs c_single_L(7:13) 오차: %.2e\n', ...
    norm(c_dual(7:13) - c_single_L(7:13)));

%% ========== 검증 8: Forward Dynamics 테스트 ==========
fprintf('\n=== Forward Dynamics 테스트 ===\n');
zeta_dot = [v_base; omega_base; theta_dot_L; theta_dot_R];
tau = zeros(14, 1);  % 토크 없음
Q = [zeros(6,1); tau];

% 가속도 계산
zeta_ddot = H_dual \ (Q - c_dual);

fprintf('Base 선가속도: [%.4f, %.4f, %.4f]\n', zeta_ddot(1:3));
fprintf('Base 각가속도: [%.4f, %.4f, %.4f]\n', zeta_ddot(4:6));
fprintf('NaN/Inf 체크: %d\n', any(isnan(zeta_ddot)) || any(isinf(zeta_ddot)));

%% ========== H 행렬 시각화 ==========
figure;
imagesc(log10(abs(H_dual) + 1e-10));
colorbar;
title('RA\_MM\_dual H 행렬 (log scale)');
xlabel('열'); ylabel('행');