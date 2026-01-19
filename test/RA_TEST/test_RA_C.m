%% test_RA_C.m - Coriolis/Centrifugal 항 검증
% MATLAB velocityProduct 함수와 비교

clear; clc; close all;
addpath(genpath('D:\hanul2026_scie\mppi_RA'));

%% 파라미터 초기화
urdf_path = 'D:\hanul2026_scie\mppi_RA\model\urdf\ASM_KARI_ARM_URDF.urdf';
params = params_init(urdf_path);
robot = params.arm.robot;

%% 테스트 조건
% 랜덤 관절 각도/속도
rng(42);
theta = 0.5 * randn(7, 1);
theta_dot = 0.3 * randn(7, 1);

% Base 상태 (정지)
p_base = [0; 0; 0];
q_base = [1; 0; 0; 0];
v_base = [0; 0; 0];
omega_base = [0; 0; 0];

%% FK 계산
FK = RA_FK(params, p_base, q_base, theta);

%% RA_C 계산 (매니퓰레이터 부분만)
c_full = RA_C(FK, params, v_base, omega_base, theta_dot);
c_theta = c_full(7:13);  % 관절 토크 부분

%% MATLAB velocityProduct 비교
c_matlab = velocityProduct(robot, theta, theta_dot);

%% 결과 비교
fprintf('=== RA_C 검증 결과 ===\n');
fprintf('c_theta (RA_C) vs c (MATLAB velocityProduct)\n\n');

fprintf('Joint |   RA_C   |  MATLAB  |   Diff\n');
fprintf('------+----------+----------+----------\n');
for i = 1:7
    fprintf('  %d   | %8.4f | %8.4f | %8.2e\n', ...
        i, c_theta(i), c_matlab(i), abs(c_theta(i) - c_matlab(i)));
end

err_c = norm(c_theta - c_matlab);
fprintf('\nFrobenius 오차: %.2e\n', err_c);

%% 다양한 속도 조건 테스트
fprintf('\n=== 다양한 속도 조건 테스트 ===\n');

test_cases = {
    'theta_dot = [1,0,0,0,0,0,0]', [1;0;0;0;0;0;0];
    'theta_dot = [0,1,0,0,0,0,0]', [0;1;0;0;0;0;0];
    'theta_dot = [0,0,0,0,0,0,1]', [0;0;0;0;0;0;1];
    'theta_dot = ones(7,1)', ones(7,1);
};

for tc = 1:size(test_cases, 1)
    td = test_cases{tc, 2};
    
    c_ours = RA_C(FK, params, v_base, omega_base, td);
    c_ref = velocityProduct(robot, theta, td);
    
    err = norm(c_ours(7:13) - c_ref);
    fprintf('%s: 오차 = %.2e\n', test_cases{tc,1}, err);
end

%% 에너지 일관성 테스트
fprintf('\n=== 에너지 일관성 테스트 ===\n');

% C 행렬은 skew-symmetric 성질: theta_dot' * C * theta_dot = 0
% (실제로는 c = C*theta_dot 벡터이므로 직접 확인 불가)
% 대신 수치적 안정성 확인

% 큰 속도에서 테스트
theta_dot_large = 2 * randn(7, 1);
c_large = RA_C(FK, params, v_base, omega_base, theta_dot_large);
c_large_matlab = velocityProduct(robot, theta, theta_dot_large);

fprintf('큰 속도 테스트: 오차 = %.2e\n', norm(c_large(7:13) - c_large_matlab));

%% 최종 판정
fprintf('\n=== 최종 판정 ===\n');
if err_c < 1e-6
    fprintf('>> 검증 통과!\n');
else
    fprintf('>> 검증 실패 - 확인 필요\n');
end