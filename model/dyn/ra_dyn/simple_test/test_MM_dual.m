%% test_MM_dual.m
% 양팔 Mass Matrix 검증 스크립트
%
% 검증 항목:
%   1. H 대칭성: H = H'
%   2. H 양정치성: 모든 고유값 > 0
%   3. 전체 질량: H(1,1) = m_sat + m_L + m_R
%   4. 블록 구조: H_m_LR = 0 (양팔 직접 coupling 없음)
%   5. 싱글암 비교 (옵션)

clear; clc; close all;

%% 파라미터 초기화
urdf_path = '/home/ndh/space_ros_ws/src/orbit_sim/urdf/kari_arm.urdf';

if exist(urdf_path, 'file')
    params = params_init(urdf_path);
    fprintf('URDF 로드 완료\n');
else
    warning('URDF not found. Using dummy params.');
    params = create_dummy_params();
end

%% 초기 상태
p_base = [0; 0; 0];
q_base = [1; 0; 0; 0];
theta_L = zeros(7, 1);
theta_R = zeros(7, 1);

%% FK 계산
FK_dual = RA_FK_dual(params, p_base, q_base, theta_L, theta_R);

%% Mass Matrix 계산
fprintf('\n=== Mass Matrix 계산 ===\n');
try
    H = RA_MM_dual(FK_dual, params);
    fprintf('[PASS] RA_MM_dual 호출 성공\n');
    fprintf('H 크기: %d x %d\n', size(H));
catch ME
    fprintf('[FAIL] RA_MM_dual 오류: %s\n', ME.message);
    return;
end

%% Test 1: 대칭성 검증
fprintf('\n=== Test 1: 대칭성 검증 ===\n');
sym_err = norm(H - H', 'fro');
fprintf('||H - H''||_F = %.2e\n', sym_err);

if sym_err < 1e-10
    fprintf('[PASS] H 대칭\n');
else
    fprintf('[FAIL] H 비대칭!\n');
end

%% Test 2: 양정치성 검증
fprintf('\n=== Test 2: 양정치성 검증 ===\n');
eig_H = eig(H);
min_eig = min(eig_H);
max_eig = max(eig_H);
cond_H = max_eig / min_eig;

fprintf('고유값 범위: [%.4f, %.4f]\n', min_eig, max_eig);
fprintf('조건수: %.2e\n', cond_H);

if min_eig > 0
    fprintf('[PASS] H 양정치 (모든 고유값 > 0)\n');
else
    fprintf('[FAIL] H 양정치 아님! min_eig = %.6f\n', min_eig);
end

%% Test 3: 전체 질량 검증
fprintf('\n=== Test 3: 전체 질량 검증 ===\n');

% 예상 질량
m_sat = params.sat.m;
m_arm = 0;
for i = 1:params.arm.n_bodies
    m_arm = m_arm + params.arm.links(i).m;
end
M_expected = m_sat + 2 * m_arm;  % 양팔

% H에서 추출한 질량
M_from_H = H(1, 1);

mass_err = abs(M_expected - M_from_H);
fprintf('예상 전체 질량: %.4f kg\n', M_expected);
fprintf('H(1,1) 질량: %.4f kg\n', M_from_H);
fprintf('오차: %.6f kg\n', mass_err);

if mass_err < 1e-6
    fprintf('[PASS] 전체 질량 일치\n');
else
    fprintf('[FAIL] 전체 질량 불일치!\n');
end

%% Test 4: 블록 구조 검증 (양팔 직접 coupling = 0)
fprintf('\n=== Test 4: 양팔 직접 coupling 검증 ===\n');

H_LR = H(7:13, 14:20);  % 왼팔-오른팔 coupling
H_RL = H(14:20, 7:13);  % 오른팔-왼팔 coupling

coupling_err = norm(H_LR, 'fro') + norm(H_RL, 'fro');
fprintf('||H_LR||_F + ||H_RL||_F = %.2e\n', coupling_err);

if coupling_err < 1e-10
    fprintf('[PASS] 양팔 직접 coupling 없음 (0 블록)\n');
else
    fprintf('[FAIL] 양팔 직접 coupling 존재!\n');
end

%% Test 5: 관성행렬 역행렬 안정성
fprintf('\n=== Test 5: 역행렬 안정성 ===\n');

try
    H_inv = inv(H);
    inv_err = norm(H * H_inv - eye(20), 'fro');
    fprintf('||H * H_inv - I||_F = %.2e\n', inv_err);
    
    if inv_err < 1e-10
        fprintf('[PASS] 역행렬 안정\n');
    else
        fprintf('[WARN] 역행렬 정밀도 저하\n');
    end
catch
    fprintf('[FAIL] 역행렬 계산 실패 (특이행렬)\n');
end

%% Test 6: 싱글암 비교 (옵션)
fprintf('\n=== Test 6: 싱글암 비교 ===\n');

try
    % 왼팔만으로 싱글암 Mass Matrix
    params_L = params;
    params_L.sat.r_mount = params.sat.mount(1).pos;
    params_L.sat.R_mount = params.sat.mount(1).R;
    FK_L = RA_FK(params_L, p_base, q_base, theta_L);
    H_single_L = RA_MM(FK_L, params_L);
    
    % 싱글암 H_m (7x7) 추출
    H_m_single_L = H_single_L(7:13, 7:13);
    
    % 듀얼에서 왼팔 H_m (7x7) 추출
    H_m_dual_L = H(7:13, 7:13);
    
    % 비교
    H_m_err = norm(H_m_single_L - H_m_dual_L, 'fro');
    fprintf('왼팔 H_m 오차: %.2e\n', H_m_err);
    
    if H_m_err < 1e-10
        fprintf('[PASS] 싱글암-듀얼 H_m 일치\n');
    else
        fprintf('[WARN] H_m 차이 존재 (상대 오차 확인 필요)\n');
    end
    
catch ME
    fprintf('[SKIP] 싱글암 비교 생략: %s\n', ME.message);
end

%% Test 7: 다양한 자세에서 검증
fprintf('\n=== Test 7: 다양한 자세 검증 ===\n');

test_configs = {
    zeros(7,1), zeros(7,1), '영자세';
    [pi/4;0;0;0;0;0;0], zeros(7,1), '왼팔 J1=45도';
    zeros(7,1), [0;pi/4;0;0;0;0;0], '오른팔 J2=45도';
    [0;pi/4;0;pi/4;0;0;0], [0;-pi/4;0;-pi/4;0;0;0], '양팔 대칭 자세';
    rand(7,1)*0.5, rand(7,1)*0.5, '랜덤 자세';
};

all_pass = true;
for tc = 1:size(test_configs, 1)
    theta_L_test = test_configs{tc, 1};
    theta_R_test = test_configs{tc, 2};
    config_name = test_configs{tc, 3};
    
    FK_test = RA_FK_dual(params, p_base, q_base, theta_L_test, theta_R_test);
    H_test = RA_MM_dual(FK_test, params);
    
    % 검증
    is_sym = norm(H_test - H_test', 'fro') < 1e-10;
    is_pd = min(eig(H_test)) > 0;
    mass_ok = abs(H_test(1,1) - M_expected) < 1e-6;
    
    if is_sym && is_pd && mass_ok
        fprintf('  [PASS] %s\n', config_name);
    else
        fprintf('  [FAIL] %s (sym=%d, pd=%d, mass=%d)\n', config_name, is_sym, is_pd, mass_ok);
        all_pass = false;
    end
end

%% 결과 요약
fprintf('\n========================================\n');
fprintf('Mass Matrix 검증 결과\n');
fprintf('========================================\n');

fprintf('\nH 구조:\n');
fprintf('  [H_b (6x6)]  [H_bm_L (6x7)]  [H_bm_R (6x7)]\n');
fprintf('  [H_bm_L''    ]  [H_m_L (7x7)]   [    0      ]\n');
fprintf('  [H_bm_R''    ]  [    0      ]   [H_m_R (7x7)]\n');

fprintf('\n주요 값:\n');
fprintf('  전체 질량 M = %.2f kg\n', M_from_H);
fprintf('  조건수 = %.2e\n', cond_H);
fprintf('  최소 고유값 = %.4f\n', min_eig);

%% 더미 파라미터 (test_FK_dual.m과 동일)
function params = create_dummy_params()
    params.sat.m = 500;
    params.sat.I = diag([260, 280, 170]);
    params.sat.mount(1).pos = [1.9; 0.9; 0];
    params.sat.mount(1).R = [1 0 0; 0 0 1; 0 -1 0];
    params.sat.mount(2).pos = [1.9; -0.9; 0];
    params.sat.mount(2).R = [1 0 0; 0 0 -1; 0 1 0];
    params.sat.r_mount = params.sat.mount(1).pos;
    params.sat.R_mount = params.sat.mount(1).R;
    
    params.arm.n_joints = 7;
    params.arm.n_bodies = 9;
    
    joint_types = {'fixed', 'revolute', 'revolute', 'revolute', ...
                   'revolute', 'revolute', 'revolute', 'revolute', 'fixed'};
    masses = [3.3436, 2.1207, 6.6621, 4.0005, 6.66208, 2.6161, 4.9702, 3.3715, 0.001];
    
    for i = 1:9
        params.arm.links(i).name = sprintf('link%d', i);
        params.arm.links(i).m = masses(i);
        params.arm.links(i).com = [0; 0; 0.05];
        params.arm.links(i).I = eye(3) * 0.01;
        params.arm.links(i).joint_type = joint_types{i};
        params.arm.links(i).joint_axis = [0; 0; 1];
        params.arm.links(i).T_fixed = eye(4);
        params.arm.links(i).T_fixed(1:3, 4) = [0; 0; 0.1];
    end
end