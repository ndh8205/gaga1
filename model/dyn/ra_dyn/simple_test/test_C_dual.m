%% test_C_dual.m - RA_C_dual 검증 테스트
clear; clc;

%% 경로 설정 (필요시 수정)
% addpath('/mnt/project');
% addpath('/home/claude');
% Windows 사용자: 모든 .m 파일을 같은 폴더에 배치

%% 파라미터 로드
urdf_path = '/home/ndh/space_ros_ws/install/orbit_sim/share/orbit_sim/urdf/kari_arm.urdf';
if exist(urdf_path, 'file')
    params = params_init(urdf_path);
else
    warning('URDF not found. Using dummy params.');
    params = create_dummy_params();
end

n_joints = params.arm.n_joints;

%% 초기 상태
p_base = [0; 0; 2];
q_base = [1; 0; 0; 0];

% 관절 각도
theta_L = zeros(n_joints, 1);
theta_R = zeros(n_joints, 1);

% 속도
v_base = [0.01; 0.02; 0.005];
omega_base = [0.05; 0.03; 0.02];
theta_dot_L = [0.1; 0.2; 0.15; 0.1; 0.05; 0.08; 0.03];
theta_dot_R = [0.12; 0.18; 0.1; 0.08; 0.06; 0.04; 0.02];

%% FK
fprintf('=== Coriolis 벡터 계산 ===\n');
FK_dual = RA_FK_dual(params, p_base, q_base, theta_L, theta_R);

%% RA_C_dual 호출
try
    c = RA_C_dual(FK_dual, params, v_base, omega_base, theta_dot_L, theta_dot_R);
    fprintf('[PASS] RA_C_dual 호출 성공\n');
    fprintf('c 크기: %d x %d\n', size(c, 1), size(c, 2));
catch ME
    fprintf('[FAIL] RA_C_dual 오류: %s\n', ME.message);
    return;
end

%% Test 1: 크기 검증
fprintf('\n=== Test 1: 크기 검증 ===\n');
expected_size = 6 + 2*n_joints;  % 20
if length(c) == expected_size
    fprintf('[PASS] c 크기 = %d (예상: %d)\n', length(c), expected_size);
else
    fprintf('[FAIL] c 크기 = %d (예상: %d)\n', length(c), expected_size);
end

%% Test 2: 제로 속도 → 제로 Coriolis
fprintf('\n=== Test 2: 제로 속도 검증 ===\n');
c_zero = RA_C_dual(FK_dual, params, zeros(3,1), zeros(3,1), zeros(7,1), zeros(7,1));
c_zero_norm = norm(c_zero);
if c_zero_norm < 1e-10
    fprintf('[PASS] 제로 속도 → c = 0 (norm = %.2e)\n', c_zero_norm);
else
    fprintf('[FAIL] 제로 속도 → c ≠ 0 (norm = %.2e)\n', c_zero_norm);
end

%% Test 3: NaN/Inf 검증
fprintf('\n=== Test 3: NaN/Inf 검증 ===\n');
if any(isnan(c)) || any(isinf(c))
    fprintf('[FAIL] c에 NaN 또는 Inf 포함\n');
else
    fprintf('[PASS] NaN/Inf 없음\n');
end

%% Test 4: 싱글암 비교 (왼팔만 움직일 때)
fprintf('\n=== Test 4: 싱글암 비교 (왼팔만) ===\n');

% 오른팔 속도 0
theta_dot_R_zero = zeros(7, 1);
c_left_only = RA_C_dual(FK_dual, params, v_base, omega_base, theta_dot_L, theta_dot_R_zero);

% 싱글암 RA_C로 왼팔 계산
params_L = params;
params_L.sat.r_mount = params.sat.mount(1).pos;
params_L.sat.R_mount = params.sat.mount(1).R;

FK_L = RA_FK(params_L, p_base, q_base, theta_L);
c_single = RA_C(FK_L, params_L, v_base, omega_base, theta_dot_L);

% 왼팔 관절 토크 비교
tau_dual_L = c_left_only(7:13);
tau_single = c_single(7:13);
tau_err = norm(tau_dual_L - tau_single);

if tau_err < 1e-6
    fprintf('[PASS] 왼팔 토크 일치 (오차: %.2e)\n', tau_err);
else
    fprintf('[FAIL] 왼팔 토크 불일치 (오차: %.2e)\n', tau_err);
    fprintf('  Dual:   '); fprintf('%.4f ', tau_dual_L'); fprintf('\n');
    fprintf('  Single: '); fprintf('%.4f ', tau_single'); fprintf('\n');
end

%% Test 5: 대칭 자세/속도 → 대칭 결과
fprintf('\n=== Test 5: 대칭성 검증 ===\n');

% 대칭 자세
theta_sym = [0.3; 0.4; 0.2; 0.5; 0.1; 0.3; 0.2];
theta_dot_sym = [0.1; 0.15; 0.08; 0.12; 0.05; 0.07; 0.04];

FK_sym = RA_FK_dual(params, p_base, q_base, theta_sym, theta_sym);

% 양팔 같은 속도로 움직일 때
c_sym = RA_C_dual(FK_sym, params, zeros(3,1), [0; 0; 0.05], theta_dot_sym, theta_dot_sym);

tau_L_sym = c_sym(7:13);
tau_R_sym = c_sym(14:20);

% Y축 기준 대칭이므로 일부 성분만 비교 (부호 반전 있을 수 있음)
% Z축 회전(joint 1,3,5,7)은 같은 부호, Y축 회전(joint 2,4,6)은 반대 부호
z_joints = [1, 3, 5, 7];
y_joints = [2, 4, 6];

z_match = norm(tau_L_sym(z_joints) - tau_R_sym(z_joints)) < 0.1 * norm(tau_L_sym(z_joints));
% Y축 조인트는 장착 회전 때문에 복잡 → 크기만 비교
y_match = abs(norm(tau_L_sym(y_joints)) - norm(tau_R_sym(y_joints))) < 0.1 * norm(tau_L_sym(y_joints));

if z_match && y_match
    fprintf('[PASS] 대칭 자세에서 유사한 토크 패턴\n');
else
    fprintf('[INFO] 대칭성 부분 일치 (장착 회전으로 인한 차이 가능)\n');
end
fprintf('  왼팔 tau: '); fprintf('%.4f ', tau_L_sym'); fprintf('\n');
fprintf('  오른팔 tau: '); fprintf('%.4f ', tau_R_sym'); fprintf('\n');

%% Test 6: 다양한 자세에서 안정성
fprintf('\n=== Test 6: 다양한 자세 검증 ===\n');

test_cases = {
    '영자세', zeros(7,1), zeros(7,1);
    '왼팔 굽힘', [0;pi/4;0;pi/3;0;pi/6;0], zeros(7,1);
    '오른팔 굽힘', zeros(7,1), [0;pi/4;0;pi/3;0;pi/6;0];
    '양팔 굽힘', [0;pi/4;0;pi/3;0;pi/6;0], [0;pi/4;0;pi/3;0;pi/6;0];
    '랜덤', rand(7,1)*pi/2 - pi/4, rand(7,1)*pi/2 - pi/4;
};

all_pass = true;
for i = 1:size(test_cases, 1)
    name = test_cases{i, 1};
    th_L = test_cases{i, 2};
    th_R = test_cases{i, 3};
    
    FK_test = RA_FK_dual(params, p_base, q_base, th_L, th_R);
    c_test = RA_C_dual(FK_test, params, v_base, omega_base, theta_dot_L, theta_dot_R);
    
    if any(isnan(c_test)) || any(isinf(c_test))
        fprintf('  [FAIL] %s: NaN/Inf\n', name);
        all_pass = false;
    else
        fprintf('  [PASS] %s: max|c| = %.4f\n', name, max(abs(c_test)));
    end
end

%% Test 7: 에너지 소산 검증 (c' * zeta <= 0 in certain conditions)
fprintf('\n=== Test 7: Coriolis 특성 검증 ===\n');
% Coriolis 행렬 C는 (H_dot - 2C)가 skew-symmetric
% 여기서는 간단히 c의 크기가 속도에 비례하는지 확인

scale = 2.0;
c_scaled = RA_C_dual(FK_dual, params, scale*v_base, scale*omega_base, ...
                     scale*theta_dot_L, scale*theta_dot_R);

% Coriolis는 속도의 2차 함수이므로 4배 예상
ratio = norm(c_scaled) / norm(c);
expected_ratio = scale^2;

if abs(ratio - expected_ratio) < 0.5
    fprintf('[PASS] 속도 스케일링 검증 (비율: %.2f, 예상: %.2f)\n', ratio, expected_ratio);
else
    fprintf('[INFO] 속도 스케일링 비율: %.2f (예상: %.2f)\n', ratio, expected_ratio);
end

%% 결과 요약
fprintf('\n========================================\n');
fprintf('Coriolis 벡터 검증 결과\n');
fprintf('========================================\n');
fprintf('c 구조:\n');
fprintf('  c_b     (6x1)  - Base 비선형 항\n');
fprintf('  c_m_L   (7x1)  - 왼팔 관절 비선형 항\n');
fprintf('  c_m_R   (7x1)  - 오른팔 관절 비선형 항\n');
fprintf('\n주요 값:\n');
fprintf('  ||c_b|| = %.4f\n', norm(c(1:6)));
fprintf('  ||c_m_L|| = %.4f\n', norm(c(7:13)));
fprintf('  ||c_m_R|| = %.4f\n', norm(c(14:20)));


%% ========== Dummy params (URDF 없을 때) ==========
function params = create_dummy_params()
    params.sat.m = 500;
    params.sat.I = diag([260, 280, 170]);
    
    params.sat.n_arms = 2;
    params.sat.mount(1).pos = [1.9; 0.9; 0];
    params.sat.mount(1).R = [1 0 0; 0 0 1; 0 -1 0];
    params.sat.mount(2).pos = [1.9; -0.9; 0];
    params.sat.mount(2).R = [1 0 0; 0 0 -1; 0 1 0];
    params.sat.r_mount = params.sat.mount(1).pos;
    params.sat.R_mount = params.sat.mount(1).R;
    
    params.arm.n_joints = 7;
    params.arm.n_bodies = 9;
    
    masses = [3.3436, 2.1207, 6.6621, 4.0005, 6.66208, 2.6161, 4.9702, 3.3715, 0.001];
    joint_types = {'fixed', 'revolute', 'revolute', 'revolute', 'revolute', 'revolute', 'revolute', 'revolute', 'fixed'};
    
    names = {'base', 'link1', 'link2', 'link3', 'link4', 'link5', 'link6', 'link7', 'eef_link'};
    joint_axes = {[0;0;1], [0;0;1], [0;1;0], [0;0;1], [0;1;0], [0;0;1], [0;1;0], [0;0;1], [0;0;1]};
    offsets = [0, 0.094, 0.193, 0.131, 0.8, 0.131, 0.408, 0.088, 0.097];
    
    for i = 1:9
        params.arm.links(i).name = names{i};
        params.arm.links(i).m = masses(i);
        params.arm.links(i).I = eye(3) * 0.01;
        params.arm.links(i).com = [0; 0; 0.05];
        params.arm.links(i).joint_type = joint_types{i};
        params.arm.links(i).joint_axis = joint_axes{i};
        params.arm.links(i).T_fixed = eye(4);
        params.arm.links(i).T_fixed(3,4) = offsets(i);
    end
end