%% test_Jacobian_dual.m - RA_Jacobian_dual 검증 테스트
clear; clc;

%% 파라미터 로드
urdf_path = 'D:\pj2025\space_challenge\model\modeling_3d\ASM_KARI_ARM\ASM_KARI_ARM_URDF.urdf';
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
theta_L = zeros(n_joints, 1);
theta_R = zeros(n_joints, 1);

%% FK
fprintf('=== Jacobian 계산 ===\n');
FK_dual = RA_FK_dual(params, p_base, q_base, theta_L, theta_R);

%% RA_Jacobian_dual 호출
try
    [Jb_L, Jm_L, Jb_R, Jm_R] = RA_Jacobian_dual(FK_dual, params);
    fprintf('[PASS] RA_Jacobian_dual 호출 성공\n');
    fprintf('Jb_L: %dx%d, Jm_L: %dx%d\n', size(Jb_L,1), size(Jb_L,2), size(Jm_L,1), size(Jm_L,2));
    fprintf('Jb_R: %dx%d, Jm_R: %dx%d\n', size(Jb_R,1), size(Jb_R,2), size(Jm_R,1), size(Jm_R,2));
catch ME
    fprintf('[FAIL] 오류: %s\n', ME.message);
    return;
end

%% Test 1: 크기 검증
fprintf('\n=== Test 1: 크기 검증 ===\n');
pass = true;
if ~isequal(size(Jb_L), [6, 6]), fprintf('[FAIL] Jb_L 크기\n'); pass = false; end
if ~isequal(size(Jm_L), [6, 7]), fprintf('[FAIL] Jm_L 크기\n'); pass = false; end
if ~isequal(size(Jb_R), [6, 6]), fprintf('[FAIL] Jb_R 크기\n'); pass = false; end
if ~isequal(size(Jm_R), [6, 7]), fprintf('[FAIL] Jm_R 크기\n'); pass = false; end
if pass
    fprintf('[PASS] 모든 Jacobian 크기 정확\n');
end

%% Test 2: 싱글암 비교
fprintf('\n=== Test 2: 싱글암 비교 ===\n');

% 왼팔 싱글암 Jacobian
[Jb_single, Jm_single] = RA_Jacobian(FK_dual.L, params);

Jb_err = norm(Jb_L - Jb_single, 'fro');
Jm_err = norm(Jm_L - Jm_single, 'fro');

if Jb_err < 1e-10 && Jm_err < 1e-10
    fprintf('[PASS] 왼팔 Jacobian 싱글암과 일치 (Jb_err=%.2e, Jm_err=%.2e)\n', Jb_err, Jm_err);
else
    fprintf('[FAIL] 왼팔 Jacobian 불일치 (Jb_err=%.2e, Jm_err=%.2e)\n', Jb_err, Jm_err);
end

%% Test 3: EE 속도 계산 검증
fprintf('\n=== Test 3: EE 속도 계산 ===\n');

v_base = [0.1; 0.05; 0.02];
omega_base = [0.01; 0.02; 0.03];
theta_dot_L = [0.1; 0.2; 0.15; 0.1; 0.05; 0.08; 0.03];
theta_dot_R = [0.12; 0.18; 0.1; 0.08; 0.06; 0.04; 0.02];

% EE 속도 계산
v_ee_L = Jb_L * [v_base; omega_base] + Jm_L * theta_dot_L;
v_ee_R = Jb_R * [v_base; omega_base] + Jm_R * theta_dot_R;

fprintf('왼팔 EE 속도:\n');
fprintf('  선속도: [%.4f, %.4f, %.4f] m/s\n', v_ee_L(1:3));
fprintf('  각속도: [%.4f, %.4f, %.4f] rad/s\n', v_ee_L(4:6));
fprintf('오른팔 EE 속도:\n');
fprintf('  선속도: [%.4f, %.4f, %.4f] m/s\n', v_ee_R(1:3));
fprintf('  각속도: [%.4f, %.4f, %.4f] rad/s\n', v_ee_R(4:6));

if ~any(isnan([v_ee_L; v_ee_R])) && ~any(isinf([v_ee_L; v_ee_R]))
    fprintf('[PASS] EE 속도 계산 성공 (NaN/Inf 없음)\n');
else
    fprintf('[FAIL] EE 속도 NaN/Inf\n');
end

%% Test 4: Base만 움직일 때 양팔 EE 속도
fprintf('\n=== Test 4: Base만 움직일 때 ===\n');

% 관절 속도 0
v_ee_L_base = Jb_L * [v_base; omega_base];
v_ee_R_base = Jb_R * [v_base; omega_base];

fprintf('왼팔 EE 선속도: [%.4f, %.4f, %.4f]\n', v_ee_L_base(1:3));
fprintf('오른팔 EE 선속도: [%.4f, %.4f, %.4f]\n', v_ee_R_base(1:3));

% 순수 병진: 양팔 EE 선속도 동일
v_base_only = [0.1; 0; 0];
omega_zero = zeros(3,1);
v_ee_L_trans = Jb_L * [v_base_only; omega_zero];
v_ee_R_trans = Jb_R * [v_base_only; omega_zero];

trans_err = norm(v_ee_L_trans(1:3) - v_ee_R_trans(1:3));
if trans_err < 1e-10
    fprintf('[PASS] 순수 병진 시 양팔 EE 선속도 동일 (오차: %.2e)\n', trans_err);
else
    fprintf('[FAIL] 순수 병진 시 양팔 EE 선속도 불일치 (오차: %.2e)\n', trans_err);
end

%% Test 5: Rank 검증
fprintf('\n=== Test 5: Jacobian Rank ===\n');

rank_Jm_L = rank(Jm_L);
rank_Jm_R = rank(Jm_R);

fprintf('Jm_L rank: %d (max 6)\n', rank_Jm_L);
fprintf('Jm_R rank: %d (max 6)\n', rank_Jm_R);

if rank_Jm_L == 6 && rank_Jm_R == 6
    fprintf('[PASS] 영자세에서 full rank (비특이)\n');
else
    fprintf('[INFO] 영자세에서 rank 부족 (특이점 근처 가능)\n');
end

%% Test 6: 다양한 자세
fprintf('\n=== Test 6: 다양한 자세 ===\n');

test_cases = {
    '영자세', zeros(7,1), zeros(7,1);
    '왼팔 J2=45도', [0;pi/4;0;0;0;0;0], zeros(7,1);
    '양팔 굽힘', [0;pi/4;0;pi/3;0;0;0], [0;pi/4;0;pi/3;0;0;0];
    '랜덤', rand(7,1)*pi/2-pi/4, rand(7,1)*pi/2-pi/4;
};

for i = 1:size(test_cases, 1)
    name = test_cases{i, 1};
    th_L = test_cases{i, 2};
    th_R = test_cases{i, 3};
    
    FK_test = RA_FK_dual(params, p_base, q_base, th_L, th_R);
    [Jb_L_t, Jm_L_t, Jb_R_t, Jm_R_t] = RA_Jacobian_dual(FK_test, params);
    
    % NaN/Inf 확인
    all_J = [Jb_L_t(:); Jm_L_t(:); Jb_R_t(:); Jm_R_t(:)];
    if any(isnan(all_J)) || any(isinf(all_J))
        fprintf('  [FAIL] %s: NaN/Inf\n', name);
    else
        fprintf('  [PASS] %s: rank(Jm_L)=%d, rank(Jm_R)=%d\n', name, rank(Jm_L_t), rank(Jm_R_t));
    end
end

%% 결과 요약
fprintf('\n========================================\n');
fprintf('Jacobian 검증 결과\n');
fprintf('========================================\n');
fprintf('Jacobian 구조:\n');
fprintf('  Jb_L, Jb_R: 6x6 (Base → EE)\n');
fprintf('  Jm_L, Jm_R: 6x7 (Joint → EE)\n');
fprintf('\n통합 Jacobian (12x20):\n');
fprintf('  J_dual = [Jb_L  Jm_L    0  ]\n');
fprintf('           [Jb_R    0   Jm_R ]\n');


%% ========== Dummy params ==========
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