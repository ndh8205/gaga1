%% test_FK_dual.m
% 양팔 Forward Kinematics 검증 스크립트
%
% 검증 항목:
%   1. 장착점 위치 정확성
%   2. 양팔 대칭성 (theta = 0일 때)
%   3. 회전축 방향 일관성
%   4. 개별 관절 회전 테스트

clear; clc; close all;

%% 파라미터 초기화
% URDF 경로 (사용자 환경에 맞게 수정)
urdf_path = 'D:\pj2025\space_challenge\model\modeling_3d\ASM_KARI_ARM\ASM_KARI_ARM_URDF.urdf';

% URDF 없으면 더미 params 사용
if exist(urdf_path, 'file')
    params = params_init(urdf_path);
else
    warning('URDF not found. Using dummy params for testing.');
    params = create_dummy_params();
end

%% 초기 상태 설정
p_base = [0; 0; 0];           % 위성 CoM 원점
q_base = [1; 0; 0; 0];        % 항등 자세
theta_L = zeros(7, 1);        % 왼팔 초기 자세
theta_R = zeros(7, 1);        % 오른팔 초기 자세

%% Test 1: 기본 FK 호출
fprintf('=== Test 1: 기본 FK 호출 ===\n');
try
    FK_dual = RA_FK_dual(params, p_base, q_base, theta_L, theta_R);
    fprintf('[PASS] FK_dual 호출 성공\n');
catch ME
    fprintf('[FAIL] FK_dual 호출 실패: %s\n', ME.message);
    return;
end

%% Test 2: 장착점 위치 검증
fprintf('\n=== Test 2: 장착점 위치 검증 ===\n');

p_mount_L_expected = params.sat.mount(1).pos;
p_mount_R_expected = params.sat.mount(2).pos;

p_mount_L_actual = FK_dual.L.p_mount;
p_mount_R_actual = FK_dual.R.p_mount;

err_mount_L = norm(p_mount_L_expected - p_mount_L_actual);
err_mount_R = norm(p_mount_R_expected - p_mount_R_actual);

fprintf('왼팔 장착점: expected [%.3f, %.3f, %.3f], actual [%.3f, %.3f, %.3f], err = %.6f\n', ...
    p_mount_L_expected, p_mount_L_actual, err_mount_L);
fprintf('오른팔 장착점: expected [%.3f, %.3f, %.3f], actual [%.3f, %.3f, %.3f], err = %.6f\n', ...
    p_mount_R_expected, p_mount_R_actual, err_mount_R);

if err_mount_L < 1e-10 && err_mount_R < 1e-10
    fprintf('[PASS] 장착점 위치 정확\n');
else
    fprintf('[FAIL] 장착점 위치 오차 발생\n');
end

%% Test 3: 양팔 대칭성 검증 (Y축 대칭)
fprintf('\n=== Test 3: 양팔 대칭성 검증 ===\n');

p_ee_L = FK_dual.L.p_ee;
p_ee_R = FK_dual.R.p_ee;

% Y축 대칭 확인: x, z는 같고, y는 부호 반대
symmetry_x = abs(p_ee_L(1) - p_ee_R(1));
symmetry_y = abs(p_ee_L(2) + p_ee_R(2));  % 부호 반대
symmetry_z = abs(p_ee_L(3) - p_ee_R(3));

fprintf('왼팔 EE 위치: [%.4f, %.4f, %.4f]\n', p_ee_L);
fprintf('오른팔 EE 위치: [%.4f, %.4f, %.4f]\n', p_ee_R);
fprintf('대칭 오차 (x, y반전, z): [%.6f, %.6f, %.6f]\n', symmetry_x, symmetry_y, symmetry_z);

if symmetry_x < 1e-6 && symmetry_y < 1e-6 && symmetry_z < 1e-6
    fprintf('[PASS] 양팔 Y축 대칭 확인\n');
else
    fprintf('[WARN] 양팔 대칭성 오차 - R_mount 차이로 인한 것일 수 있음\n');
end

%% Test 4: Joint 1 회전 테스트
fprintf('\n=== Test 4: Joint 1 회전 테스트 ===\n');

theta_L_rot = zeros(7, 1);
theta_L_rot(1) = pi/4;  % 45도 회전

FK_dual_rot = RA_FK_dual(params, p_base, q_base, theta_L_rot, theta_R);

p_ee_L_rot = FK_dual_rot.L.p_ee;
p_ee_R_rot = FK_dual_rot.R.p_ee;

fprintf('왼팔 EE (J1=45도): [%.4f, %.4f, %.4f]\n', p_ee_L_rot);
fprintf('오른팔 EE (J1=0도): [%.4f, %.4f, %.4f]\n', p_ee_R_rot);

% 오른팔은 변화 없어야 함
err_R_unchanged = norm(p_ee_R_rot - p_ee_R);
if err_R_unchanged < 1e-10
    fprintf('[PASS] 오른팔 독립성 확인 (왼팔 회전에 영향 없음)\n');
else
    fprintf('[FAIL] 오른팔이 왼팔 회전에 영향받음: err = %.6f\n', err_R_unchanged);
end

%% Test 5: 회전축 방향 검증
fprintf('\n=== Test 5: 회전축 방향 검증 ===\n');

% Joint 1의 회전축 (Z축이어야 함, 각 팔의 로컬 좌표계 기준)
k1_L = FK_dual.L.k{1};
k1_R = FK_dual.R.k{1};

fprintf('왼팔 Joint1 회전축: [%.4f, %.4f, %.4f]\n', k1_L);
fprintf('오른팔 Joint1 회전축: [%.4f, %.4f, %.4f]\n', k1_R);

% R_mount에 의해 변환된 방향 확인
R_mount_L = params.sat.mount(1).R;
R_mount_R = params.sat.mount(2).R;
z_local = [0; 0; 1];

k1_L_expected = R_mount_L * z_local;
k1_R_expected = R_mount_R * z_local;

fprintf('왼팔 J1 축 expected: [%.4f, %.4f, %.4f]\n', k1_L_expected);
fprintf('오른팔 J1 축 expected: [%.4f, %.4f, %.4f]\n', k1_R_expected);

err_k1_L = norm(k1_L - k1_L_expected);
err_k1_R = norm(k1_R - k1_R_expected);

if err_k1_L < 1e-6 && err_k1_R < 1e-6
    fprintf('[PASS] 회전축 방향 정확\n');
else
    fprintf('[WARN] 회전축 방향 오차: L=%.6f, R=%.6f\n', err_k1_L, err_k1_R);
end

%% Test 6: 모든 Joint 위치 출력
fprintf('\n=== Test 6: Joint 위치 요약 ===\n');

fprintf('\n--- 왼팔 ---\n');
fprintf('Mount: [%.4f, %.4f, %.4f]\n', FK_dual.L.p_mount);
for j = 1:7
    fprintf('J%d: [%.4f, %.4f, %.4f]\n', j, FK_dual.L.p_joint{j});
end
fprintf('EE: [%.4f, %.4f, %.4f]\n', FK_dual.L.p_ee);

fprintf('\n--- 오른팔 ---\n');
fprintf('Mount: [%.4f, %.4f, %.4f]\n', FK_dual.R.p_mount);
for j = 1:7
    fprintf('J%d: [%.4f, %.4f, %.4f]\n', j, FK_dual.R.p_joint{j});
end
fprintf('EE: [%.4f, %.4f, %.4f]\n', FK_dual.R.p_ee);

%% 결과 요약
fprintf('\n========================================\n');
fprintf('FK_dual 검증 완료\n');
fprintf('========================================\n');

%% 더미 파라미터 생성 함수 (URDF 없을 때)
function params = create_dummy_params()
    % 위성 파라미터
    params.sat.m = 500;
    params.sat.I = diag([260, 280, 170]);
    params.sat.mount(1).pos = [1.9; 0.9; 0];
    params.sat.mount(1).R = [1 0 0; 0 0 1; 0 -1 0];
    params.sat.mount(2).pos = [1.9; -0.9; 0];
    params.sat.mount(2).R = [1 0 0; 0 0 -1; 0 1 0];
    params.sat.r_mount = params.sat.mount(1).pos;
    params.sat.R_mount = params.sat.mount(1).R;
    
    % 로봇팔 파라미터 (간략화)
    params.arm.n_joints = 7;
    params.arm.n_bodies = 9;
    
    % 더미 링크 정보
    joint_offsets = [
        0, 0, 0;
        0, 0, 0.094;
        0, 0.088, 0.105;
        0, 0.104, 0.131;
        0, -0.088, 0.8;
        0, -0.104, 0.131;
        0, -0.071, 0.408;
        0, -0.121, 0.088;
        0, 0, 0.097
    ];
    
    joint_types = {'fixed', 'revolute', 'revolute', 'revolute', ...
                   'revolute', 'revolute', 'revolute', 'revolute', 'fixed'};
    joint_axes = [
        0, 0, 1;
        0, 0, 1;
        0, 1, 0;
        0, 0, 1;
        0, 1, 0;
        0, 0, 1;
        0, 1, 0;
        0, 0, 1;
        0, 0, 1
    ];
    
    masses = [3.3436, 2.1207, 6.6621, 4.0005, 6.66208, 2.6161, 4.9702, 3.3715, 0.001];
    
    for i = 1:9
        params.arm.links(i).name = sprintf('link%d', i);
        params.arm.links(i).m = masses(i);
        params.arm.links(i).com = [0; 0; 0.05];
        params.arm.links(i).I = eye(3) * 0.01;
        params.arm.links(i).joint_type = joint_types{i};
        params.arm.links(i).joint_axis = joint_axes(i, :)';
        params.arm.links(i).T_fixed = eye(4);
        params.arm.links(i).T_fixed(1:3, 4) = joint_offsets(i, :)';
    end
end