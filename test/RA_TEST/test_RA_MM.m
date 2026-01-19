%% test_RA_MM.m - RA_MM 함수 검증
% MATLAB massMatrix와 비교 (관절 부분만)

clear; clc; close all;
addpath(genpath('D:\hanul2026_scie\mppi_RA'));

%% 파라미터 초기화
urdf_path = 'D:\hanul2026_scie\mppi_RA\model\urdf\ASM_KARI_ARM_URDF.urdf';
params = params_init(urdf_path);
robot = params.arm.robot;
n_joints = params.arm.n_joints;

%% 테스트 조건
p_base = [0; 0; 0];
q_base = [1; 0; 0; 0];

% theta=0에서 먼저 테스트 (단순화)
theta_zero = zeros(7,1);
theta = theta_zero;  % 먼저 0으로 테스트

%% FK 및 Mass Matrix 계산
FK = RA_FK(params, p_base, q_base, theta);
H = RA_MM(FK, params);

% theta=0 전용 FK (검증용)
FK_zero = FK;  % theta=0이므로 동일

%% MATLAB 내장 함수 (massMatrix)
% massMatrix: nxn (고정 base 가정)
H_matlab = massMatrix(robot, theta);

%% 비교 (H_m' 부분만)
% H의 우하단 nxn 블록 = H_m'
H_m = H(7:end, 7:end);
err_Hm = norm(H_m - H_matlab, 'fro');

fprintf('=== RA_MM 검증 결과 ===\n');
fprintf('H_m (매니퓰레이터 관성) 오차: %.2e (Frobenius)\n', err_Hm);

%% 디버깅: 링크 구조 확인
fprintf('\n=== 링크 구조 확인 ===\n');
fprintf('n_bodies: %d, n_joints: %d\n', params.arm.n_bodies, params.arm.n_joints);
for i = 1:params.arm.n_bodies
    link = params.arm.links(i);
    fprintf('[%d] %s: joint_type=%s, m=%.4f\n', i, link.name, link.joint_type, link.m);
end

%% 디버깅: 대각 성분 비교
fprintf('\n=== H_m 대각 성분 비교 ===\n');
fprintf('Joint | RA_MM    | MATLAB   | Diff\n');
for j = 1:n_joints
    fprintf('  %d   | %.4f | %.4f | %.2e\n', j, H_m(j,j), H_matlab(j,j), abs(H_m(j,j)-H_matlab(j,j)));
end

%% 디버깅: p_joint 검증 (theta=0)
fprintf('\n=== p_joint 검증 (theta=0) ===\n');
for i = 1:n_joints
    link_name = sprintf('link%d', i);
    T_link = getTransform(robot, theta_zero, link_name);
    
    % MATLAB: joint i 위치 = link i frame 원점
    p_joint_matlab = T_link(1:3, 4) + params.sat.r_mount;
    p_joint_ours = FK_zero.p_joint{i};
    
    err_joint = norm(p_joint_ours - p_joint_matlab);
    fprintf('Joint %d 위치 오차: %.2e\n', i, err_joint);
    if err_joint > 1e-6
        fprintf('  Ours:   [%.4f, %.4f, %.4f]\n', p_joint_ours(1), p_joint_ours(2), p_joint_ours(3));
        fprintf('  MATLAB: [%.4f, %.4f, %.4f]\n', p_joint_matlab(1), p_joint_matlab(2), p_joint_matlab(3));
    end
end
theta_zero = zeros(7,1);
FK_zero = RA_FK(params, p_base, q_base, theta_zero);

% 각 link의 transform에서 CoM 계산
for i = 1:n_joints
    link_name = sprintf('link%d', i);
    T_link = getTransform(robot, theta_zero, link_name);
    
    % link frame에서의 CoM (params에서)
    link_idx = i + 1;  % base가 index 1
    com_local = params.arm.links(link_idx).com;
    
    % world frame에서의 CoM
    p_com_matlab = T_link(1:3,4) + T_link(1:3,1:3)*com_local + params.sat.r_mount;
    p_com_ours = FK_zero.p_com{i};
    
    err_com = norm(p_com_ours - p_com_matlab);
    fprintf('Link %d CoM 오차: %.2e\n', i, err_com);
end

%% 디버깅: 관성텐서 검증
fprintf('\n=== 관성텐서 검증 (theta=0) ===\n');
for i = 1:n_joints
    link_name = sprintf('link%d', i);
    T_link = getTransform(robot, theta_zero, link_name);
    R_matlab = T_link(1:3, 1:3);
    
    link_idx = i + 1;
    I_body = params.arm.links(link_idx).I;
    
    % World frame 관성텐서
    I_world_matlab = R_matlab * I_body * R_matlab';
    I_world_ours = FK_zero.R{i} * I_body * FK_zero.R{i}';
    
    % R 비교
    err_R = norm(FK_zero.R{i} - R_matlab, 'fro');
    err_I = norm(I_world_ours - I_world_matlab, 'fro');
    fprintf('Link %d: R오차=%.2e, I오차=%.2e\n', i, err_R, err_I);
end

%% 디버깅: robot.Bodies 순서 확인
fprintf('\n=== robot.Bodies 순서 확인 ===\n');
fprintf('NumBodies = %d\n', robot.NumBodies);
for i = 1:robot.NumBodies
    body = robot.Bodies{i};
    fprintf('[%d] %s: m=%.4f\n', i, body.Name, body.Mass);
end
fprintf('BaseName = %s\n', robot.BaseName);

%% Link 1 상세 분석 먼저
fprintf('\n--- Link 1 상세 분석 ---\n');
k_1 = FK.k{1};
p_com_1 = FK.p_com{1};
p_joint_1 = FK.p_joint{1};
r_11 = p_com_1 - p_joint_1;

fprintf('k_1 = [%.6f, %.6f, %.6f]\n', k_1(1), k_1(2), k_1(3));
fprintf('p_joint_1 = [%.6f, %.6f, %.6f]\n', p_joint_1(1), p_joint_1(2), p_joint_1(3));
fprintf('p_com_1 = [%.6f, %.6f, %.6f]\n', p_com_1(1), p_com_1(2), p_com_1(3));
fprintf('r_11 = [%.6f, %.6f, %.6f]\n', r_11(1), r_11(2), r_11(3));

J_T1_1 = cross(k_1, r_11);
fprintf('J_T{1}(:,1) = [%.6f, %.6f, %.6f]\n', J_T1_1(1), J_T1_1(2), J_T1_1(3));

%% base 링크 (URDF의 base, fixed joint) 관성 확인
fprintf('\n--- URDF base 링크 확인 ---\n');
base_link = params.arm.links(1);
fprintf('base mass = %.4f\n', base_link.m);
fprintf('base I_zz = %.6f\n', base_link.I(3,3));
fprintf('base joint_type = %s\n', base_link.joint_type);

%% 오차 원인 분석: base 링크 관성이 빠졌나?
% MATLAB massMatrix는 base를 고정으로 가정하므로 base 관성 제외
% 우리도 revolute만 포함하므로 base 제외... 동일해야 함

fprintf('\n');
% H_m(1,1) = sum_i { m_i * J_Ti(1:3,1)^T * J_Ti(1:3,1) + J_Ri(1:3,1)^T * I_i * J_Ri(1:3,1) }
% Joint 1에 대해서만 분석
Hm_11_trans = 0;
Hm_11_rot = 0;
for i = 1:n_joints
    link_idx = i + 1;
    m_i = params.arm.links(link_idx).m;
    
    k_1 = FK.k{1};
    r_1i = FK.p_com{i} - FK.p_joint{1};
    J_Ti_1 = cross(k_1, r_1i);  % J_T{i}(:,1)
    
    if i >= 1
        J_Ri_1 = k_1;  % J_R{i}(:,1)
    else
        J_Ri_1 = [0;0;0];
    end
    
    I_i = FK.R{i} * params.arm.links(link_idx).I * FK.R{i}';
    
    contrib_trans = m_i * (J_Ti_1' * J_Ti_1);
    contrib_rot = J_Ri_1' * I_i * J_Ri_1;
    
    Hm_11_trans = Hm_11_trans + contrib_trans;
    Hm_11_rot = Hm_11_rot + contrib_rot;
    
    fprintf('Link %d: trans=%.4f, rot=%.4f\n', i, contrib_trans, contrib_rot);
end
fprintf('Total: trans=%.4f + rot=%.4f = %.4f\n', Hm_11_trans, Hm_11_rot, Hm_11_trans + Hm_11_rot);
fprintf('MATLAB H(1,1) = %.4f\n', H_matlab(1,1));

%% 에너지 일관성 테스트
% T = (1/2) * q_dot^T * H_m * q_dot
% 두 방법으로 계산한 에너지가 같아야 함
fprintf('\n=== 에너지 일관성 테스트 ===\n');
q_dot_test = [0.1; 0.2; 0.1; 0.15; 0.1; 0.05; 0.02];

T_ours = 0.5 * q_dot_test' * H_m * q_dot_test;
T_matlab = 0.5 * q_dot_test' * H_matlab * q_dot_test;

fprintf('운동에너지 (RA_MM): %.6f\n', T_ours);
fprintf('운동에너지 (MATLAB): %.6f\n', T_matlab);
fprintf('에너지 차이: %.2e (%.2f%%)\n', abs(T_ours - T_matlab), 100*abs(T_ours - T_matlab)/T_matlab);

%% 대칭성 검사
sym_err = norm(H - H', 'fro');
fprintf('\n전체 H 대칭성 오차: %.2e\n', sym_err);

%% 양정치 검사
eig_H = eig(H);
fprintf('H 최소 고유값: %.4f\n', min(eig_H));

if err_Hm < 1e-6 && sym_err < 1e-10 && min(eig_H) > 0
    fprintf('>> 검증 통과\n');
else
    fprintf('>> 검증 실패 - 확인 필요\n');
end