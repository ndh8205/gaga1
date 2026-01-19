%% params_init 검증 스크립트
clear all; close all; clc;

%% URDF 경로 설정
addpath(genpath('D:\hanul2026_scie\mppi_RA'));
urdf_path = 'D:\hanul2026_scie\mppi_RA\model\urdf\ASM_KARI_ARM_URDF.urdf';

%% 파라미터 초기화
params = params_init(urdf_path);

%% 위성 파라미터 확인
fprintf('========== SATELLITE ==========\n');
fprintf('질량: %.1f kg\n', params.sat.m);
fprintf('크기: [%.1f, %.1f, %.1f] m\n', params.sat.size);
fprintf('장착점: [%.1f, %.1f, %.1f] m\n', params.sat.r_mount);
fprintf('관성텐서:\n');
disp(params.sat.I);

%% 로봇팔 기본 정보
fprintf('========== ROBOT ARM ==========\n');
fprintf('총 바디 수: %d\n', params.arm.n_bodies);
fprintf('관절 수 (revolute): %d\n', params.arm.n_joints);

%% 각 링크 정보 출력
fprintf('\n---------- 링크별 정보 ----------\n');
for i = 1:params.arm.n_bodies
    link = params.arm.links(i);
    fprintf('\n[%d] %s\n', i, link.name);
    fprintf('    질량: %.4f kg\n', link.m);
    fprintf('    CoM: [%.4f, %.4f, %.4f]\n', link.com);
    fprintf('    부모: %s\n', link.parent);
    fprintf('    관절: %s (%s)\n', link.joint_name, link.joint_type);
    if strcmp(link.joint_type, 'revolute')
        fprintf('    관절축: [%.1f, %.1f, %.1f]\n', link.joint_axis);
    end
end

%% 총 질량 계산
total_arm_mass = 0;
for i = 1:params.arm.n_bodies
    total_arm_mass = total_arm_mass + params.arm.links(i).m;
end
fprintf('\n========== 시스템 총 질량 ==========\n');
fprintf('위성: %.1f kg\n', params.sat.m);
fprintf('로봇팔: %.4f kg\n', total_arm_mass);
fprintf('합계: %.4f kg\n', params.sat.m + total_arm_mass);

%% 환경 확인
fprintf('\n========== ENVIRONMENT ==========\n');
fprintf('중력: [%.1f, %.1f, %.1f] m/s^2\n', params.env.g);

fprintf('\n검증 완료!\n');