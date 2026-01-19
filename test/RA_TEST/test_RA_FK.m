%% test_RA_FK.m - RA_FK 함수 검증
% MATLAB getTransform과 비교

clear; clc; close all;
addpath(genpath('D:\hanul2026_scie\mppi_RA'));

%% 파라미터 초기화
urdf_path = 'D:\hanul2026_scie\mppi_RA\model\urdf\ASM_KARI_ARM_URDF.urdf';
params = params_init(urdf_path);
robot = params.arm.robot;
n_joints = params.arm.n_joints;

%% 테스트 조건
% 위성: 원점, 단위 자세 (장착점 = r_mount)
p_base = [0; 0; 0];
q_base = [1; 0; 0; 0];

% 관절 각도: 랜덤 또는 지정
theta = [0.1; -0.2; 0.3; -0.1; 0.2; -0.3; 0.1];  % 7-DOF

%% RA_FK 계산
FK = RA_FK(params, p_base, q_base, theta);

%% MATLAB 내장 함수 (getTransform)
% rigidBodyTree는 base에서 시작, 장착점 offset 없음
T_ee_matlab = getTransform(robot, theta, 'eef_link');
p_ee_matlab = T_ee_matlab(1:3, 4) + params.sat.r_mount;  % 장착점 보정
R_ee_matlab = T_ee_matlab(1:3, 1:3);

%% 비교
err_pos = norm(FK.p_ee - p_ee_matlab);
err_rot = norm(FK.R_ee - R_ee_matlab, 'fro');

fprintf('=== RA_FK 검증 결과 ===\n');
fprintf('End-effector 위치 오차: %.2e [m]\n', err_pos);
fprintf('End-effector 자세 오차: %.2e (Frobenius)\n', err_rot);

if err_pos < 1e-6 && err_rot < 1e-6
    fprintf('>> 검증 통과\n');
else
    fprintf('>> 검증 실패 - 확인 필요\n');
    
    fprintf('\nRA_FK p_ee:\n');
    disp(FK.p_ee');
    fprintf('MATLAB p_ee:\n');
    disp(p_ee_matlab');
end