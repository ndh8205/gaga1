%% test_RA_Jacobian.m - RA_Jacobian 함수 검증
% MATLAB geometricJacobian과 비교

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
theta = [0.1; -0.2; 0.3; -0.1; 0.2; -0.3; 0.1];

%% FK 및 Jacobian 계산
FK = RA_FK(params, p_base, q_base, theta);
[Jb, Jm] = RA_Jacobian(FK, params);

%% MATLAB 내장 함수 (geometricJacobian)
% geometricJacobian: 6xn, [Jw; Jv] 순서 (각속도 먼저)
J_matlab = geometricJacobian(robot, theta, 'eef_link');

% 순서 변환: [Jw; Jv] -> [Jv; Jw]
Jm_matlab = [J_matlab(4:6, :); J_matlab(1:3, :)];

%% 비교
err_Jm = norm(Jm - Jm_matlab, 'fro');

fprintf('=== RA_Jacobian 검증 결과 ===\n');
fprintf('Manipulator Jacobian 오차: %.2e (Frobenius)\n', err_Jm);

if err_Jm < 1e-6
    fprintf('>> 검증 통과\n');
else
    fprintf('>> 검증 실패 - 확인 필요\n');
    fprintf('\nRA_Jacobian Jm:\n');
    disp(Jm);
    fprintf('MATLAB Jm:\n');
    disp(Jm_matlab);
end