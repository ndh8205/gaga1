%% verify_jacobian_gen.m
% Generalized Jacobian 검증
%
% 검증 방법:
%   1. 수치 미분과 비교 (정적)
%   2. 시뮬레이션에서 ẋ_ee = J* * θ̇ 확인 (동적)
% =========================================================================

clear; clc; close all;
addpath(genpath('D:\pj2025\space_challenge'));

r2d = 180/pi;
d2r = pi/180;

%% ========== 파라미터 로드 ==========
urdf_path = 'D:\pj2025\space_challenge\model\modeling_3d\ASM_KARI_ARM\ASM_KARI_ARM_URDF.urdf';
params = params_init(urdf_path);

fprintf('===========================================\n');
fprintf('  Generalized Jacobian 검증\n');
fprintf('===========================================\n\n');

%% ========== 테스트 1: 수치 미분 비교 (정적) ==========
fprintf('--- 테스트 1: 수치 미분 비교 ---\n\n');

% 테스트 구성 (여러 자세)
test_configs = {
    zeros(7, 1),                                    % 영점
    [30; 0; 0; 0; 0; 0; 0] * d2r,                   % J1만
    [0; 45; 0; 30; 0; 0; 0] * d2r,                  % J2, J4
    [20; 30; 15; 25; 10; 20; 5] * d2r               % 전체
};

config_names = {'영점', 'J1=30°', 'J2=45°,J4=30°', '전체 랜덤'};

delta = 1e-6;  % 수치 미분용 미소 변위

for cfg_idx = 1:length(test_configs)
    theta = test_configs{cfg_idx};
    
    fprintf('구성 %d: %s\n', cfg_idx, config_names{cfg_idx});
    
    % 초기 상태 (운동량 0 가정)
    p_base = [0; 0; 2];
    q_base = [1; 0; 0; 0];
    
    % FK 및 Jacobian 계산
    FK = RA_FK(params, p_base, q_base, theta);
    Jg = RA_Jacobian_Gen(FK, params);
    
    % 수치 Jacobian 계산
    Jg_num = zeros(6, 7);
    
    for j = 1:7
        % +delta
        theta_p = theta;
        theta_p(j) = theta_p(j) + delta;
        
        % 운동량 보존으로 Base 상태 변화 계산
        % 단순화: 미소 변위이므로 Base 변화 무시하고 EE만 계산
        FK_p = RA_FK(params, p_base, q_base, theta_p);
        p_ee_p = FK_p.p_ee;
        R_ee_p = FK_p.R_ee;
        
        % -delta
        theta_m = theta;
        theta_m(j) = theta_m(j) - delta;
        FK_m = RA_FK(params, p_base, q_base, theta_m);
        p_ee_m = FK_m.p_ee;
        R_ee_m = FK_m.R_ee;
        
        % 선속도 Jacobian (위치 미분)
        Jg_num(1:3, j) = (p_ee_p - p_ee_m) / (2 * delta);
        
        % 각속도 Jacobian (회전 미분)
        % R_diff = R_ee_p * R_ee_m' ≈ I + [δω]_x
        R_diff = R_ee_p * R_ee_m';
        % 회전 벡터 추출 (소각도 근사)
        omega_delta = [R_diff(3,2) - R_diff(2,3);
                       R_diff(1,3) - R_diff(3,1);
                       R_diff(2,1) - R_diff(1,2)] / 2;
        Jg_num(4:6, j) = omega_delta / (2 * delta);
    end
    
    % 오차 계산
    error_lin = Jg(1:3, :) - Jg_num(1:3, :);
    error_ang = Jg(4:6, :) - Jg_num(4:6, :);
    
    fprintf('  선속도 Jacobian 최대 오차: %.2e\n', max(abs(error_lin(:))));
    fprintf('  각속도 Jacobian 최대 오차: %.2e\n', max(abs(error_ang(:))));
    fprintf('\n');
end

%% ========== 테스트 2: 동적 검증 (시뮬레이션) ==========
fprintf('--- 테스트 2: 동적 검증 (시뮬레이션) ---\n\n');

% 시뮬레이션 설정
dt = 0.01;
T_sim = 2;
N = T_sim / dt + 1;
time = 0:dt:T_sim;

% 초기 조건
p_base = [0; 0; 2];
q_base = [1; 0; 0; 0];
v_base = [0; 0; 0];
omega_base = [0; 0; 0];
theta = [20; 10; 15; 25; 10; 5; 0] * d2r;  % 비영점 초기자세
theta_dot = zeros(7, 1);

% 토크 입력 (sinusoidal)
tau_amp = [0.5; 0.3; 0.2; 0.4; 0.2; 0.1; 0.05];

% 데이터 저장
data.v_ee_actual = zeros(N, 3);     % 실제 EE 선속도 (수치 미분)
data.v_ee_jac = zeros(N, 3);        % J* * θ̇ 예측
data.w_ee_actual = zeros(N, 3);     % 실제 EE 각속도 (수치 미분)
data.w_ee_jac = zeros(N, 3);        % J* * θ̇ 예측
data.p_ee = zeros(N, 3);
data.theta_dot = zeros(N, 7);

% 이전 EE pose 저장 (수치 미분용)
FK_prev = RA_FK(params, p_base, q_base, theta);
p_ee_prev = FK_prev.p_ee;
R_ee_prev = FK_prev.R_ee;

fprintf('시뮬레이션 시작...\n');

for i = 1:N
    t = time(i);
    
    % 토크
    tau = tau_amp .* sin(2 * pi * 0.5 * t);
    
    % FK
    FK = RA_FK(params, p_base, q_base, theta);
    
    % Generalized Jacobian
    Jg = RA_Jacobian_Gen(FK, params);
    
    % 예측 EE 속도
    x_dot_pred = Jg * theta_dot;
    
    % 실제 EE 속도 (수치 미분)
    if i > 1
        v_ee_actual = (FK.p_ee - p_ee_prev) / dt;
        
        % 각속도 (회전행렬 미분)
        R_diff = FK.R_ee * R_ee_prev';
        omega_delta = [R_diff(3,2) - R_diff(2,3);
                       R_diff(1,3) - R_diff(3,1);
                       R_diff(2,1) - R_diff(1,2)] / 2;
        w_ee_actual = omega_delta / dt;
    else
        v_ee_actual = [0; 0; 0];
        w_ee_actual = [0; 0; 0];
    end
    
    % 저장
    data.v_ee_actual(i, :) = v_ee_actual';
    data.v_ee_jac(i, :) = x_dot_pred(1:3)';
    data.w_ee_actual(i, :) = w_ee_actual';
    data.w_ee_jac(i, :) = x_dot_pred(4:6)';
    data.p_ee(i, :) = FK.p_ee';
    data.theta_dot(i, :) = theta_dot';
    
    % 이전 값 저장
    p_ee_prev = FK.p_ee;
    R_ee_prev = FK.R_ee;
    
    % 적분
    if i < N
        tau_func = @(t_) tau;
        [p_base, q_base, v_base, omega_base, theta, theta_dot] = ...
            rk4_ff(p_base, q_base, v_base, omega_base, theta, theta_dot, tau_func, t, dt, params);
    end
end

fprintf('시뮬레이션 완료!\n\n');

% 오차 계산 (첫 프레임 제외)
v_error = data.v_ee_actual(2:end, :) - data.v_ee_jac(2:end, :);
w_error = data.w_ee_actual(2:end, :) - data.w_ee_jac(2:end, :);

fprintf('--- 동적 검증 결과 ---\n');
fprintf('선속도 오차 (RMS): [%.4e, %.4e, %.4e] m/s\n', rms(v_error));
fprintf('각속도 오차 (RMS): [%.4e, %.4e, %.4e] rad/s\n', rms(w_error));
fprintf('선속도 오차 (MAX): %.4e m/s\n', max(abs(v_error(:))));
fprintf('각속도 오차 (MAX): %.4e rad/s\n', max(abs(w_error(:))));

%% ========== 플롯: 속도 비교 ==========
figure('Name', 'Generalized Jacobian 검증', 'Position', [100, 100, 1200, 600]);

% 선속도
subplot(2,3,1);
plot(time, data.v_ee_actual(:,1), 'b-', 'LineWidth', 1.5); hold on;
plot(time, data.v_ee_jac(:,1), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('v_x [m/s]');
title('EE 선속도 X');
legend('실제', 'J*θ̇', 'Location', 'best');
grid on;

subplot(2,3,2);
plot(time, data.v_ee_actual(:,2), 'b-', 'LineWidth', 1.5); hold on;
plot(time, data.v_ee_jac(:,2), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('v_y [m/s]');
title('EE 선속도 Y');
grid on;

subplot(2,3,3);
plot(time, data.v_ee_actual(:,3), 'b-', 'LineWidth', 1.5); hold on;
plot(time, data.v_ee_jac(:,3), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('v_z [m/s]');
title('EE 선속도 Z');
grid on;

% 각속도
subplot(2,3,4);
plot(time, data.w_ee_actual(:,1)*r2d, 'b-', 'LineWidth', 1.5); hold on;
plot(time, data.w_ee_jac(:,1)*r2d, 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('\omega_x [deg/s]');
title('EE 각속도 X');
legend('실제', 'J*θ̇', 'Location', 'best');
grid on;

subplot(2,3,5);
plot(time, data.w_ee_actual(:,2)*r2d, 'b-', 'LineWidth', 1.5); hold on;
plot(time, data.w_ee_jac(:,2)*r2d, 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('\omega_y [deg/s]');
title('EE 각속도 Y');
grid on;

subplot(2,3,6);
plot(time, data.w_ee_actual(:,3)*r2d, 'b-', 'LineWidth', 1.5); hold on;
plot(time, data.w_ee_jac(:,3)*r2d, 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('\omega_z [deg/s]');
title('EE 각속도 Z');
grid on;

sgtitle('Generalized Jacobian 검증: ẋ_{ee} = J^* θ̇');

%% ========== 플롯: 오차 ==========
figure('Name', 'Jacobian 오차', 'Position', [150, 150, 800, 400]);

subplot(1,2,1);
plot(time(2:end), v_error * 1000, 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Error [mm/s]');
title('선속도 오차');
legend('x', 'y', 'z', 'Location', 'best');
grid on;

subplot(1,2,2);
plot(time(2:end), w_error * r2d, 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Error [deg/s]');
title('각속도 오차');
legend('x', 'y', 'z', 'Location', 'best');
grid on;

sgtitle('Generalized Jacobian 예측 오차');

fprintf('\n===========================================\n');
fprintf('  검증 완료\n');
fprintf('===========================================\n');