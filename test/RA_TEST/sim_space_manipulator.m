%% sim_space_manipulator.m - Space Manipulator 동역학 시뮬레이션
% Simscape 비교용: sin(t) 토크 입력
%
% 동역학 방정식: H * zeta_ddot + c = tau_gen
% Free-floating: f_base = 0, tau_base = 0

clear; clc; close all;
addpath(genpath('D:\hanul2026_scie\mppi_RA'));

%% 파라미터 초기화
urdf_path = 'D:\hanul2026_scie\mppi_RA\model\urdf\ASM_KARI_ARM_URDF.urdf';
params = params_init(urdf_path);

%% 시뮬레이션 설정
dt = 0.001;          % [sec]
t_end = 5.0;         % [sec]
t = 0:dt:t_end;
N = length(t);

n_joints = params.arm.n_joints;  % 7

%% 초기 조건
% Base 상태
p_base_0 = [0; 0; 0];
q_base_0 = [1; 0; 0; 0];  % Identity quaternion
v_base_0 = [0; 0; 0];
omega_base_0 = [0; 0; 0];

% 관절 상태
theta_0 = zeros(n_joints, 1);
theta_dot_0 = zeros(n_joints, 1);

%% 상태 벡터 초기화
% 상태: [p_base(3); q_base(4); v_base(3); omega_base(3); theta(7); theta_dot(7)]
% 총 27 차원

state = zeros(27, N);
state(1:3, 1) = p_base_0;
state(4:7, 1) = q_base_0;
state(8:10, 1) = v_base_0;
state(11:13, 1) = omega_base_0;
state(14:20, 1) = theta_0;
state(21:27, 1) = theta_dot_0;

%% 토크 입력 저장
tau_history = zeros(n_joints, N);

%% 시뮬레이션 루프 (RK4)
fprintf('시뮬레이션 시작...\n');
tic;

for k = 1:N-1
    % 현재 상태 추출
    x = state(:, k);
    t_k = t(k);
    
    % 토크 입력: sin(t)
    tau_joint = -0.1*sin(t_k) * ones(n_joints, 1);
    tau_history(:, k) = tau_joint;
    
    % RK4 적분
    k1 = dynamics(x, tau_joint, params);
    k2 = dynamics(x + 0.5*dt*k1, tau_joint, params);
    k3 = dynamics(x + 0.5*dt*k2, tau_joint, params);
    k4 = dynamics(x + dt*k3, tau_joint, params);
    
    state(:, k+1) = x + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);
    
    % Quaternion 정규화
    q = state(4:7, k+1);
    state(4:7, k+1) = q / norm(q);
    
    % 진행률 표시
    if mod(k, 1000) == 0
        fprintf('  t = %.2f / %.2f sec\n', t_k, t_end);
    end
end

tau_history(:, N) = 0.1*sin(t(N)) * ones(n_joints, 1);
sim_time = toc;
fprintf('시뮬레이션 완료! (%.2f초)\n', sim_time);

%% 결과 추출
p_base = state(1:3, :);
q_base = state(4:7, :);
v_base = state(8:10, :);
omega_base = state(11:13, :);
theta = state(14:20, :);
theta_dot = state(21:27, :);

%% 결과 저장 (Simscape 비교용)
save('sim_result_ours.mat', 't', 'p_base', 'q_base', 'v_base', 'omega_base', ...
     'theta', 'theta_dot', 'tau_history', 'dt', 't_end');
fprintf('결과 저장: sim_result_ours.mat\n');

%% 플롯
figure('Name', 'Space Manipulator Simulation', 'Position', [100 100 1200 800]);

% Joint angles
subplot(2,3,1);
plot(t, rad2deg(theta));
xlabel('Time [s]'); ylabel('Joint Angle [deg]');
title('Joint Angles');
legend('J1','J2','J3','J4','J5','J6','J7', 'Location', 'best');
grid on;

% Joint velocities
subplot(2,3,2);
plot(t, rad2deg(theta_dot));
xlabel('Time [s]'); ylabel('Joint Velocity [deg/s]');
title('Joint Velocities');
grid on;

% Base position
subplot(2,3,3);
plot(t, p_base);
xlabel('Time [s]'); ylabel('Position [m]');
title('Base Position');
legend('x','y','z');
grid on;

% Base velocity
subplot(2,3,4);
plot(t, v_base);
xlabel('Time [s]'); ylabel('Velocity [m/s]');
title('Base Linear Velocity');
legend('vx','vy','vz');
grid on;

% Base angular velocity
subplot(2,3,5);
plot(t, rad2deg(omega_base));
xlabel('Time [s]'); ylabel('Angular Velocity [deg/s]');
title('Base Angular Velocity');
legend('\omega_x','\omega_y','\omega_z');
grid on;

% Torque input
subplot(2,3,6);
plot(t, tau_history(1,:));
xlabel('Time [s]'); ylabel('Torque [Nm]');
title('Joint Torque Input (sin(t))');
grid on;

saveas(gcf, 'sim_result_plot.png');
fprintf('플롯 저장: sim_result_plot.png\n');

%% ========== 동역학 함수 ==========
function x_dot = dynamics(x, tau_joint, params)
    % 상태 추출
    p_base = x(1:3);
    q_base = x(4:7);
    v_base = x(8:10);
    omega_base = x(11:13);
    theta = x(14:20);
    theta_dot = x(21:27);
    
    n_joints = params.arm.n_joints;
    
    % FK 계산
    FK = RA_FK(params, p_base, q_base, theta);
    
    % Mass Matrix
    H = RA_MM(FK, params);
    
    % Coriolis/Centrifugal
    c = RA_C(FK, params, v_base, omega_base, theta_dot);
    
    % 일반화 힘
    % tau_gen = [f_base; tau_base; tau_joint]
    % Free-floating: f_base = 0, tau_base = 0
    tau_gen = [zeros(6,1); tau_joint];
    
    % 일반화 속도
    zeta = [v_base; omega_base; theta_dot];
    
    % 가속도 계산: H * zeta_dot + c = tau_gen
    % zeta_dot = H \ (tau_gen - c)
    zeta_dot = H \ (tau_gen - c);
    
    % 상태 미분
    x_dot = zeros(27, 1);
    
    % 위치 미분
    R_base = GetDCM_QUAT(q_base);
    x_dot(1:3) = v_base;  % p_base_dot = v_base (관성좌표계)
    
    % 쿼터니언 미분
    x_dot(4:7) = Derivative_Quat(q_base, omega_base);
    
    % 속도 미분
    x_dot(8:10) = zeta_dot(1:3);    % v_base_dot
    x_dot(11:13) = zeta_dot(4:6);   % omega_base_dot
    
    % 관절 미분
    x_dot(14:20) = theta_dot;        % theta_dot
    x_dot(21:27) = zeta_dot(7:13);   % theta_ddot
end