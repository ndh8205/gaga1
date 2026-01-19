%% compare_simscape.m - Simscape vs 우리 코드 비교
clear; clc; close all;
addpath(genpath('D:\hanul2026_scie\mppi_RA'));

%% 파라미터 초기화
urdf_path = 'D:\hanul2026_scie\mppi_RA\model\urdf\ASM_KARI_ARM_URDF.urdf';
params = params_init(urdf_path);

%% 시뮬레이션 설정
dt = 0.01;
t_end = 5.0;
t = 0:dt:t_end;
N = length(t);
n_joints = 7;

%% 초기 조건
p_base_0 = [0;0;0];
q_base_0 = [1;0;0;0];
v_base_0 = [0;0;0];
omega_base_0 = [0;0;0];
theta_0 = zeros(n_joints,1);
theta_dot_0 = zeros(n_joints,1);

%% 상태 초기화
state = zeros(27, N);
state(1:3,1) = p_base_0;
state(4:7,1) = q_base_0;
state(8:10,1) = v_base_0;
state(11:13,1) = omega_base_0;
state(14:20,1) = theta_0;
state(21:27,1) = theta_dot_0;

%% 시뮬레이션 (RK4)
fprintf('우리 코드 시뮬레이션...\n');
tic;
for k = 1:N-1
    x = state(:,k);
    tau_joint = sin(t(k)) * ones(n_joints,1);
    
    k1 = dynamics(x, tau_joint, params);
    k2 = dynamics(x + 0.5*dt*k1, tau_joint, params);
    k3 = dynamics(x + 0.5*dt*k2, tau_joint, params);
    k4 = dynamics(x + dt*k3, tau_joint, params);
    
    state(:,k+1) = x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
    state(4:7,k+1) = state(4:7,k+1) / norm(state(4:7,k+1));
end
fprintf('완료 (%.2f초)\n', toc);

%% 결과 추출
theta_ours = state(14:20,:);
theta_dot_ours = state(21:27,:);
p_base_ours = state(1:3,:);
omega_base_ours = state(11:13,:);

%% Simscape 결과 로드
fprintf('Simscape 결과 로드...\n');
simscape_path = 'D:\hanul2026_scie\mppi_RA\ref\simscape\';

% Joint angles (timeseries 형식)
for i = 1:7
    data = load([simscape_path, sprintf('Joint%d_q.mat', i)]);
    fn = fieldnames(data); 
    ts = data.(fn{1});
    theta_sim(i,:) = interp1(ts.Time, ts.Data, t, 'linear', 'extrap');
end

% Joint velocities
for i = 1:7
    data = load([simscape_path, sprintf('Joint%d_qd.mat', i)]);
    fn = fieldnames(data);
    ts = data.(fn{1});
    theta_dot_sim(i,:) = interp1(ts.Time, ts.Data, t, 'linear', 'extrap');
end

% Base position
base_x = load([simscape_path, 'base_x.mat']); fn=fieldnames(base_x); base_x=base_x.(fn{1});
base_y = load([simscape_path, 'base_y.mat']); fn=fieldnames(base_y); base_y=base_y.(fn{1});
base_z = load([simscape_path, 'base_z.mat']); fn=fieldnames(base_z); base_z=base_z.(fn{1});
p_base_sim(1,:) = interp1(base_x.Time, base_x.Data, t, 'linear', 'extrap');
p_base_sim(2,:) = interp1(base_y.Time, base_y.Data, t, 'linear', 'extrap');
p_base_sim(3,:) = interp1(base_z.Time, base_z.Data, t, 'linear', 'extrap');

% Base angular velocity
base_roll_av = load([simscape_path, 'base_roll_AV.mat']); fn=fieldnames(base_roll_av); base_roll_av=base_roll_av.(fn{1});
base_pitch_av = load([simscape_path, 'base_pitch_AV.mat']); fn=fieldnames(base_pitch_av); base_pitch_av=base_pitch_av.(fn{1});
base_yaw_av = load([simscape_path, 'base_yaw_AV.mat']); fn=fieldnames(base_yaw_av); base_yaw_av=base_yaw_av.(fn{1});
omega_base_sim(1,:) = interp1(base_roll_av.Time, base_roll_av.Data, t, 'linear', 'extrap');
omega_base_sim(2,:) = interp1(base_pitch_av.Time, base_pitch_av.Data, t, 'linear', 'extrap');
omega_base_sim(3,:) = interp1(base_yaw_av.Time, base_yaw_av.Data, t, 'linear', 'extrap');

%% 오차 계산
err_theta = theta_ours - theta_sim;
err_theta_dot = theta_dot_ours - theta_dot_sim;
err_p_base = p_base_ours - p_base_sim;
err_omega_base = omega_base_ours - omega_base_sim;

fprintf('\n=== 최종 오차 (t=%.1f초) ===\n', t_end);
fprintf('Joint angle 오차 [deg]:\n');
for i = 1:7
    fprintf('  J%d: %.4f\n', i, rad2deg(err_theta(i,end)));
end
fprintf('Joint velocity 오차 [deg/s]:\n');
for i = 1:7
    fprintf('  J%d: %.4f\n', i, rad2deg(err_theta_dot(i,end)));
end
fprintf('Base position 오차 [mm]: [%.4f, %.4f, %.4f]\n', 1000*err_p_base(:,end));
fprintf('Base angular velocity 오차 [deg/s]: [%.4f, %.4f, %.4f]\n', rad2deg(err_omega_base(:,end)));

%% 플롯
figure('Position', [100 100 1400 900]);

subplot(2,3,1);
plot(t, rad2deg(theta_ours(1,:)), 'b-', t, rad2deg(theta_sim(1,:)), 'r--');
xlabel('Time [s]'); ylabel('[deg]'); title('Joint 1 Angle');
legend('Ours','Simscape'); grid on;

subplot(2,3,2);
plot(t, rad2deg(theta_ours(2,:)), 'b-', t, rad2deg(theta_sim(2,:)), 'r--');
xlabel('Time [s]'); ylabel('[deg]'); title('Joint 2 Angle');
legend('Ours','Simscape'); grid on;

subplot(2,3,3);
plot(t, rad2deg(err_theta'));
xlabel('Time [s]'); ylabel('[deg]'); title('Joint Angle Errors');
legend('J1','J2','J3','J4','J5','J6','J7'); grid on;

subplot(2,3,4);
plot(t, 1000*p_base_ours(1,:), 'b-', t, 1000*p_base_sim(1,:), 'r--');
xlabel('Time [s]'); ylabel('[mm]'); title('Base X Position');
legend('Ours','Simscape'); grid on;

subplot(2,3,5);
plot(t, rad2deg(omega_base_ours(3,:)), 'b-', t, rad2deg(omega_base_sim(3,:)), 'r--');
xlabel('Time [s]'); ylabel('[deg/s]'); title('Base Yaw Rate');
legend('Ours','Simscape'); grid on;

subplot(2,3,6);
plot(t, 1000*vecnorm(err_p_base), 'b-', t, rad2deg(vecnorm(err_omega_base)), 'r-');
xlabel('Time [s]'); title('Error Norms');
legend('Position [mm]', 'Ang Vel [deg/s]'); grid on;

%% 동역학 함수
function x_dot = dynamics(x, tau_joint, params)
    p_base = x(1:3);
    q_base = x(4:7);
    v_base = x(8:10);
    omega_base = x(11:13);
    theta = x(14:20);
    theta_dot = x(21:27);
    
    FK = RA_FK(params, p_base, q_base, theta);
    H = RA_MM(FK, params);
    c = RA_C(FK, params, v_base, omega_base, theta_dot);
    
    tau_gen = [zeros(6,1); tau_joint];
    zeta = [v_base; omega_base; theta_dot];
    zeta_dot = H \ (tau_gen - c);
    
    x_dot = zeros(27,1);
    x_dot(1:3) = v_base;
    x_dot(4:7) = Derivative_Quat(q_base, omega_base);
    x_dot(8:10) = zeta_dot(1:3);
    x_dot(11:13) = zeta_dot(4:6);
    x_dot(14:20) = theta_dot;
    x_dot(21:27) = zeta_dot(7:13);
end