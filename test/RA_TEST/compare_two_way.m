%% compare_two_way.m
% 2자 비교: current (rk4_ff) vs Simscape
% 
% =========================================================================
% 비교 대상:
%   1. current: rk4_ff (현재 구현된 코드)
%   2. Simscape: 참조 데이터
% =========================================================================

clear; clc; close all;
addpath(genpath('D:\pj2025\space_challenge'));

r2d = 180/pi;
d2r = 1/r2d;

%% ========== 파라미터 초기화 ==========
urdf_path = 'D:\pj2025\space_challenge\model\modeling_3d\ASM_KARI_ARM\ASM_KARI_ARM_URDF.urdf';
params = params_init_old(urdf_path);

%% ========== Simscape 데이터 로드 ==========
simscape_path = 'D:\pj2025\space_challenge\ref\simscape\';

Joint1_q = load([simscape_path, 'Joint1_q.mat']); Joint1_q = Joint1_q.ans;
Joint2_q = load([simscape_path, 'Joint2_q.mat']); Joint2_q = Joint2_q.ans;
Joint3_q = load([simscape_path, 'Joint3_q.mat']); Joint3_q = Joint3_q.ans;
Joint4_q = load([simscape_path, 'Joint4_q.mat']); Joint4_q = Joint4_q.ans;
Joint5_q = load([simscape_path, 'Joint5_q.mat']); Joint5_q = Joint5_q.ans;
Joint6_q = load([simscape_path, 'Joint6_q.mat']); Joint6_q = Joint6_q.ans;
Joint7_q = load([simscape_path, 'Joint7_q.mat']); Joint7_q = Joint7_q.ans;

Joint1_qd = load([simscape_path, 'Joint1_qd.mat']); Joint1_qd = Joint1_qd.ans;
Joint2_qd = load([simscape_path, 'Joint2_qd.mat']); Joint2_qd = Joint2_qd.ans;
Joint3_qd = load([simscape_path, 'Joint3_qd.mat']); Joint3_qd = Joint3_qd.ans;
Joint4_qd = load([simscape_path, 'Joint4_qd.mat']); Joint4_qd = Joint4_qd.ans;
Joint5_qd = load([simscape_path, 'Joint5_qd.mat']); Joint5_qd = Joint5_qd.ans;
Joint6_qd = load([simscape_path, 'Joint6_qd.mat']); Joint6_qd = Joint6_qd.ans;
Joint7_qd = load([simscape_path, 'Joint7_qd.mat']); Joint7_qd = Joint7_qd.ans;

base_x = load([simscape_path, 'base_x.mat']); base_x = base_x.ans;
base_y = load([simscape_path, 'base_y.mat']); base_y = base_y.ans;
base_z = load([simscape_path, 'base_z.mat']); base_z = base_z.ans;
base_roll = load([simscape_path, 'base_roll.mat']); base_roll = base_roll.ans;
base_pitch = load([simscape_path, 'base_pitch.mat']); base_pitch = base_pitch.ans;
base_yaw = load([simscape_path, 'base_yaw.mat']); base_yaw = base_yaw.ans;

tmp = load([simscape_path, 'base_quat.mat']); base_quat = tmp.ans;

%% ========== 시뮬레이션 설정 ==========
dt = 0.001;
T_max = 5;
t_custom = 0:dt:T_max;
N_custom = length(t_custom);

% Simscape 시간 구간
idx_end = find(Joint1_q.Time <= T_max, 1, 'last');
time_sim = Joint1_q.Time(1:idx_end);
N_sim = length(time_sim);

% Simscape 데이터 정리
theta_sim = zeros(7, N_sim);
theta_sim(1,:) = Joint1_q.Data(1:idx_end)';
theta_sim(2,:) = Joint2_q.Data(1:idx_end)';
theta_sim(3,:) = Joint3_q.Data(1:idx_end)';
theta_sim(4,:) = Joint4_q.Data(1:idx_end)';
theta_sim(5,:) = Joint5_q.Data(1:idx_end)';
theta_sim(6,:) = Joint6_q.Data(1:idx_end)';
theta_sim(7,:) = Joint7_q.Data(1:idx_end)';

r_sim = zeros(3, N_sim);
r_sim(1,:) = base_x.Data(1:idx_end)';
r_sim(2,:) = base_y.Data(1:idx_end)';
r_sim(3,:) = base_z.Data(1:idx_end)';

euler_sim = zeros(3, N_sim);
euler_sim(1,:) = base_roll.Data(1:idx_end)';
euler_sim(2,:) = base_pitch.Data(1:idx_end)';
euler_sim(3,:) = base_yaw.Data(1:idx_end)';

% Simscape quaternion
quat_sim_raw = base_quat.Data;
quat_sim_full = squeeze(quat_sim_raw)';
quat_sim = quat_sim_full(1:idx_end, :);
if abs(quat_sim(1,1) - 1) < 0.01
    quat_sim_reorder = quat_sim;
elseif abs(quat_sim(1,4) - 1) < 0.01
    quat_sim_reorder = [quat_sim(:,4), quat_sim(:,1:3)];
else
    quat_sim_reorder = quat_sim;
end

fprintf('=== 2자 비교: current vs Simscape ===\n');
fprintf('dt = %.4f sec, T = %.1f sec\n\n', dt, T_max);

%% ========== current 시뮬레이션 (rk4_ff) ==========
fprintf('current (rk4_ff) 시뮬레이션...\n');

p_cur = [0; 0; 0];
q_cur = [1; 0; 0; 0];
v_cur = [0; 0; 0];
omega_cur = [0; 0; 0];
theta_cur = zeros(7, 1);
theta_dot_cur = zeros(7, 1);

theta_hist_cur = zeros(7, N_custom);
r_hist_cur = zeros(3, N_custom);
quat_hist_cur = zeros(4, N_custom);
euler_hist_cur = zeros(3, N_custom);

tic;
for k = 1:N_custom
    t = t_custom(k);
    
    theta_hist_cur(:,k) = theta_cur;
    r_hist_cur(:,k) = p_cur;
    quat_hist_cur(:,k) = q_cur;
    euler_hist_cur(:,k) = Quat2Euler(q_cur);
    
    if k == N_custom, break; end
    
    tau_func = @(tt) [0.1*sin(tt); zeros(6,1)];
    [p_cur, q_cur, v_cur, omega_cur, theta_cur, theta_dot_cur] = ...
        rk4_ff(p_cur, q_cur, v_cur, omega_cur, theta_cur, theta_dot_cur, ...
               tau_func, t, dt, params);
end
t_cur = toc;
fprintf('   완료 (%.2f sec)\n', t_cur);

%% ========== Interpolation ==========
theta_interp_cur = interp1(t_custom, theta_hist_cur', time_sim)';
r_interp_cur = interp1(t_custom, r_hist_cur', time_sim)';
euler_interp_cur = interp1(t_custom, euler_hist_cur', time_sim)';

%% ========== 오차 계산 ==========
theta_err_cur = theta_interp_cur - theta_sim;
r_err_cur = r_interp_cur - r_sim;
euler_err_cur = euler_interp_cur - euler_sim;

%% ========== 결과 출력 ==========
fprintf('\n');
fprintf('============================================================\n');
fprintf('                     최대 오차 (current vs Simscape)\n');
fprintf('============================================================\n');

fprintf('\n--- 관절각 오차 [deg] ---\n');
fprintf('Joint |  Max Error  \n');
fprintf('------+-------------\n');
for i = 1:7
    fprintf('  J%d  | %11.6f\n', i, rad2deg(max(abs(theta_err_cur(i,:)))));
end

fprintf('\n--- 위성 위치 오차 [mm] ---\n');
fprintf(' Axis |  Max Error  \n');
fprintf('------+-------------\n');
fprintf('  x   | %11.6f\n', max(abs(r_err_cur(1,:)))*1000);
fprintf('  y   | %11.6f\n', max(abs(r_err_cur(2,:)))*1000);
fprintf('  z   | %11.6f\n', max(abs(r_err_cur(3,:)))*1000);

fprintf('\n--- Euler Angle 오차 [deg] ---\n');
fprintf(' Axis  |  Max Error  \n');
fprintf('-------+-------------\n');
fprintf(' Roll  | %11.6f\n', rad2deg(max(abs(euler_err_cur(1,:)))));
fprintf(' Pitch | %11.6f\n', rad2deg(max(abs(euler_err_cur(2,:)))));
fprintf(' Yaw   | %11.6f\n', rad2deg(max(abs(euler_err_cur(3,:)))));

fprintf('\n--- 실행 시간 [sec] ---\n');
fprintf(' current: %.2f\n', t_cur);

%% ========== 플롯 1: 관절각 비교 ==========
figure('Name', '관절각 비교', 'Position', [50, 50, 1400, 800]);

for i = 1:7
    subplot(2,4,i);
    plot(time_sim, rad2deg(theta_interp_cur(i,:)), 'b-', 'LineWidth', 1.5);
    hold on;
    plot(time_sim, rad2deg(theta_sim(i,:)), 'r--', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Angle [deg]');
    title(sprintf('Joint %d', i));
    if i == 1
        legend('current', 'Simscape', 'Location', 'best');
    end
    grid on;
end

subplot(2,4,8);
bar(1:7, rad2deg(max(abs(theta_err_cur), [], 2)));
xlabel('Joint'); ylabel('Max Error [deg]');
title('관절각 최대 오차');
grid on;

sgtitle('관절각 비교: current vs Simscape');

%% ========== 플롯 2: 위성 위치 ==========
figure('Name', '위성 위치 비교', 'Position', [100, 100, 1200, 400]);

subplot(1,2,1);
plot(time_sim, r_interp_cur'*1000, '-', 'LineWidth', 1.5);
hold on;
plot(time_sim, r_sim'*1000, '--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Position [mm]');
title('위성 위치');
legend('x(cur)','y(cur)','z(cur)','x(Sim)','y(Sim)','z(Sim)', 'Location', 'best');
grid on;

subplot(1,2,2);
plot(time_sim, r_err_cur'*1000, '-', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Error [mm]');
title('위치 오차');
legend('x','y','z', 'Location', 'best');
grid on;

sgtitle('위성 위치 비교: current vs Simscape');

%% ========== 플롯 3: Euler 비교 ==========
figure('Name', 'Euler 비교', 'Position', [150, 150, 1200, 400]);

subplot(1,2,1);
plot(time_sim, rad2deg(euler_interp_cur'), '-', 'LineWidth', 1.5);
hold on;
plot(time_sim, rad2deg(euler_sim'), '--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Angle [deg]');
title('Euler Angles');
legend('Roll(cur)','Pitch(cur)','Yaw(cur)','Roll(Sim)','Pitch(Sim)','Yaw(Sim)', 'Location', 'best');
grid on;

subplot(1,2,2);
plot(time_sim, rad2deg(euler_err_cur'), '-', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Error [deg]');
title('Euler 오차');
legend('Roll','Pitch','Yaw', 'Location', 'best');
grid on;

sgtitle('Euler Angle 비교: current vs Simscape');

%% ========== t=5초 상세 출력 ==========
fprintf('\n============================================================\n');
fprintf('                    t=5초 상세 비교\n');
fprintf('============================================================\n');

fprintf('\n--- 관절각 [deg] ---\n');
fprintf('Joint |  current  | Simscape  |   Error   \n');
fprintf('------+-----------+-----------+-----------\n');
for i = 1:7
    fprintf('  J%d  | %9.4f | %9.4f | %9.6f\n', i, ...
        rad2deg(theta_interp_cur(i,end)), ...
        rad2deg(theta_sim(i,end)), ...
        rad2deg(theta_err_cur(i,end)));
end

fprintf('\n--- 위성 위치 [mm] ---\n');
fprintf(' Axis |  current  | Simscape  |   Error   \n');
fprintf('------+-----------+-----------+-----------\n');
fprintf('  x   | %9.4f | %9.4f | %9.6f\n', r_interp_cur(1,end)*1000, r_sim(1,end)*1000, r_err_cur(1,end)*1000);
fprintf('  y   | %9.4f | %9.4f | %9.6f\n', r_interp_cur(2,end)*1000, r_sim(2,end)*1000, r_err_cur(2,end)*1000);
fprintf('  z   | %9.4f | %9.4f | %9.6f\n', r_interp_cur(3,end)*1000, r_sim(3,end)*1000, r_err_cur(3,end)*1000);

fprintf('\n--- Euler [deg] ---\n');
fprintf('      |  current  | Simscape  |   Error   \n');
fprintf('------+-----------+-----------+-----------\n');
fprintf(' Roll | %9.4f | %9.4f | %9.6f\n', rad2deg(euler_interp_cur(1,end)), rad2deg(euler_sim(1,end)), rad2deg(euler_err_cur(1,end)));
fprintf(' Pitch| %9.4f | %9.4f | %9.6f\n', rad2deg(euler_interp_cur(2,end)), rad2deg(euler_sim(2,end)), rad2deg(euler_err_cur(2,end)));
fprintf(' Yaw  | %9.4f | %9.4f | %9.6f\n', rad2deg(euler_interp_cur(3,end)), rad2deg(euler_sim(3,end)), rad2deg(euler_err_cur(3,end)));

fprintf('\n============================================================\n');