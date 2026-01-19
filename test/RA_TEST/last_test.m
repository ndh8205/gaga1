%% compare_three_way_v2.m
% 3자 비교: current (rk4_ff) vs ssh (SMS_7Link) vs Simscape
% 
% =========================================================================
% 비교 대상:
%   1. current: rk4_ff (현재 구현된 코드)
%   2. ssh: SMS_7Link 로직 (Euler 기반)
%   3. Simscape: 참조 데이터
% =========================================================================

clear; clc; close all;
addpath(genpath('D:\pj2025\space_challenge'));

r2d = 180/pi;
d2r = 1/r2d;

%% ========== 파라미터 초기화 ==========
urdf_path = 'D:\pj2025\space_challenge\model\modeling_3d\ASM_KARI_ARM\ASM_KARI_ARM_URDF.urdf';
params = params_init_old(urdf_path);

%% ========== ssh용 robot 구조체 (SMS_7Link 방식) ==========
global robot
robot2 = importrobot('ASM_KARI_ARM_URDF\urdf\ASM_KARI_ARM_URDF.urdf');
for i = 1:7
    k(:,i) = robot2.Bodies{i+1}.Joint.JointAxis';
end
robot.k = k;
d = [0 0 0.5]';
for i = 2:9
    d(:,i) = robot2.Bodies{i}.Joint.JointToParentTransform(1:3,4);
end
robot.d = d;
CoM = [0 0 0]';
for i = 2:9
    CoM(:,i) = robot2.Bodies{i-1}.CenterOfMass';
end
robot.CoM = CoM;

m = [500, robot2.Bodies{1}.Mass, robot2.Bodies{2}.Mass, robot2.Bodies{3}.Mass, robot2.Bodies{4}.Mass, robot2.Bodies{5}.Mass, robot2.Bodies{6}.Mass, robot2.Bodies{7}.Mass, robot2.Bodies{8}.Mass]';
robot.m = m;
I(:,:,1) = [260 -0.2 0.6; -0.2 280 4; 0.6 4 170];
ixx = 0.00751250; ixy = -0.00000006; ixz = 0.00000167;
iyy = 0.00752435; iyz = -0.00000111; izz = 0.00631252;
I(:,:,2) = [ixx, ixy, ixz; ixy, iyy, iyz; ixz, iyz, izz];
ixx = 0.02665777; ixy = 0.00012951; ixz = -0.00002113;
iyy = 0.00821363; iyz = -0.00194208; izz = 0.02512228;
I(:,:,3) = [ixx, ixy, ixz; ixy, iyy, iyz; ixz, iyz, izz];
ixx = 0.05097328; ixy = 0.00000002; ixz = 0.00000000;
iyy = 0.03629437; iyz = 0.01622322; izz = 0.02722892;
I(:,:,4) = [ixx, ixy, ixz; ixy, iyy, iyz; ixz, iyz, izz];
ixx = 0.29888290; ixy = -0.00000478; ixz = 0.00033497;
iyy = 0.29796389; iyz = -0.00781668; izz = 0.01207673;
I(:,:,5) = [ixx, ixy, ixz; ixy, iyy, iyz; ixz, iyz, izz];
ixx = 0.05097328; ixy = 0.00000002; ixz = 0.00000000;
iyy = 0.03629437; iyz = -0.01622322; izz = 0.02722892;
I(:,:,6) = [ixx, ixy, ixz; ixy, iyy, iyz; ixz, iyz, izz];
ixx = 0.05207280; ixy = -0.00000483; ixz = 0.00009635;
iyy = 0.05170972; iyz = -0.00247387; izz = 0.00805695;
I(:,:,7) = [ixx, ixy, ixz; ixy, iyy, iyz; ixz, iyz, izz];
ixx = 0.05197714; ixy = -0.00005372; ixz = -0.00015573;
iyy = 0.03740379; iyz = 0.01165841; izz = 0.02571022;
I(:,:,8) = [ixx, ixy, ixz; ixy, iyy, iyz; ixz, iyz, izz];
ixx = 0.00768334; ixy = 0.00000000; ixz = 0.00000006;
iyy = 0.00768346; iyz = 0.00000000; izz = 0.00607932;
I(:,:,9) = [ixx, ixy, ixz; ixy, iyy, iyz; ixz, iyz, izz];
robot.I = I;

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

fprintf('=== 3자 비교: current vs ssh vs Simscape ===\n');
fprintf('dt = %.4f sec, T = %.1f sec\n\n', dt, T_max);

%% ========== 1. current 시뮬레이션 (rk4_ff) ==========
fprintf('1. current (rk4_ff) 시뮬레이션...\n');

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

%% ========== 2. ssh 시뮬레이션 (SMS_7Link 로직) ==========
fprintf('2. ssh (SMS_7Link) 시뮬레이션...\n');

x0_ssh = zeros(26,1);
data_ssh = zeros(26, N_custom);
data_ssh(:,1) = x0_ssh;

tic;
for i = 1:N_custom-1
    t = (i-1)*dt;
    u = [0 0 0 0 0 0 0.1*sin(t) 0 0 0 0 0 0]';
    x0_ssh = rk4_ssh(@RNEA_freefloating, x0_ssh, u, dt);
    data_ssh(:,i+1) = x0_ssh;
end
t_ssh = toc;
fprintf('   완료 (%.2f sec)\n', t_ssh);

% ssh 결과 추출
theta_hist_ssh = data_ssh(7:13,:);
r_hist_ssh = data_ssh(1:3,:);
euler_hist_ssh = data_ssh(4:6,:);

%% ========== Interpolation ==========
theta_interp_cur = interp1(t_custom, theta_hist_cur', time_sim)';
r_interp_cur = interp1(t_custom, r_hist_cur', time_sim)';
euler_interp_cur = interp1(t_custom, euler_hist_cur', time_sim)';

theta_interp_ssh = interp1(t_custom, theta_hist_ssh', time_sim)';
r_interp_ssh = interp1(t_custom, r_hist_ssh', time_sim)';
euler_interp_ssh = interp1(t_custom, euler_hist_ssh', time_sim)';

%% ========== 오차 계산 ==========
theta_err_cur = theta_interp_cur - theta_sim;
theta_err_ssh = theta_interp_ssh - theta_sim;

r_err_cur = r_interp_cur - r_sim;
r_err_ssh = r_interp_ssh - r_sim;

euler_err_cur = euler_interp_cur - euler_sim;
euler_err_ssh = euler_interp_ssh - euler_sim;

%% ========== 결과 출력 ==========
fprintf('\n');
fprintf('============================================================\n');
fprintf('                     최대 오차 비교\n');
fprintf('============================================================\n');

fprintf('\n--- 관절각 오차 [deg] ---\n');
fprintf('Joint |  current  |    ssh    \n');
fprintf('------+-----------+-----------\n');
for i = 1:7
    fprintf('  J%d  | %9.6f | %9.6f\n', i, ...
        rad2deg(max(abs(theta_err_cur(i,:)))), ...
        rad2deg(max(abs(theta_err_ssh(i,:)))));
end

fprintf('\n--- 위성 위치 오차 [mm] ---\n');
fprintf(' Axis |  current  |    ssh    \n');
fprintf('------+-----------+-----------\n');
fprintf('  x   | %9.6f | %9.6f\n', max(abs(r_err_cur(1,:)))*1000, max(abs(r_err_ssh(1,:)))*1000);
fprintf('  y   | %9.6f | %9.6f\n', max(abs(r_err_cur(2,:)))*1000, max(abs(r_err_ssh(2,:)))*1000);
fprintf('  z   | %9.6f | %9.6f\n', max(abs(r_err_cur(3,:)))*1000, max(abs(r_err_ssh(3,:)))*1000);

fprintf('\n--- Euler Angle 오차 [deg] ---\n');
fprintf(' Axis  |  current  |    ssh    \n');
fprintf('-------+-----------+-----------\n');
fprintf(' Roll  | %9.6f | %9.6f\n', rad2deg(max(abs(euler_err_cur(1,:)))), rad2deg(max(abs(euler_err_ssh(1,:)))));
fprintf(' Pitch | %9.6f | %9.6f\n', rad2deg(max(abs(euler_err_cur(2,:)))), rad2deg(max(abs(euler_err_ssh(2,:)))));
fprintf(' Yaw   | %9.6f | %9.6f\n', rad2deg(max(abs(euler_err_cur(3,:)))), rad2deg(max(abs(euler_err_ssh(3,:)))));

fprintf('\n--- 실행 시간 [sec] ---\n');
fprintf(' current: %.2f, ssh: %.2f\n', t_cur, t_ssh);

%% ========== 플롯 1: 관절각 비교 ==========
figure('Name', '관절각 비교', 'Position', [50, 50, 1400, 800]);

for i = 1:7
    subplot(2,4,i);
    plot(time_sim, rad2deg(theta_interp_cur(i,:)), 'b-', 'LineWidth', 1.2);
    hold on;
    plot(time_sim, rad2deg(theta_interp_ssh(i,:)), 'g-', 'LineWidth', 1.2);
    plot(time_sim, rad2deg(theta_sim(i,:)), 'r--', 'LineWidth', 1.2);
    xlabel('Time [s]'); ylabel('Angle [deg]');
    title(sprintf('Joint %d', i));
    if i == 1
        legend('current', 'ssh', 'Simscape', 'Location', 'best');
    end
    grid on;
end

subplot(2,4,8);
bar(1:7, [rad2deg(max(abs(theta_err_cur), [], 2)), rad2deg(max(abs(theta_err_ssh), [], 2))]);
xlabel('Joint'); ylabel('Max Error [deg]');
title('관절각 최대 오차');
legend('current', 'ssh', 'Location', 'best');
grid on;

sgtitle('관절각 비교: current vs ssh vs Simscape');

%% ========== 플롯 2: 위성 위치 ==========
figure('Name', '위성 위치 비교', 'Position', [100, 100, 1400, 500]);

subplot(1,3,1);
plot(time_sim, r_interp_cur'*1000, '-', 'LineWidth', 1.2);
hold on;
plot(time_sim, r_sim'*1000, '--', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('Position [mm]');
title('current vs Simscape');
legend('x(cur)','y(cur)','z(cur)','x(S)','y(S)','z(S)', 'Location', 'best');
grid on;

subplot(1,3,2);
plot(time_sim, r_interp_ssh'*1000, '-', 'LineWidth', 1.2);
hold on;
plot(time_sim, r_sim'*1000, '--', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('Position [mm]');
title('ssh vs Simscape');
legend('x(ssh)','y(ssh)','z(ssh)','x(S)','y(S)','z(S)', 'Location', 'best');
grid on;

subplot(1,3,3);
plot(time_sim, r_err_cur'*1000, '-', 'LineWidth', 1.2);
hold on;
plot(time_sim, r_err_ssh'*1000, '--', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('Error [mm]');
title('위치 오차 (실선:current, 점선:ssh)');
legend('x','y','z', 'Location', 'best');
grid on;

sgtitle('위성 위치 비교');

%% ========== 플롯 3: Euler 비교 ==========
figure('Name', 'Euler 비교', 'Position', [150, 150, 1400, 500]);

subplot(1,3,1);
plot(time_sim, rad2deg(euler_interp_cur'), '-', 'LineWidth', 1.2);
hold on;
plot(time_sim, rad2deg(euler_sim'), '--', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('Angle [deg]');
title('current vs Simscape');
legend('R(cur)','P(cur)','Y(cur)','R(S)','P(S)','Y(S)', 'Location', 'best');
grid on;

subplot(1,3,2);
plot(time_sim, rad2deg(euler_interp_ssh'), '-', 'LineWidth', 1.2);
hold on;
plot(time_sim, rad2deg(euler_sim'), '--', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('Angle [deg]');
title('ssh vs Simscape');
legend('R(ssh)','P(ssh)','Y(ssh)','R(S)','P(S)','Y(S)', 'Location', 'best');
grid on;

subplot(1,3,3);
plot(time_sim, rad2deg(euler_err_cur'), '-', 'LineWidth', 1.2);
hold on;
plot(time_sim, rad2deg(euler_err_ssh'), '--', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('Error [deg]');
title('Euler 오차 (실선:current, 점선:ssh)');
legend('Roll','Pitch','Yaw', 'Location', 'best');
grid on;

sgtitle('Euler Angle 비교');

%% ========== t=5초 상세 출력 ==========
fprintf('\n============================================================\n');
fprintf('                    t=5초 상세 비교\n');
fprintf('============================================================\n');

fprintf('\n--- 관절각 [deg] ---\n');
fprintf('Joint |  current  |    ssh    | Simscape  \n');
fprintf('------+-----------+-----------+-----------\n');
for i = 1:7
    fprintf('  J%d  | %9.4f | %9.4f | %9.4f\n', i, ...
        rad2deg(theta_interp_cur(i,end)), ...
        rad2deg(theta_interp_ssh(i,end)), ...
        rad2deg(theta_sim(i,end)));
end

fprintf('\n--- 위성 위치 [mm] ---\n');
fprintf(' Axis |  current  |    ssh    | Simscape  \n');
fprintf('------+-----------+-----------+-----------\n');
fprintf('  x   | %9.4f | %9.4f | %9.4f\n', r_interp_cur(1,end)*1000, r_interp_ssh(1,end)*1000, r_sim(1,end)*1000);
fprintf('  y   | %9.4f | %9.4f | %9.4f\n', r_interp_cur(2,end)*1000, r_interp_ssh(2,end)*1000, r_sim(2,end)*1000);
fprintf('  z   | %9.4f | %9.4f | %9.4f\n', r_interp_cur(3,end)*1000, r_interp_ssh(3,end)*1000, r_sim(3,end)*1000);

fprintf('\n--- Euler [deg] ---\n');
fprintf('      |  current  |    ssh    | Simscape  \n');
fprintf('------+-----------+-----------+-----------\n');
fprintf(' Roll | %9.4f | %9.4f | %9.4f\n', rad2deg(euler_interp_cur(1,end)), rad2deg(euler_interp_ssh(1,end)), rad2deg(euler_sim(1,end)));
fprintf(' Pitch| %9.4f | %9.4f | %9.4f\n', rad2deg(euler_interp_cur(2,end)), rad2deg(euler_interp_ssh(2,end)), rad2deg(euler_sim(2,end)));
fprintf(' Yaw  | %9.4f | %9.4f | %9.4f\n', rad2deg(euler_interp_cur(3,end)), rad2deg(euler_interp_ssh(3,end)), rad2deg(euler_sim(3,end)));

fprintf('\n============================================================\n');


%% ========== SMS_7Link용 RK4 ==========
function x_new = rk4_ssh(f, x, u, dt)
    k1 = f(x, u);
    k2 = f(x + 0.5*dt*k1, u);
    k3 = f(x + 0.5*dt*k2, u);
    k4 = f(x + dt*k3, u);
    x_new = x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
end

%% ========== SMS_7Link 함수들 ==========

function xdot = RNEA_freefloating(x0, u)
global robot;
[T, CoM_0_to_i, k, Joint_pos, R] = fn_forward_Kinematics(robot, x0);

m = robot.m;
I = robot.I;

qd = x0(20:26);
w = zeros(3, 10);
wd = zeros(3, 10);
vd = zeros(3, 10);
vc = zeros(3, 9);
F = zeros(3, 9); N = zeros(3, 9);
tau = zeros(7,1);

N_matrix = [1, 0,           -sin(x0(5))
            0, cos(x0(4)),  sin(x0(4))*cos(x0(5))
            0, -sin(x0(4)), cos(x0(4))*cos(x0(5))];

w(:,2) = w(:,1) + x0(17:19);
wd(:,2) = wd(:,1) + cross(w(:,1), x0(17:19));

p_i = x0(1:3);
p_ip = Joint_pos(:,1);
p_ic = CoM_0_to_i(:,1);

vd(:,2) = vd(:,1) + cross(wd(:,2), p_ip - p_i) + cross(w(:,2), cross(w(:,2), p_ip - p_i));
vc(:,1) = vd(:,2) + cross(wd(:,2), p_ic - p_ip) + cross(w(:,2), cross(w(:,2), p_ic - p_ip));
F(:,1) = m(1) * vc(:,1);
N(:,1) = R(:,:,1)*I(:,:,1)*R(:,:,1)' * wd(:,2) + cross(w(:,2), R(:,:,1)*I(:,:,1)*R(:,:,1)' * w(:,2));

p_ci(:,1) = p_ic - p_ip;
p_ii(:,1) = p_ip - p_i;

w(:,3) = w(:,2);
wd(:,3) = wd(:,2);

p_i = Joint_pos(:,1);
p_ip = Joint_pos(:,2);
p_ic = CoM_0_to_i(:,2);

vd(:,3) = vd(:,2) + cross(wd(:,3), p_ip - p_i) + cross(w(:,3), cross(w(:,3), p_ip - p_i));
vc(:,2) = vd(:,3) + cross(wd(:,3), p_ic - p_ip) + cross(w(:,3), cross(w(:,3), p_ic - p_ip));
F(:,2) = m(2) * vc(:,2);
N(:,2) = R(:,:,2)*I(:,:,2)*R(:,:,2)' * wd(:,3) + cross(w(:,3), R(:,:,2)*I(:,:,2)*R(:,:,2)' * w(:,3));

p_ci(:,2) = p_ic - p_ip;
p_ii(:,2) = p_ip - p_i;

for i = 3:9
    p_i = Joint_pos(:,i-1);
    p_ip = Joint_pos(:,i);
    p_ic = CoM_0_to_i(:,i);

    w(:,i+1) = w(:,i) + qd(i-2)*k(:,i-2);
    wd(:,i+1) = wd(:,i) + cross(w(:,i), qd(i-2)*k(:,i-2));
    
    vd(:,i+1) = vd(:,i) + cross(wd(:,i+1), p_ip - p_i) + cross(w(:,i+1), cross(w(:,i+1), p_ip - p_i));
    vc(:,i) = vd(:,i+1) + cross(wd(:,i+1), p_ic - p_ip) + cross(w(:,i+1), cross(w(:,i+1), p_ic - p_ip));
    F(:,i) = m(i) * vc(:,i);
    N(:,i) = R(:,:,i)*I(:,:,i)*R(:,:,i)' * wd(:,i+1) + cross(w(:,i+1), R(:,:,i)*I(:,:,i)*R(:,:,i)' * w(:,i+1));
    
    p_ci(:,i) = p_ic - p_ip;
    p_ii(:,i) = p_ip - p_i;
end

f_next = [0;0;0]; n_next = [0;0;0];

for i = 9:-1:3
    f = f_next + F(:,i);
    n = N(:,i) + n_next + cross(f_next, p_ci(:,i)) + cross(p_ci(:,i)+p_ii(:,i), f);
    tau(i-2,1) = n' * k(:,i-2);
    f_next = f;
    n_next = n;
end
f = f_next + F(:,2);
n = N(:,2) + n_next + cross(f_next, p_ci(:,2)) + cross(p_ci(:,2)+p_ii(:,2), f);
f_next = f;
n_next = n;

f = f_next + F(:,1);
n = N(:,1) + n_next + cross(f_next, p_ci(:,1)) + cross(p_ci(:,1)+p_ii(:,1), f);
f_next = f;
n_next = n;

f_base = f_next;
n_base = n_next;
C = [f_base; n_base; tau];

a = CoM_0_to_i(:,1)-x0(1:3);
b = Joint_pos(:,1)-CoM_0_to_i(:,1);
r_0i(:,1) = a;
for i = 2:9
    a(:,i) = CoM_0_to_i(:,i) - Joint_pos(:,i-1);
    b(:,i) = Joint_pos(:,i) - CoM_0_to_i(:,i);
    r_0i(:,i) = r_0i(:,i-1) + a(:,i) + b(:,i-1);
end

r_0g = r_0i * m/sum(m);
J_Ti = zeros(3,3,7);
for i = 1:7
    for j = 1:i
        J_Ti(:,j,i) = skew_fn(k(:,j))*(CoM_0_to_i(:,i+2)-Joint_pos(:,j+1));
    end
end

J_Ri = [k(:,1), zeros(3,6)];
for i = 2:7
    J_Ri(:,:,i) = J_Ri(:,:,i-1);
    J_Ri(:,i,i) = k(:,i);
end

M_total = sum(m);

H_w = zeros(3,3);
for i = 1:9
    H_w = H_w + m(i)*skew_fn(r_0i(:,i))'*skew_fn(r_0i(:,i)) + R(:,:,i)*I(:,:,i)*R(:,:,i)';
end

H_m = zeros(7,7);
for i = 1:7
    H_m = H_m + m(i+2)*J_Ti(:,:,i)'*J_Ti(:,:,i) + J_Ri(:,:,i)'*R(:,:,i+2)*I(:,:,i+2)*R(:,:,i+2)'*J_Ri(:,:,i);
end

H_vw = M_total*skew_fn(r_0g)';

H_wth = zeros(3,7);
for i = 1:7
    H_wth = H_wth + m(i+2)*skew_fn(r_0i(:,i+2))*J_Ti(:,:,i) + R(:,:,i+2)*I(:,:,i+2)*R(:,:,i+2)'*J_Ri(:,:,i);
end

J_Tw = zeros(3,7);
for i = 1:7
    J_Tw = J_Tw + m(i+2)*J_Ti(:,:,i);
end

H = [M_total*eye(3), H_vw, J_Tw
    H_vw', H_w, H_wth
    J_Tw', H_wth', H_m];

Xdot = H\(-C + u);
xdot = [x0(14:16); N_matrix\R(:,:,1)'*x0(17:19); x0(20:26); Xdot];
end

function [T, CoM_0_to_i, k, Joint_pos, R] = fn_forward_Kinematics(robot, x0)
    CoM = robot.CoM;
    d = robot.d;
    k = robot.k;
    thetas = x0(7:13);

    T  = [angle2dcm(x0(6),x0(5),x0(4))',x0(1:3);zeros(1,3), 1]*[eye(3), d(:,1); zeros(1,3),1];
    Joint_pos(:,1) = T(1:3,4);
    CoM_0_to_i(:,1) = x0(1:3);
    R(:,:,1) = T(1:3,1:3);

    A_CoM = [eye(3), CoM(:,2); zeros(1,3), 1];
    A = [eye(3), d(:,2); zeros(1,3), 1];
    CoM_i = T * A_CoM;
    T = T * A;
    Joint_pos(:,2) = T(1:3,4);
    CoM_0_to_i(:,2) = CoM_i(1:3,4);
    R(:,:,2) = T(1:3,1:3);

    for i = 1:7
        A   =   [angle2dcm(thetas(i)*k(3,i),thetas(i)*k(2,i),thetas(i)*k(1,i))',zeros(3,1);zeros(1,3),1] * [eye(3), d(:,i+2); zeros(1,3), 1];
        A_CoM = [angle2dcm(thetas(i)*k(3,i),thetas(i)*k(2,i),thetas(i)*k(1,i))',zeros(3,1);zeros(1,3),1] * [eye(3), CoM(:,i+2); zeros(1,3), 1];
        k_tmp = T(1:3,1:3)*k(:,i);
        CoM_i = T * A_CoM;
        T = T * A;
        R(:,:,i+2) = T(1:3,1:3);
        CoM_0_to_i(:,i+2) = CoM_i(1:3,4);
        Joint_pos(:,i+2) = T(1:3,4);
        k(:,i) = k_tmp/sqrt(k_tmp'*k_tmp);
    end
end

function A = skew_fn(a)
A = [0, -a(3), a(2)
     a(3), 0, -a(1)
     -a(2), a(1), 0];
end