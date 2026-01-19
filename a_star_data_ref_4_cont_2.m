clc; clear all; close all;
addpath(genpath('D:\star_tracker_test\main_pj_code'));

%% 기본 상수
Mu = 3.98600441500000e+05;
R_e = 6.37813630000000e+03;
J2 = 0.001082627;

param.orbit.Mu = Mu;
param.orbit.R_e = R_e;
param.orbit.J2 = J2;

%% 큐브샛 사양 (6U)
cubesat.mass_kg = 12.0;
cubesat.thrust_mN = 75.0;
cubesat.max_accel_ms2 = (cubesat.thrust_mN/1000) / cubesat.mass_kg;
cubesat.max_accel_kms2 = cubesat.max_accel_ms2 / 1000;

fprintf('=== 큐브샛 사양 (6U) ===\n');
fprintf('질량: %.1f kg\n', cubesat.mass_kg);
fprintf('추력: %.1f mN\n', cubesat.thrust_mN);
fprintf('최대 가속도: %.3f mm/s² (%.1f μm/s²)\n', ...
    cubesat.max_accel_ms2*1000, cubesat.max_accel_ms2*1e6);

%% Chief A 궤도 요소
e_A = 0;
a_A = 6908.136;
h_A = sqrt(a_A*(1-e_A^2)*Mu);
i_A = deg2rad(97.45);
RAAN_A = deg2rad(270.8);
omega_A = deg2rad(90);
theta_A = deg2rad(0);

[r_A, v_A] = coe2rv([h_A, e_A, RAAN_A, i_A, omega_A, theta_A], Mu);
x_A = [r_A; v_A];

n = sqrt(Mu/a_A^3);
T_Op = 2*pi/n;

%% Chief C 초기화 (A로부터 in-track +10km)
fprintf('\n=== Chief C 설정 ===\n');

hB_I = cross(r_A, v_A);
i_I = r_A/norm(r_A);
k_I = hB_I/norm(hB_I);
j_I = cross(k_I, i_I);
R_L2I = [i_I, j_I, k_I];

r_C_inA = [0; 3; 0];  % A 기준 LVLH [km]
v_C_inA = [0; 0; 0];   % In-track formation

w_r_C = [-n*r_C_inA(2); n*r_C_inA(1); 0];
r_C = r_A + R_L2I*r_C_inA;
v_C = v_A + R_L2I*(v_C_inA + w_r_C);
x_C = [r_C; v_C];

fprintf('C 위치 (A 기준 LVLH): [%.3f, %.3f, %.3f] km\n', r_C_inA);

%% Deputy B 초기 상태 (A 기준 GCO)
fprintf('\n=== Deputy B 초기 상태 (A 기준) ===\n');

% In Track initial LVLH state
dist_formation = 1.2; %LVLH [km]

x_0_B = 0; % Initial x position LVLH [km]
x_0_dot_B = 0; % Initial x velocity LVLH [km/s]
y_0_B = 0 - dist_formation; % Initial y position LVLH [km]
y_0_dot_B = 0; % Initial y velocity LVLH [km/s]
z_0_B = 0; % Initial z position LVLH [km]
z_0_dot_B = 0; % Initial z velocity LVLH [km/s]

r_0_B = [x_0_B; y_0_B; z_0_B];
v_0_B = [x_0_dot_B; y_0_dot_B; z_0_dot_B];
x_B = [r_0_B; v_0_B];

% p_pi_B = deg2rad(0);
% r_init_B = 0.5;
% 
% x_0_B = r_init_B/2 * sin(p_pi_B);
% x_0_dot_B = r_init_B * n/2 * cos(p_pi_B);
% y_0_B = 2 * x_0_dot_B/n;
% y_0_dot_B = -2 * n * x_0_B;
% z_0_B = sqrt(3) * x_0_B;
% z_0_dot_B = sqrt(3) * x_0_dot_B;
% 
% r_0_B = [x_0_B; y_0_B; z_0_B];
% v_0_B = [x_0_dot_B; y_0_dot_B; z_0_dot_B];
% 
% fprintf('초기 LVLH 상태 (A 기준):\n');
% fprintf('  위치: [%.3f, %.3f, %.3f] km\n', r_0_B);
% fprintf('  속도: [%.6f, %.6f, %.6f] km/s\n', v_0_B);
% 
% w_r_B = [-n*r_0_B(2); n*r_0_B(1); 0];
% r_B = r_A + R_L2I*r_0_B;
% v_B = v_A + R_L2I*(v_0_B + w_r_B);
% x_B = [r_B; v_B];

%% 목표 Formation (C 기준)
x_target_wrt_C = [0; -0.5; 0; 0; 0; 0];  % C로부터 -0.5km in-track

fprintf('\n목표 LVLH 상태 (C 기준):\n');
fprintf('  위치: [%.3f, %.3f, %.3f] km\n', x_target_wrt_C(1:3));
fprintf('  속도: [%.6f, %.6f, %.6f] km/s\n', x_target_wrt_C(4:6));

%% LQR 제어기
fprintf('\n=== LQR 제어기 설계 ===\n');

A_cw = [zeros(3), eye(3);
        diag([3*n^2, 0, -n^2]), [0, 2*n, 0; -2*n, 0, 0; 0, 0, 0]];
B_cw = [zeros(3); eye(3)];

Q = 1e+1 * diag([1, 1, 1, 1, 1, 1]);
R = 1e+10 * diag([1, 1, 1]);

K = lqr(A_cw, B_cw, Q, R);

t_transition = 1/2 * T_Op;

fprintf('전환 시점: %.1f초 (%.2f 궤도)\n', t_transition, t_transition/T_Op);

%% 시뮬레이션 설정
dt = 0.2;
tt = 2.5*T_Op;
tspan = 0:dt:tt;

N_steps = length(tspan);

r_A_I = zeros(3, N_steps);
v_A_I = zeros(3, N_steps);
r_C_I = zeros(3, N_steps);
v_C_I = zeros(3, N_steps);
r_B_I = zeros(3, N_steps);
v_B_I = zeros(3, N_steps);
r_B_La = zeros(3, N_steps);  % A 기준
v_B_La = zeros(3, N_steps);
r_B_Lc = zeros(3, N_steps);  % C 기준
v_B_Lc = zeros(3, N_steps);
u_I_hist = zeros(3, N_steps);
u_L_hist = zeros(3, N_steps);
u_L_desired = zeros(3, N_steps);
thrust_mag = zeros(1, N_steps);
thrust_desired = zeros(1, N_steps);
saturated = false(1, N_steps);

r_A_I(:,1) = x_A(1:3);
v_A_I(:,1) = x_A(4:6);
r_C_I(:,1) = x_C(1:3);
v_C_I(:,1) = x_C(4:6);
r_B_I(:,1) = x_B(1:3);
v_B_I(:,1) = x_B(4:6);

% A 기준 LVLH 초기화
r_hat = r_A_I(:,1)/norm(r_A_I(:,1));
rcv = cross(r_A_I(:,1), v_A_I(:,1));
h_hat = rcv/norm(rcv);
t_hat = cross(h_hat, r_hat);
R_I2L = [r_hat, t_hat, h_hat]';

del_r = r_B_I(:,1) - r_A_I(:,1);
r_B_La(:,1) = R_I2L*del_r;
del_v = v_B_I(:,1) - v_A_I(:,1);
omega_0 = cross(r_A_I(:,1), v_A_I(:,1))/(norm(r_A_I(:,1))^2);
v_B_La(:,1) = R_I2L*(del_v - cross(omega_0, del_r));

% C 기준 LVLH 초기화
r_hat_C = r_C_I(:,1)/norm(r_C_I(:,1));
rcv_C = cross(r_C_I(:,1), v_C_I(:,1));
h_hat_C = rcv_C/norm(rcv_C);
t_hat_C = cross(h_hat_C, r_hat_C);
R_I2L_C = [r_hat_C, t_hat_C, h_hat_C]';

del_r_BC = r_B_I(:,1) - r_C_I(:,1);
r_B_Lc(:,1) = R_I2L_C*del_r_BC;
del_v_BC = v_B_I(:,1) - v_C_I(:,1);
omega_C = cross(r_C_I(:,1), v_C_I(:,1))/(norm(r_C_I(:,1))^2);
v_B_Lc(:,1) = R_I2L_C*(del_v_BC - cross(omega_C, del_r_BC));

%% 궤도 전파
fprintf('\n=== 궤도 전파 시작 ===\n');
orbit_dyn = @(x, u, params, ~) orbit_propagation_j2(0, x, u, [0;0;0], params);

for k = 1:N_steps-1
    t_current = tspan(k);
    
    x_A_current = [r_A_I(:,k); v_A_I(:,k)];
    x_C_current = [r_C_I(:,k); v_C_I(:,k)];
    x_B_current = [r_B_I(:,k); v_B_I(:,k)];
    
    % C 기준 LVLH 계산
    r_hat_C = r_C_I(:,k)/norm(r_C_I(:,k));
    rcv_C = cross(r_C_I(:,k), v_C_I(:,k));
    h_hat_C = rcv_C/norm(rcv_C);
    t_hat_C = cross(h_hat_C, r_hat_C);
    R_L2I_C_k = [r_hat_C, t_hat_C, h_hat_C];
    R_I2L_C_k = R_L2I_C_k';
    
    % 제어 계산 (전환 후 C 기준)
    if t_current >= t_transition
        x_rel_Lc = [r_B_Lc(:,k); v_B_Lc(:,k)];
        u_L_cmd = -K*(x_rel_Lc - x_target_wrt_C);
        
        u_L_desired(:,k) = u_L_cmd;
        thrust_desired(k) = norm(u_L_cmd);
        
        if norm(u_L_cmd) > cubesat.max_accel_kms2
            u_L = u_L_cmd / norm(u_L_cmd) * cubesat.max_accel_kms2;
            saturated(k) = true;
        else
            u_L = u_L_cmd;
        end
        
        u_I = R_L2I_C_k*u_L;
    else
        u_L = [0; 0; 0];
        u_I = [0; 0; 0];
        u_L_desired(:,k) = [0; 0; 0];
    end
    
    u_I_hist(:,k) = u_I;
    u_L_hist(:,k) = u_L;
    thrust_mag(k) = norm(u_I);
    
    % 궤도 전파
    x_A_next = rk4(orbit_dyn, x_A_current, [0;0;0], param, dt);
    x_C_next = rk4(orbit_dyn, x_C_current, [0;0;0], param, dt);
    x_B_next = rk4(orbit_dyn, x_B_current, u_I, param, dt);
    
    r_A_I(:,k+1) = x_A_next(1:3);
    v_A_I(:,k+1) = x_A_next(4:6);
    r_C_I(:,k+1) = x_C_next(1:3);
    v_C_I(:,k+1) = x_C_next(4:6);
    r_B_I(:,k+1) = x_B_next(1:3);
    v_B_I(:,k+1) = x_B_next(4:6);
    
    % A 기준 LVLH 계산
    r_hat_next = r_A_I(:,k+1)/norm(r_A_I(:,k+1));
    rcv_next = cross(r_A_I(:,k+1), v_A_I(:,k+1));
    h_hat_next = rcv_next/norm(rcv_next);
    t_hat_next = cross(h_hat_next, r_hat_next);
    R_I2L_next = [r_hat_next, t_hat_next, h_hat_next]';
    
    del_r_next = r_B_I(:,k+1) - r_A_I(:,k+1);
    r_B_La(:,k+1) = R_I2L_next*del_r_next;
    del_v_next = v_B_I(:,k+1) - v_A_I(:,k+1);
    omega_next = cross(r_A_I(:,k+1), v_A_I(:,k+1))/(norm(r_A_I(:,k+1))^2);
    v_B_La(:,k+1) = R_I2L_next*(del_v_next - cross(omega_next, del_r_next));
    
    % C 기준 LVLH 계산
    r_hat_C_next = r_C_I(:,k+1)/norm(r_C_I(:,k+1));
    rcv_C_next = cross(r_C_I(:,k+1), v_C_I(:,k+1));
    h_hat_C_next = rcv_C_next/norm(rcv_C_next);
    t_hat_C_next = cross(h_hat_C_next, r_hat_C_next);
    R_I2L_C_next = [r_hat_C_next, t_hat_C_next, h_hat_C_next]';
    
    del_r_BC_next = r_B_I(:,k+1) - r_C_I(:,k+1);
    r_B_Lc(:,k+1) = R_I2L_C_next*del_r_BC_next;
    del_v_BC_next = v_B_I(:,k+1) - v_C_I(:,k+1);
    omega_C_next = cross(r_C_I(:,k+1), v_C_I(:,k+1))/(norm(r_C_I(:,k+1))^2);
    v_B_Lc(:,k+1) = R_I2L_C_next*(del_v_BC_next - cross(omega_C_next, del_r_BC_next));
    
    if mod(k, 1000) == 0
        fprintf('  진행: %d/%d (%.1f%%)\n', k, N_steps, k/N_steps*100);
    end
end

fprintf('궤도 전파 완료!\n');

%% 결과 분석
time_min = tspan'/60;
idx_trans = find(tspan >= t_transition, 1);
target_dist = vecnorm(r_B_Lc - x_target_wrt_C(1:3), 2, 1);

sat_count = sum(saturated(idx_trans:end));
sat_ratio = sat_count / (N_steps - idx_trans) * 100;

fprintf('\n=== 제어 성능 (C 기준) ===\n');
fprintf('최종 위치 오차: %.2f m\n', target_dist(end)*1000);
fprintf('최종 속도 오차: %.2f mm/s\n', norm(v_B_Lc(:,end) - x_target_wrt_C(4:6))*1e6);
fprintf('평균 제어력 (전환 후): %.1f μm/s²\n', mean(thrust_mag(idx_trans:end))*1e6);
fprintf('최대 제어력 요구: %.1f μm/s²\n', max(thrust_desired(idx_trans:end))*1e6);
fprintf('최대 제어력 실제: %.1f μm/s²\n', max(thrust_mag(idx_trans:end))*1e6);
fprintf('포화 발생률: %.1f%% (%d/%d steps)\n', sat_ratio, sat_count, N_steps-idx_trans);

%% 플롯팅
figure('Position', [100, 100, 1600, 1000]);

% 3D 궤적 (C 기준)
subplot(2,3,1);
plot3(r_B_Lc(1,:), r_B_Lc(2,:), r_B_Lc(3,:), 'b-', 'LineWidth', 1.5);
hold on;
plot3(0, 0, 0, 'r+', 'MarkerSize', 15, 'LineWidth', 2);  % Chief C
plot3(x_target_wrt_C(1), x_target_wrt_C(2), x_target_wrt_C(3), 'ro', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
plot3(r_B_Lc(1,idx_trans), r_B_Lc(2,idx_trans), r_B_Lc(3,idx_trans), ...
      'gs', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
% A의 C 기준 위치
r_A_Lc_sample = R_I2L_C * (r_A_I(:,1) - r_C_I(:,1));
plot3(r_A_Lc_sample(1), r_A_Lc_sample(2), r_A_Lc_sample(3), 'k^', 'MarkerSize', 12, 'MarkerFaceColor', 'k');
grid on; axis equal;
xlabel('Radial [km]'); ylabel('In-track [km]'); zlabel('Cross-track [km]');
title('3D Formation Trajectory (C frame)');
legend('B Traj', 'Chief C', 'Target', 'Transition', 'Chief A', 'Location', 'best');
view(45, 25);

subplot(2,3,2);
plot(time_min, r_B_Lc(1,:)*1000, 'LineWidth', 1.5); hold on;
plot(time_min, r_B_Lc(2,:)*1000, 'LineWidth', 1.5);
plot(time_min, r_B_Lc(3,:)*1000, 'LineWidth', 1.5);
xline(t_transition/60, 'k--', 'LineWidth', 2);
grid on;
xlabel('Time [min]'); ylabel('Position [m]');
title('Position (C frame LVLH)');
legend('Radial', 'In-track', 'Cross-track', 'Transition');

subplot(2,3,3);
plot(time_min, v_B_Lc(1,:)*1000, 'LineWidth', 1.5); hold on;
plot(time_min, v_B_Lc(2,:)*1000, 'LineWidth', 1.5);
plot(time_min, v_B_Lc(3,:)*1000, 'LineWidth', 1.5);
xline(t_transition/60, 'k--', 'LineWidth', 2);
grid on;
xlabel('Time [min]'); ylabel('Velocity [m/s]');
title('Velocity (C frame LVLH)');
legend('Radial', 'In-track', 'Cross-track', 'Transition');

subplot(2,3,4);
plot(time_min, u_L_hist(1,:)*1e6, 'LineWidth', 1.5); hold on;
plot(time_min, u_L_hist(2,:)*1e6, 'LineWidth', 1.5);
plot(time_min, u_L_hist(3,:)*1e6, 'LineWidth', 1.5);
xline(t_transition/60, 'k--', 'LineWidth', 2);
grid on;
xlabel('Time [min]'); ylabel('Control [μm/s²]');
title('Control Input (C frame LVLH)');
legend('u_x', 'u_y', 'u_z', 'Transition');

subplot(2,3,5);
plot(time_min, thrust_desired*1e6, 'r--', 'LineWidth', 1); hold on;
plot(time_min, thrust_mag*1e6, 'b-', 'LineWidth', 1.5);
yline(cubesat.max_accel_ms2*1e6, 'k:', 'LineWidth', 2);
xline(t_transition/60, 'k--', 'LineWidth', 1.5);
if any(saturated)
    sat_idx = find(saturated);
    plot(time_min(sat_idx), thrust_mag(sat_idx)*1e6, 'r.', 'MarkerSize', 8);
end
grid on;
xlabel('Time [min]'); ylabel('Thrust [μm/s²]');
title(sprintf('Control Magnitude (Sat: %.1f%%)', sat_ratio));
legend('Desired', 'Actual', 'Limit', 'Location', 'best');

subplot(2,3,6);
plot(time_min, vecnorm(r_B_La,2,1)*1000, 'k-', 'LineWidth', 1.5); hold on;
plot(time_min, vecnorm(r_B_Lc,2,1)*1000, 'r-', 'LineWidth', 1.5);
plot(time_min, target_dist*1000, 'b--', 'LineWidth', 1.5);
xline(t_transition/60, 'k--', 'LineWidth', 2);
grid on;
xlabel('Time [min]'); ylabel('Distance [m]');
title('Relative Distance');
legend('From A', 'From C', 'From Target(C)', 'Transition');

sgtitle('3-Satellite Formation (A→C Transition)', 'FontSize', 14, 'FontWeight', 'bold');

%% 전환 후 추적 성능
figure('Position', [150, 150, 1400, 500]);

idx_after = tspan >= t_transition;
time_after = (tspan(idx_after) - t_transition)/60;
error_pos = r_B_Lc(:,idx_after) - x_target_wrt_C(1:3);
error_vel = v_B_Lc(:,idx_after) - x_target_wrt_C(4:6);

subplot(1,3,1);
plot(time_after, error_pos(1,:)*1000, 'LineWidth', 1.5); hold on;
plot(time_after, error_pos(2,:)*1000, 'LineWidth', 1.5);
plot(time_after, error_pos(3,:)*1000, 'LineWidth', 1.5);
plot(time_after, vecnorm(error_pos,2,1)*1000, 'k--', 'LineWidth', 2);
grid on;
xlabel('Time after transition [min]'); ylabel('Error [m]');
title('Position Tracking Error (C frame)');
legend('Radial', 'In-track', 'Cross-track', 'Total');

subplot(1,3,2);
plot(time_after, error_vel(1,:)*1000, 'LineWidth', 1.5); hold on;
plot(time_after, error_vel(2,:)*1000, 'LineWidth', 1.5);
plot(time_after, error_vel(3,:)*1000, 'LineWidth', 1.5);
plot(time_after, vecnorm(error_vel,2,1)*1000, 'k--', 'LineWidth', 2);
grid on;
xlabel('Time after transition [min]'); ylabel('Error [m/s]');
title('Velocity Tracking Error (C frame)');
legend('Radial', 'In-track', 'Cross-track', 'Total');

subplot(1,3,3);
plot(time_after, thrust_desired(idx_after)*1e6, 'r--', 'LineWidth', 1); hold on;
plot(time_after, thrust_mag(idx_after)*1e6, 'b-', 'LineWidth', 1.5);
yline(cubesat.max_accel_ms2*1e6, 'k:', 'LineWidth', 2);
sat_after = saturated(idx_after);
if any(sat_after)
    sat_idx_local = find(sat_after);
    plot(time_after(sat_idx_local), thrust_mag(find(idx_after,1)+sat_idx_local-1)*1e6, ...
         'r.', 'MarkerSize', 8);
end
grid on;
xlabel('Time after transition [min]'); ylabel('Thrust [μm/s²]');
title('Control Effort');
legend('Desired', 'Actual', 'Limit', 'Location', 'best');

sgtitle('Tracking Performance (After A→C Transition)', 'FontSize', 14, 'FontWeight', 'bold');

fprintf('\n=== 완료 ===\n');