clc; clear all; 
% close all;
addpath(genpath('D:\Gaimsat_pj\matsim'));

%% 기본 상수
Mu = 3.98600441500000e+05;
R_e = 6.37813630000000e+03;
J2 = 0.001082627;

param.orbit.Mu = Mu;
param.orbit.R_e = R_e;
param.orbit.J2 = J2;

%% 큐브샛 사양 (6U)
cubesat.mass_kg = 10.5;
cubesat.thrust_mN = 300.0;
cubesat.max_accel_ms2 = (cubesat.thrust_mN/1000) / cubesat.mass_kg;
cubesat.max_accel_kms2 = cubesat.max_accel_ms2 / 1000;

fprintf('=== 큐브샛 사양 (6U) ===\n');
fprintf('질량: %.1f kg\n', cubesat.mass_kg);
fprintf('추력: %.1f mN\n', cubesat.thrust_mN);
fprintf('최대 가속도: %.3f mm/s² (%.1f μm/s²)\n', ...
    cubesat.max_accel_ms2*1000, cubesat.max_accel_ms2*1e6);

%% Chief [A] 궤도 요소
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

%% ========== FORMATION 설정 (사용자 입력) ==========
fprintf('\n=== FORMATION 설정 ===\n');
% 
% % % --- 초기 Formation (GCO) ---
% p_pi_B = deg2rad(0);
% r_init_B = 1.2;
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

% fprintf('초기 LVLH 상태:\n');
% fprintf('  위치: [%.3f, %.3f, %.3f] km\n', r_0_B);
% fprintf('  속도: [%.6f, %.6f, %.6f] km/s\n', v_0_B);

% In Track initial LVLH state
dist_formation = 1.2; %LVLH [km]

x_0_B = 0; % Initial x position LVLH [km]
x_0_dot_B = 0; % Initial x velocity LVLH [km/s]
y_0_B = 0 - dist_formation; % Initial y position LVLH [km]
y_0_dot_B = 0; % Initial y velocity LVLH [km/s]
z_0_B = 0; % Initial z position LVLH [km]
z_0_dot_B = 0; % Initial z velocity LVLH [km/s]

r_0_B = [ x_0_B; y_0_B; z_0_B ]; % Initial position vector LVLH [km]
v_0_B = [ x_0_dot_B; y_0_dot_B; z_0_dot_B ]; % Initial velocity vector LVLH [km/s]


fprintf('초기 LVLH 상태:\n');
fprintf('  위치: [%.3f, %.3f, %.3f] km\n', r_0_B);
fprintf('  속도: [%.6f, %.6f, %.6f] km/s\n', v_0_B);

% --- 목표 Formation (In-track) ---
x_target = [0; -0.15; 0; 0; 0; 0];  % [r_L; v_L]
% x_target = [r_0_B1; v_0_B1];  % [r_L; v_L]


fprintf('목표 LVLH 상태:\n');
fprintf('  위치: [%.3f, %.3f, %.3f] km\n', x_target(1:3));
fprintf('  속도: [%.6f, %.6f, %.6f] km/s\n', x_target(4:6));

%% LVLH → ECI
hB_I = cross(r_A, v_A);
i_I = r_A/norm(r_A);
k_I = hB_I/norm(hB_I);
j_I = cross(k_I, i_I);
R_L2I = [i_I, j_I, k_I];

w_r_B = [-n*r_0_B(2); n*r_0_B(1); 0];
r_B = r_A + R_L2I*r_0_B;
v_B = v_A + R_L2I*(v_0_B + w_r_B);
x_B = [r_B; v_B];

%% LQR 제어기
fprintf('\n=== LQR 제어기 설계 ===\n');

A_cw = [zeros(3), eye(3);
        diag([3*n^2, 0, -n^2]), [0, 2*n, 0; -2*n, 0, 0; 0, 0, 0]];
B_cw = [zeros(3); eye(3)];

Q = 1e+1 * diag([1, 1, 1, 1, 1, 1]);
R = 5e+12 * diag([1, 1, 1]);


K = lqr(A_cw, B_cw, Q, R);

t_transition = 1 * T_Op;

fprintf('전환 시점: %.1f초 (%.2f 궤도)\n', t_transition, t_transition/T_Op);

%% 시뮬레이션 설정

dt = 0.2;
tt = 3 * T_Op;
% tt = 60 * 60 * 24;

tspan = 0 : dt : tt;

N_steps = length(tspan);

r_A_I = zeros(3, N_steps);
v_A_I = zeros(3, N_steps);
r_B_I = zeros(3, N_steps);
v_B_I = zeros(3, N_steps);
r_B_L = zeros(3, N_steps);
v_B_L = zeros(3, N_steps);
u_I_hist = zeros(3, N_steps);
u_L_hist = zeros(3, N_steps);
u_L_desired = zeros(3, N_steps);
thrust_mag = zeros(1, N_steps);
thrust_desired = zeros(1, N_steps);
saturated = false(1, N_steps);

r_A_I(:,1) = x_A(1:3);
v_A_I(:,1) = x_A(4:6);
r_B_I(:,1) = x_B(1:3);
v_B_I(:,1) = x_B(4:6);

r_hat = r_A_I(:,1)/norm(r_A_I(:,1));
rcv = cross(r_A_I(:,1), v_A_I(:,1));
h_hat = rcv/norm(rcv);
t_hat = cross(h_hat, r_hat);
R_I2L = [r_hat, t_hat, h_hat]';

del_r = r_B_I(:,1) - r_A_I(:,1);
r_B_L(:,1) = R_I2L*del_r;
del_v = v_B_I(:,1) - v_A_I(:,1);
omega_0 = cross(r_A_I(:,1), v_A_I(:,1))/(norm(r_A_I(:,1))^2);
v_B_L(:,1) = R_I2L*(del_v - cross(omega_0, del_r));

%% 궤도 전파
fprintf('\n=== 궤도 전파 시작 ===\n');
orbit_dyn = @(x, u, params, ~) orbit_propagation_j2(0, x, u, [0;0;0], params);

for k = 1:N_steps-1
    t_current = tspan(k);
    
    x_A_current = [r_A_I(:,k); v_A_I(:,k)];
    x_B_current = [r_B_I(:,k); v_B_I(:,k)];
    
    r_hat = r_A_I(:,k)/norm(r_A_I(:,k));
    rcv = cross(r_A_I(:,k), v_A_I(:,k));
    h_hat = rcv/norm(rcv);
    t_hat = cross(h_hat, r_hat);
    R_L2I_k = [r_hat, t_hat, h_hat];
    R_I2L_k = R_L2I_k';
    
    % 제어 계산
    if t_current >= t_transition
        x_rel_L = [r_B_L(:,k); v_B_L(:,k)];
        u_L_cmd = -K*(x_rel_L - x_target);
        
        % 추력 포화
        u_L_desired(:,k) = u_L_cmd;
        thrust_desired(k) = norm(u_L_cmd);
        
        if norm(u_L_cmd) > cubesat.max_accel_kms2
            u_L = u_L_cmd / norm(u_L_cmd) * cubesat.max_accel_kms2;
            saturated(k) = true;
        else
            u_L = u_L_cmd;
        end
        
        u_I = R_L2I_k * u_L;
    else
        u_L = [0; 0; 0];
        u_I = [0; 0; 0];
        u_L_desired(:,k) = [0; 0; 0];
    end
    
    u_I_hist(:,k) = u_I;
    u_L_hist(:,k) = u_L;
    thrust_mag(k) = norm(u_I);
    
    x_A_next = rk4(orbit_dyn, x_A_current, [0;0;0], param, dt);
    x_B_next = rk4(orbit_dyn, x_B_current, u_I, param, dt);
    
    r_A_I(:,k+1) = x_A_next(1:3);
    v_A_I(:,k+1) = x_A_next(4:6);
    r_B_I(:,k+1) = x_B_next(1:3);
    v_B_I(:,k+1) = x_B_next(4:6);
    
    r_hat_next = r_A_I(:,k+1)/norm(r_A_I(:,k+1));
    rcv_next = cross(r_A_I(:,k+1), v_A_I(:,k+1));
    h_hat_next = rcv_next/norm(rcv_next);
    t_hat_next = cross(h_hat_next, r_hat_next);
    R_L2I_next = [r_hat_next, t_hat_next, h_hat_next];
    R_I2L_next = R_L2I_next';
    
    del_r_next = r_B_I(:,k+1) - r_A_I(:,k+1);
    r_B_L(:,k+1) = R_I2L_next*del_r_next;
    
    del_v_next = v_B_I(:,k+1) - v_A_I(:,k+1);
    omega_next = cross(r_A_I(:,k+1), v_A_I(:,k+1))/(norm(r_A_I(:,k+1))^2);
    v_B_L(:,k+1) = R_I2L_next*(del_v_next - cross(omega_next, del_r_next));
    
    if mod(k, 1000) == 0
        fprintf('  진행: %d/%d (%.1f%%)\n', k, N_steps, k/N_steps*100);
    end
end

fprintf('궤도 전파 완료!\n');

%% 결과 분석
time_min = tspan'/60;
idx_trans = find(tspan >= t_transition, 1);
rel_dist = vecnorm(r_B_L, 2, 1);
target_dist = vecnorm(r_B_L - x_target(1:3), 2, 1);

sat_count = sum(saturated(idx_trans:end));
sat_ratio = sat_count / (N_steps - idx_trans) * 100;

fprintf('\n=== 제어 성능 ===\n');
fprintf('최종 위치 오차: %.2f m\n', target_dist(end)*1000);
fprintf('최종 속도 오차: %.2f mm/s\n', norm(v_B_L(:,end) - x_target(4:6))*1e6);
fprintf('평균 제어력 (전환 후): %.1f μm/s²\n', mean(thrust_mag(idx_trans:end))*1e6);
fprintf('최대 제어력 요구: %.1f μm/s²\n', max(thrust_desired(idx_trans:end))*1e6);
fprintf('최대 제어력 실제: %.1f μm/s²\n', max(thrust_mag(idx_trans:end))*1e6);
fprintf('포화 발생률: %.1f%% (%d/%d steps)\n', sat_ratio, sat_count, N_steps-idx_trans);

%% 플롯팅
figure('Position', [100, 100, 1600, 1000]);

subplot(2,3,1);
plot3(r_B_L(1,:), r_B_L(2,:), r_B_L(3,:), 'b-', 'LineWidth', 1.5);
hold on;
plot3(0, 0, 0, 'k+', 'MarkerSize', 15, 'LineWidth', 2);
plot3(x_target(1), x_target(2), x_target(3), 'ro', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
plot3(r_B_L(1,idx_trans), r_B_L(2,idx_trans), r_B_L(3,idx_trans), ...
      'gs', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
grid on; axis equal;
xlabel('Radial [km]'); ylabel('In-track [km]'); zlabel('Cross-track [km]');
title('3D Formation Trajectory');
legend('Trajectory', 'Chief', 'Target', 'Transition', 'Location', 'best');
hold on
view(45, 25);

subplot(2,3,2);
plot(time_min, r_B_L(1,:)*1000, 'LineWidth', 1.5); hold on;
plot(time_min, r_B_L(2,:)*1000, 'LineWidth', 1.5);
plot(time_min, r_B_L(3,:)*1000, 'LineWidth', 1.5);
xline(t_transition/60, 'k--', 'LineWidth', 2);
grid on;
xlabel('Time [min]'); ylabel('Position [m]');
title('Position Components (LVLH)');
legend('Radial', 'In-track', 'Cross-track', 'Transition');
hold on

subplot(2,3,3);
plot(time_min, v_B_L(1,:)*1000, 'LineWidth', 1.5); hold on;
plot(time_min, v_B_L(2,:)*1000, 'LineWidth', 1.5);
plot(time_min, v_B_L(3,:)*1000, 'LineWidth', 1.5);
xline(t_transition/60, 'k--', 'LineWidth', 2);
grid on;
xlabel('Time [min]'); ylabel('Velocity [m/s]');
title('Velocity Components (LVLH)');
legend('Radial', 'In-track', 'Cross-track', 'Transition');
hold on

subplot(2,3,4);
plot(time_min, u_L_hist(1,:)*1e6, 'LineWidth', 1.5); hold on;
plot(time_min, u_L_hist(2,:)*1e6, 'LineWidth', 1.5);
plot(time_min, u_L_hist(3,:)*1e6, 'LineWidth', 1.5);
xline(t_transition/60, 'k--', 'LineWidth', 2);
grid on;
xlabel('Time [min]'); ylabel('Control [μm/s²]');
title('Control Input (LVLH)');
legend('u_x', 'u_y', 'u_z', 'Transition');
hold on

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
hold on

subplot(2,3,6);
plot(time_min, rel_dist*1000, 'b-', 'LineWidth', 1.5); hold on;
plot(time_min, target_dist*1000, 'r-', 'LineWidth', 1.5);
xline(t_transition/60, 'k--', 'LineWidth', 2);
grid on;
xlabel('Time [min]'); ylabel('Distance [m]');
title('Relative Distance');
legend('From Chief', 'From Target', 'Transition');
hold on

sgtitle(sprintf('LQR Control (Thrust: %.1f mN, Mass: %.1f kg)', ...
        cubesat.thrust_mN, cubesat.mass_kg), 'FontSize', 14, 'FontWeight', 'bold');

%% 전환 후 추적 성능
figure('Position', [150, 150, 1400, 500]);

idx_after = tspan >= t_transition;
time_after = (tspan(idx_after) - t_transition)/60;
error_pos = r_B_L(:,idx_after) - x_target(1:3);
error_vel = v_B_L(:,idx_after) - x_target(4:6);

subplot(1,3,1);
plot(time_after, error_pos(1,:)*1000, 'LineWidth', 1.5); hold on;
plot(time_after, error_pos(2,:)*1000, 'LineWidth', 1.5);
plot(time_after, error_pos(3,:)*1000, 'LineWidth', 1.5);
plot(time_after, vecnorm(error_pos,2,1)*1000, 'k--', 'LineWidth', 2);
grid on;
xlabel('Time after transition [min]'); ylabel('Error [m]');
title('Position Tracking Error');
legend('Radial', 'In-track', 'Cross-track', 'Total');
hold on

subplot(1,3,2);
plot(time_after, error_vel(1,:)*1000, 'LineWidth', 1.5); hold on;
plot(time_after, error_vel(2,:)*1000, 'LineWidth', 1.5);
plot(time_after, error_vel(3,:)*1000, 'LineWidth', 1.5);
plot(time_after, vecnorm(error_vel,2,1)*1000, 'k--', 'LineWidth', 2);
grid on;
xlabel('Time after transition [min]'); ylabel('Error [m/s]');
title('Velocity Tracking Error');
legend('Radial', 'In-track', 'Cross-track', 'Total');
hold on

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
hold on

sgtitle('Tracking Performance (After Transition)', 'FontSize', 14, 'FontWeight', 'bold');

fprintf('\n=== 완료 ===\n');