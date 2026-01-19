%% test_rk4_dual.m - rk4_ff_dual 검증 (운동량 보존 포함)
clear; clc; close all;

%% 파라미터 로드
urdf_path = '/home/ndh/space_ros_ws/install/orbit_sim/share/orbit_sim/urdf/kari_arm.urdf';
if exist(urdf_path, 'file')
    params = params_init(urdf_path);
else
    warning('URDF not found. Using dummy params.');
    params = create_dummy_params();
end

n_joints = params.arm.n_joints;

%% 초기 상태
p_base = [0; 0; 2];
q_base = [1; 0; 0; 0];
v_base = [0; 0; 0];
omega_base = [0; 0; 0];

% 관절각 (약간 굽힌 자세 - 특이점 회피)
theta_L = [0; 0.3; 0; 0.5; 0; 0.2; 0];
theta_R = [0; 0.3; 0; 0.5; 0; 0.2; 0];
theta_dot_L = zeros(7, 1);
theta_dot_R = zeros(7, 1);

%% 시뮬레이션 파라미터
dt = 0.01;  % 10ms
T_total = 5.0;  % 5초
N_steps = round(T_total / dt);

%% 토크 함수 (사인파 - 양팔 교대 운동)
tau_func = @(t) [
    5*sin(2*pi*0.5*t);    % J1_L
    3*sin(2*pi*0.5*t);    % J2_L
    2*sin(2*pi*0.5*t);    % J3_L
    3*sin(2*pi*0.5*t);    % J4_L
    1*sin(2*pi*0.5*t);    % J5_L
    1*sin(2*pi*0.5*t);    % J6_L
    0.5*sin(2*pi*0.5*t);  % J7_L
    -5*sin(2*pi*0.5*t);   % J1_R (반대 방향)
    -3*sin(2*pi*0.5*t);   % J2_R
    -2*sin(2*pi*0.5*t);   % J3_R
    -3*sin(2*pi*0.5*t);   % J4_R
    -1*sin(2*pi*0.5*t);   % J5_R
    -1*sin(2*pi*0.5*t);   % J6_R
    -0.5*sin(2*pi*0.5*t); % J7_R
];

%% 데이터 저장
time = zeros(N_steps+1, 1);
pos_base = zeros(N_steps+1, 3);
vel_base = zeros(N_steps+1, 3);
omega_base_hist = zeros(N_steps+1, 3);
momentum_linear = zeros(N_steps+1, 3);
momentum_angular = zeros(N_steps+1, 3);
theta_L_hist = zeros(N_steps+1, 7);
theta_R_hist = zeros(N_steps+1, 7);

%% 초기 운동량 계산
fprintf('=== RK4 시뮬레이션 시작 ===\n');
[P0, L0] = compute_system_momentum_dual(p_base, q_base, v_base, omega_base, ...
                                        theta_L, theta_R, theta_dot_L, theta_dot_R, params);

% 초기값 저장
time(1) = 0;
pos_base(1,:) = p_base';
vel_base(1,:) = v_base';
omega_base_hist(1,:) = omega_base';
momentum_linear(1,:) = P0';
momentum_angular(1,:) = L0';
theta_L_hist(1,:) = theta_L';
theta_R_hist(1,:) = theta_R';

fprintf('초기 선운동량: [%.6f, %.6f, %.6f]\n', P0);
fprintf('초기 각운동량: [%.6f, %.6f, %.6f]\n', L0);

%% 시뮬레이션 루프
tic;
for k = 1:N_steps
    t = (k-1) * dt;
    
    % RK4 적분
    [p_base, q_base, v_base, omega_base, theta_L, theta_R, theta_dot_L, theta_dot_R] = ...
        rk4_ff_dual(p_base, q_base, v_base, omega_base, ...
                    theta_L, theta_R, theta_dot_L, theta_dot_R, ...
                    tau_func, t, dt, params);
    
    % 운동량 계산
    [P, L] = compute_system_momentum_dual(p_base, q_base, v_base, omega_base, ...
                                          theta_L, theta_R, theta_dot_L, theta_dot_R, params);
    
    % 저장
    time(k+1) = t + dt;
    pos_base(k+1,:) = p_base';
    vel_base(k+1,:) = v_base';
    omega_base_hist(k+1,:) = omega_base';
    momentum_linear(k+1,:) = P';
    momentum_angular(k+1,:) = L';
    theta_L_hist(k+1,:) = theta_L';
    theta_R_hist(k+1,:) = theta_R';
    
    % 진행률 표시
    if mod(k, 100) == 0
        fprintf('  %.1f%% 완료...\n', 100*k/N_steps);
    end
end
elapsed = toc;
fprintf('시뮬레이션 완료 (%.2f초)\n', elapsed);

%% 운동량 보존 검증
fprintf('\n=== 운동량 보존 검증 ===\n');

P_final = momentum_linear(end,:)';
L_final = momentum_angular(end,:)';

P_err = norm(P_final - P0);
L_err = norm(L_final - L0);

P_max_var = max(vecnorm(momentum_linear - P0', 2, 2));
L_max_var = max(vecnorm(momentum_angular - L0', 2, 2));

fprintf('선운동량:\n');
fprintf('  초기: [%.6f, %.6f, %.6f]\n', P0);
fprintf('  최종: [%.6f, %.6f, %.6f]\n', P_final);
fprintf('  오차: %.6e\n', P_err);
fprintf('  최대 변동: %.6e\n', P_max_var);

fprintf('\n각운동량:\n');
fprintf('  초기: [%.6f, %.6f, %.6f]\n', L0);
fprintf('  최종: [%.6f, %.6f, %.6f]\n', L_final);
fprintf('  오차: %.6e\n', L_err);
fprintf('  최대 변동: %.6e\n', L_max_var);

if P_max_var < 1e-6 && L_max_var < 1e-3
    fprintf('\n[PASS] 운동량 보존 성공!\n');
else
    fprintf('\n[WARN] 운동량 변동 있음 (수치 오차 또는 버그 확인 필요)\n');
end

%% NaN/Inf 검증
fprintf('\n=== 안정성 검증 ===\n');
if any(isnan(pos_base(:))) || any(isinf(pos_base(:)))
    fprintf('[FAIL] 위치에 NaN/Inf\n');
elseif any(isnan(theta_L_hist(:))) || any(isinf(theta_L_hist(:)))
    fprintf('[FAIL] 관절각에 NaN/Inf\n');
else
    fprintf('[PASS] NaN/Inf 없음 - 시뮬레이션 안정\n');
end

%% 플롯
figure('Name', 'Dual-arm Free-floating Simulation', 'Position', [100 100 1200 800]);

% 위성 위치
subplot(2,3,1);
plot(time, pos_base);
xlabel('Time [s]'); ylabel('Position [m]');
title('Satellite Position');
legend('X', 'Y', 'Z');
grid on;

% 위성 속도
subplot(2,3,2);
plot(time, vel_base);
xlabel('Time [s]'); ylabel('Velocity [m/s]');
title('Satellite Velocity');
legend('Vx', 'Vy', 'Vz');
grid on;

% 위성 각속도
subplot(2,3,3);
plot(time, omega_base_hist);
xlabel('Time [s]'); ylabel('Angular Vel [rad/s]');
title('Satellite Angular Velocity');
legend('\omega_x', '\omega_y', '\omega_z');
grid on;

% 선운동량
subplot(2,3,4);
plot(time, momentum_linear);
xlabel('Time [s]'); ylabel('Momentum [kg·m/s]');
title('Linear Momentum (should be constant)');
legend('Px', 'Py', 'Pz');
grid on;

% 각운동량
subplot(2,3,5);
plot(time, momentum_angular);
xlabel('Time [s]'); ylabel('Ang. Mom. [kg·m²/s]');
title('Angular Momentum (should be constant)');
legend('Lx', 'Ly', 'Lz');
grid on;

% 관절각 (왼팔 J2, 오른팔 J2)
subplot(2,3,6);
plot(time, theta_L_hist(:,2)*180/pi, 'b-', time, theta_R_hist(:,2)*180/pi, 'r-');
xlabel('Time [s]'); ylabel('Angle [deg]');
title('Joint 2 Angles');
legend('Left J2', 'Right J2');
grid on;

%% 결과 요약
fprintf('\n========================================\n');
fprintf('시뮬레이션 결과 요약\n');
fprintf('========================================\n');
fprintf('시뮬레이션 시간: %.1f초\n', T_total);
fprintf('시간 간격: %.3f초\n', dt);
fprintf('위성 이동 거리: %.4f m\n', norm(pos_base(end,:) - pos_base(1,:)));
fprintf('위성 최종 속도: [%.4f, %.4f, %.4f] m/s\n', vel_base(end,:));
fprintf('위성 최종 각속도: [%.4f, %.4f, %.4f] rad/s\n', omega_base_hist(end,:));


%% ========== 운동량 계산 함수 ==========
function [P, L] = compute_system_momentum_dual(p_base, q_base, v_base, omega_base, ...
                                               theta_L, theta_R, theta_dot_L, theta_dot_R, params)
% 양팔 시스템 전체 운동량 계산

% FK
FK_dual = RA_FK_dual(params, p_base, q_base, theta_L, theta_R);
R_base = FK_dual.R_base;

% omega: body → inertial
omega_inertial = R_base * omega_base(:);

% 위성 운동량
m_sat = params.sat.m;
I_sat = R_base * params.sat.I * R_base';

P = m_sat * v_base(:);
L = I_sat * omega_inertial;

% 왼팔 링크 운동량
P_L = compute_arm_momentum(FK_dual.L, params, p_base, v_base, omega_inertial, theta_dot_L);
% 오른팔 링크 운동량
P_R = compute_arm_momentum(FK_dual.R, params, p_base, v_base, omega_inertial, theta_dot_R);

P = P + P_L.linear + P_R.linear;
L = L + P_L.angular + P_R.angular;

end


function mom = compute_arm_momentum(FK, params, p_base, v_base, omega_inertial, theta_dot)
% 한쪽 팔의 운동량 계산

n_bodies = params.arm.n_bodies;
n_joints = params.arm.n_joints;

P_arm = zeros(3, 1);
L_arm = zeros(3, 1);

joint_idx = 0;
omega_i = omega_inertial;

for i = 1:n_bodies
    link = params.arm.links(i);
    m_i = link.m;
    
    if m_i < 1e-10
        continue;
    end
    
    R_i = FK.R_all{i};
    I_i = R_i * link.I * R_i';
    p_com_i = FK.p_com_all{i};
    
    % 링크 각속도
    if strcmp(link.joint_type, 'revolute')
        joint_idx = joint_idx + 1;
        omega_i = omega_i + theta_dot(joint_idx) * FK.k{joint_idx};
    end
    
    % 링크 CoM 선속도
    r_rel = p_com_i - p_base(:);
    v_com_i = v_base(:) + cross(omega_inertial, r_rel);
    
    % 관절 기여 추가
    for j = 1:min(joint_idx, n_joints)
        r_j = p_com_i - FK.p_joint{j};
        v_com_i = v_com_i + cross(theta_dot(j) * FK.k{j}, r_j);
    end
    
    % 운동량 기여
    P_arm = P_arm + m_i * v_com_i;
    L_arm = L_arm + m_i * cross(p_com_i, v_com_i) + I_i * omega_i;
end

mom.linear = P_arm;
mom.angular = L_arm;

end


%% ========== Dummy params ==========
function params = create_dummy_params()
    params.sat.m = 500;
    params.sat.I = diag([260, 280, 170]);
    
    params.sat.mount(1).pos = [1.9; 0.9; 0];
    params.sat.mount(1).R = [1 0 0; 0 0 1; 0 -1 0];
    params.sat.mount(2).pos = [1.9; -0.9; 0];
    params.sat.mount(2).R = [1 0 0; 0 0 -1; 0 1 0];
    params.sat.r_mount = params.sat.mount(1).pos;
    params.sat.R_mount = params.sat.mount(1).R;
    
    params.arm.n_joints = 7;
    params.arm.n_bodies = 9;
    
    masses = [3.3436, 2.1207, 6.6621, 4.0005, 6.66208, 2.6161, 4.9702, 3.3715, 0.001];
    joint_types = {'fixed', 'revolute', 'revolute', 'revolute', 'revolute', 'revolute', 'revolute', 'revolute', 'fixed'};
    names = {'base', 'link1', 'link2', 'link3', 'link4', 'link5', 'link6', 'link7', 'eef_link'};
    joint_axes = {[0;0;1], [0;0;1], [0;1;0], [0;0;1], [0;1;0], [0;0;1], [0;1;0], [0;0;1], [0;0;1]};
    offsets = [0, 0.094, 0.193, 0.131, 0.8, 0.131, 0.408, 0.088, 0.097];
    
    for i = 1:9
        params.arm.links(i).name = names{i};
        params.arm.links(i).m = masses(i);
        params.arm.links(i).I = eye(3) * 0.01;
        params.arm.links(i).com = [0; 0; 0.05];
        params.arm.links(i).joint_type = joint_types{i};
        params.arm.links(i).joint_axis = joint_axes{i};
        params.arm.links(i).T_fixed = eye(4);
        params.arm.links(i).T_fixed(3,4) = offsets(i);
    end
end