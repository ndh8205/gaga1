%% test_physical_dt.m - 물리적 운동량 dt 민감도
clear; clc;

params = create_dummy_params();

p0 = [0; 0; 2]; q0 = [1; 0; 0; 0];
v0 = [0.01; 0; 0]; w0 = [0; 0; 0.02];
th_L0 = [0; 0.3; 0; 0.5; 0; 0.2; 0];
th_R0 = [0; 0.3; 0; 0.5; 0; 0.2; 0];
thd_L0 = [0.05; 0.1; 0; 0.05; 0; 0; 0];
thd_R0 = [-0.05; -0.1; 0; -0.05; 0; 0; 0];

tau_zero = @(t) zeros(14, 1);
T_total = 1.0;

dt_list = [0.01, 0.005, 0.002, 0.001];

fprintf('=== 물리적 운동량 dt 민감도 ===\n');
fprintf('%-8s %-12s %-12s %-12s\n', 'dt', 'P_err', 'L_err', 'L_err/dt^4');

for dt = dt_list
    p = p0; q = q0; v = v0; w = w0;
    th_L = th_L0; th_R = th_R0;
    thd_L = thd_L0; thd_R = thd_R0;
    
    [P0, L0] = compute_physical_momentum(params, p, q, v, w, th_L, th_R, thd_L, thd_R);
    
    N = round(T_total / dt);
    for k = 1:N
        [p, q, v, w, th_L, th_R, thd_L, thd_R] = ...
            rk4_ff_dual(p, q, v, w, th_L, th_R, thd_L, thd_R, tau_zero, (k-1)*dt, dt, params);
    end
    
    [P, L] = compute_physical_momentum(params, p, q, v, w, th_L, th_R, thd_L, thd_R);
    
    P_err = norm(P - P0);
    L_err = norm(L - L0);
    
    fprintf('%-8.4f %-12.2e %-12.2e %-12.2f\n', dt, P_err, L_err, L_err/dt^4);
end

fprintf('\ndt^4에 비례하면 RK4 정상\n');


%% 운동량 계산
function [P, L] = compute_physical_momentum(params, p_base, q_base, v_base, omega_base, ...
                                            theta_L, theta_R, theta_dot_L, theta_dot_R)
R_base = GetDCM_QUAT(q_base);
omega_base_I = R_base * omega_base(:);

m_sat = params.sat.m;
I_sat_I = R_base * params.sat.I * R_base';

P = m_sat * v_base(:);
L = cross(p_base(:), m_sat * v_base(:)) + I_sat_I * omega_base_I;

FK_dual = RA_FK_dual(params, p_base, q_base, theta_L, theta_R);

[P_L, L_L] = compute_arm_momentum(FK_dual.L, params, p_base, v_base, omega_base_I, theta_dot_L);
[P_R, L_R] = compute_arm_momentum(FK_dual.R, params, p_base, v_base, omega_base_I, theta_dot_R);

P = P + P_L + P_R;
L = L + L_L + L_R;
end

function [P_arm, L_arm] = compute_arm_momentum(FK, params, p_base, v_base, omega_base_I, theta_dot)
n_bodies = params.arm.n_bodies;
n_joints = params.arm.n_joints;
P_arm = zeros(3,1); L_arm = zeros(3,1);

joint_idx = 0;
omega_link = omega_base_I;

for i = 1:n_bodies
    link = params.arm.links(i);
    m_i = link.m;
    if m_i < 1e-10, continue; end
    
    R_i = FK.R_all{i};
    I_i = R_i * link.I * R_i';
    p_com_i = FK.p_com_all{i};
    
    if strcmp(link.joint_type, 'revolute')
        joint_idx = joint_idx + 1;
        omega_link = omega_link + theta_dot(joint_idx) * FK.k{joint_idx};
    end
    
    v_com_i = v_base(:) + cross(omega_base_I, p_com_i - p_base(:));
    for j = 1:min(joint_idx, n_joints)
        v_com_i = v_com_i + theta_dot(j) * cross(FK.k{j}, p_com_i - FK.p_joint{j});
    end
    
    P_arm = P_arm + m_i * v_com_i;
    L_arm = L_arm + cross(p_com_i, m_i * v_com_i) + I_i * omega_link;
end
end

%% Dummy params
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