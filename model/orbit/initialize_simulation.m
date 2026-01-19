function [t1, r_A_I, v_A_I, r_B_I, v_B_I, LVLH_pos, LVLH_vel, ...
          orbit_normal_vec, initial_pointcloud, param, dt, tt, ...
          camera_interval, Rk_IMU, Rk_ST] = initialize_simulation(varargin)
% 시뮬레이션 전체 초기화 (올인원)
%
% Usage:
%   [t1, r_A_I, v_A_I, ...] = initialize_simulation();  % 기본값
%   [...] = initialize_simulation(duration_min);        % 시간만 지정
%   [...] = initialize_simulation(duration_min, camera_interval_sec);
%   [...] = initialize_simulation(duration_min, camera_interval_sec, dt_sec);
%
% Inputs (모두 선택):
%   duration_min        - 시뮬레이션 시간 [분] (기본: 10)
%   camera_interval_sec - 카메라 간격 [초] (기본: 60)
%   dt_sec              - 샘플링 간격 [초] (기본: 0.2)
%
% Outputs (기존 변수명 그대로):
%   t1                - 시간 벡터 (Nx1) [s]
%   r_A_I, v_A_I      - Chief 궤도 (3xN) [km, km/s]
%   r_B_I, v_B_I      - Deputy 궤도 (3xN) [km, km/s]
%   LVLH_pos, LVLH_vel- Deputy LVLH 상태 (3xN) [km, km/s]
%   orbit_normal_vec  - 궤도 법선 (3x1)
%   initial_pointcloud- Chief 포인트 클라우드 (3xM) [km]
%   param             - 궤도 파라미터 구조체
%   dt, tt            - 샘플링 간격 [s], 총 시간 [s]
%   camera_interval   - 카메라 측정 간격 [s]
%   Rk_IMU, Rk_ST     - 센서 노이즈 공분산

fprintf('\n========== 시뮬레이션 초기화 ==========\n');

%% 입력 파싱
p = inputParser;
addOptional(p, 'duration_min', 90, @isnumeric);
addOptional(p, 'camera_interval_sec', 60, @isnumeric);
addOptional(p, 'dt_sec', 0.2, @isnumeric);
parse(p, varargin{:});

duration_min = p.Results.duration_min;
camera_interval = p.Results.camera_interval_sec;
dt = p.Results.dt_sec;
tt = duration_min * 60;

fprintf('설정: %.1f분, %.1fHz, 카메라 %.0fs 간격\n', duration_min, 1/dt, camera_interval);

%% 1. 궤도 상수
Mu  = 3.98600441500000e+05;
R_e = 6.37813630000000e+03;
J2  = 0.001082627;

param.orbit.Mu  = Mu;
param.orbit.R_e = R_e;
param.orbit.J2  = J2;

%% 2. Chief 초기 상태
e_A     = 0;
a_A     = 6892.137;
h_A     = sqrt(a_A*(1-e_A^2)*Mu);
i_A     = deg2rad(97.45);
RAAN_A  = deg2rad(185.898);
omega_A = deg2rad(90);
theta_A = deg2rad(0);

[r_A, v_A] = coe2rv([h_A, e_A, RAAN_A, i_A, omega_A, theta_A], Mu);
x_A = [r_A; v_A];

%% 3. Deputy 초기 상태 (GCO)
n = sqrt(Mu/a_A^3);
p_pi_B   = deg2rad(0);
r_init_B = 0.15;

x_0_B     = r_init_B/2 * sin(p_pi_B);
x_0_dot_B = r_init_B*n/2 * cos(p_pi_B);
y_0_B     = 2*x_0_dot_B/n;
y_0_dot_B = -2*n*x_0_B;
z_0_B     = sqrt(3)*x_0_B;
z_0_dot_B = sqrt(3)*x_0_dot_B;

r_0_B = [x_0_B; y_0_B; z_0_B];
v_0_B = [x_0_dot_B; y_0_dot_B; z_0_dot_B];

% LVLH -> ECI
hB_I  = cross(r_A, v_A);
i_I   = r_A/norm(r_A);
k_I   = hB_I/norm(hB_I);
j_I   = cross(k_I, i_I);
R_L2I = [i_I, j_I, k_I];

w_r_B = [-n*r_0_B(2); n*r_0_B(1); 0];
r_B   = r_A + R_L2I*r_0_B;
v_B   = v_A + R_L2I*(v_0_B + w_r_B);
x_B   = [r_B; v_B];

fprintf('Chief/Deputy 초기 상태 계산 완료\n');

%% 4. 궤도 전파
u   = [0;0;0];
w_I = [0;0;0];
tspan = 0:dt:tt;
opts  = odeset('Reltol',1e-12,'AbsTol',1e-12,'Stats','off');

fprintf('궤도 전파 중...\n');
[t1, x_A_out] = ode78(@(t,X) orbit_propagation_j2(t,X,u,w_I,param), tspan, x_A, opts);
[~,  x_B_out] = ode78(@(t,X) orbit_propagation_j2(t,X,u,w_I,param), tspan, x_B, opts);
fprintf('  완료 (%d 시점)\n', length(t1));

%% 5. ECI -> LVLH 변환
r_A_I = x_A_out(:,1:3)';
v_A_I = x_A_out(:,4:6)';
r_B_I = x_B_out(:,1:3)';
v_B_I = x_B_out(:,4:6)';

N = length(t1);
LVLH_pos = zeros(3, N);
LVLH_vel = zeros(3, N);

for i = 1:N
    r_hat = r_A_I(:,i)/norm(r_A_I(:,i));
    rcv   = cross(r_A_I(:,i), v_A_I(:,i));
    h_hat = rcv/norm(rcv);
    t_hat = cross(h_hat, r_hat);
    R_I2L = [r_hat, t_hat, h_hat]';
    
    del_r = r_B_I(:,i) - r_A_I(:,i);
    LVLH_pos(:,i) = R_I2L * del_r;
    
    del_v = v_B_I(:,i) - v_A_I(:,i);
    omega = cross(r_A_I(:,i), v_A_I(:,i))/norm(r_A_I(:,i))^2;
    LVLH_vel(:,i) = R_I2L * (del_v - cross(omega, del_r));
end

fprintf('LVLH 변환 완료\n');

%% 6. 궤도 법선 벡터
[coeff, ~, ~] = pca(LVLH_pos');
orbit_normal_vec = coeff(:,3);
if orbit_normal_vec(3) < 0
    orbit_normal_vec = -orbit_normal_vec;
end

%% 7. Point Cloud 로드
mat_filename = 'chief_pointcloud.mat';
if exist(mat_filename, 'file')
    fprintf('Point Cloud 로드: %s\n', mat_filename);
    load(mat_filename, 'ptCloud', 'points');
    
    all_points_km = points' / 1000;
    total_points = size(all_points_km, 2);
    
    max_display = 1500;
    if total_points > max_display
        step = floor(total_points/max_display);
        idx = 1:step:total_points;
        idx = idx(1:min(max_display, length(idx)));
        fprintf('  다운샘플링: %d -> %d\n', total_points, length(idx));
    else
        idx = 1:total_points;
    end
    
    initial_pointcloud = all_points_km(:, idx);
    center = mean(initial_pointcloud, 2);
    initial_pointcloud = initial_pointcloud - center;
    fprintf('  최종 포인트: %d (중심 보정)\n', size(initial_pointcloud,2));
else
    fprintf('경고: Point Cloud 없음. 기본 큐브 생성\n');
    [X,Y,Z] = meshgrid(linspace(-0.05, 0.05, 5));
    initial_pointcloud = [X(:)'; Y(:)'; Z(:)'];
end

%% 8. 센서 노이즈 공분산
Rk_IMU = eye(6);
Rk_ST  = eye(4);

fprintf('======================================\n');
fprintf('초기화 완료!\n');
fprintf('  예상 측정: IMU/ST %d회, Camera %d회\n', ...
    length(t1), floor(tt/camera_interval));
fprintf('======================================\n\n');

end