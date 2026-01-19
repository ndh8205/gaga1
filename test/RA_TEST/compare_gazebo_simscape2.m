%% compare_matlab_gazebo.m
% MATLAB (rk4_ff) 시뮬레이션 실행 + Gazebo CSV 로드 + 비교
% 토크 명령 로깅 포함

clear; clc; close all;
addpath(genpath('D:\pj2025\space_challenge'));

simscape_path = 'D:\pj2025\space_challenge\ref\simscape\';

%% ========== 1. Gazebo 데이터 로드 ==========
gz_files = dir([simscape_path, 'gazebo_log_*.csv']);
if isempty(gz_files)
    error('gazebo_log_*.csv 파일 없음. Gazebo 테스트 먼저 실행하세요.');
end
[~, idx] = max([gz_files.datenum]);
gz_file = fullfile(simscape_path, gz_files(idx).name);
fprintf('Gazebo 로드: %s\n', gz_file);
gz = readtable(gz_file);

% Gazebo 데이터 추출
t_gz = gz.time;
if ismember('torque_cmd', gz.Properties.VariableNames)
    tau_gz = gz.torque_cmd;
else
    tau_gz = gz.torque;
end
theta_gz = [gz.q1, gz.q2, gz.q3, gz.q4, gz.q5, gz.q6, gz.q7];
N_gz = height(gz);
fprintf('Gazebo: %d 샘플, %.3f ~ %.3f sec\n\n', N_gz, t_gz(1), t_gz(end));

%% ========== 2. MATLAB (rk4_ff) 시뮬레이션 ==========
fprintf('MATLAB (rk4_ff) 시뮬레이션 시작...\n');

urdf_path = 'D:\pj2025\space_challenge\model\modeling_3d\ASM_KARI_ARM\ASM_KARI_ARM_URDF.urdf';
params = params_init_old(urdf_path);

dt = 0.01;
T_max = 5;
t_vec = 0:dt:T_max;
N = length(t_vec);

% 초기 조건
p_cur = [0; 0; 0];
q_cur = [1; 0; 0; 0];
v_cur = [0; 0; 0];
omega_cur = [0; 0; 0];
theta_cur = zeros(7, 1);
theta_dot_cur = zeros(7, 1);

% 로깅 배열
log_time = zeros(1, N);
log_torque = zeros(1, N);
log_theta = zeros(7, N);

tic;
for k = 1:N
    t = t_vec(k);
    
    % 토크 명령 (Gazebo와 동일)
    tau_cmd = 0.1*sin(t);
    
    % 로깅
    log_time(k) = t;
    log_torque(k) = tau_cmd;
    log_theta(:,k) = theta_cur;
    
    if k == N, break; end
    
    % RK4 적분
    tau_func = @(tt) [0.1*sin(tt); zeros(6,1)];
    [p_cur, q_cur, v_cur, omega_cur, theta_cur, theta_dot_cur] = ...
        rk4_ff(p_cur, q_cur, v_cur, omega_cur, theta_cur, theta_dot_cur, ...
               tau_func, t, dt, params);
end
elapsed = toc;
fprintf('완료! (%.2f sec)\n\n', elapsed);

% 1ms 간격 (Gazebo 로깅과 동일)
ds_idx = 1:1:N;
t_mat = log_time(ds_idx)';
tau_mat = log_torque(ds_idx)';
theta_mat = log_theta(:, ds_idx)';

%% ========== 3. 스파이크 제거 (outlier filtering) ==========
% 연속 샘플 간 변화가 너무 크면 보간으로 대체
threshold_deg = 1.0;  % 0.01초에 1도 이상 변화는 비정상
threshold_rad = deg2rad(threshold_deg);

for j = 1:7
    for i = 2:length(t_gz)-1
        diff_prev = abs(theta_gz(i,j) - theta_gz(i-1,j));
        diff_next = abs(theta_gz(i+1,j) - theta_gz(i,j));
        if diff_prev > threshold_rad && diff_next > threshold_rad
            % 스파이크 → 선형 보간
            theta_gz(i,j) = (theta_gz(i-1,j) + theta_gz(i+1,j)) / 2;
        end
    end
end

%% ========== 4. 보간 (시간축 맞춤) ==========
theta_mat_interp = interp1(t_mat, theta_mat, t_gz, 'linear', 'extrap');
tau_mat_interp = interp1(t_mat, tau_mat, t_gz, 'linear', 'extrap');

%% ========== 5. 비교 결과 ==========
fprintf('============================================================\n');
fprintf('              토크 명령 분석\n');
fprintf('============================================================\n');
tau_diff = tau_gz - tau_mat_interp;
fprintf('Gazebo 토크 범위: [%.4f, %.4f] Nm\n', min(tau_gz), max(tau_gz));
fprintf('MATLAB 토크 범위: [%.4f, %.4f] Nm\n', min(tau_mat), max(tau_mat));
fprintf('토크 차이 최대값: %.6f Nm\n', max(abs(tau_diff)));

fprintf('\n============================================================\n');
fprintf('              관절각 최대 오차 [deg]\n');
fprintf('============================================================\n');
err = zeros(7,1);
for i = 1:7
    err(i) = rad2deg(max(abs(theta_gz(:,i) - theta_mat_interp(:,i))));
    fprintf('  J%d: %.4f\n', i, err(i));
end

fprintf('\n============================================================\n');
fprintf('              t=5초 값 비교\n');
fprintf('============================================================\n');
fprintf('Joint |   Gazebo   |   MATLAB   |   Error [deg]\n');
fprintf('------+------------+------------+--------------\n');
for i = 1:7
    fprintf('  J%d  | %10.4f | %10.4f | %12.4f\n', i, ...
        rad2deg(theta_gz(end,i)), rad2deg(theta_mat(end,i)), ...
        rad2deg(theta_gz(end,i) - theta_mat(end,i)));
end

%% ========== 6. 플롯 ==========
figure('Name', 'Gazebo vs MATLAB 비교', 'Position', [50 50 1400 800]);

% 토크 명령 비교
subplot(2,4,1);
plot(t_gz, tau_gz, 'b-', 'LineWidth', 1.5); hold on;
plot(t_mat, tau_mat, 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Torque [Nm]');
title('토크 명령');
legend('Gazebo', 'MATLAB', 'Location', 'best');
grid on;

% 관절각 비교 (J1~J7)
for i = 1:7
    subplot(2,4,i+1);
    plot(t_gz, rad2deg(theta_gz(:,i)), 'b-', 'LineWidth', 1.5); hold on;
    plot(t_mat, rad2deg(theta_mat(:,i)), 'r--', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Angle [deg]');
    title(sprintf('Joint %d (err: %.2f°)', i, err(i)));
    if i == 1
        legend('Gazebo', 'MATLAB', 'Location', 'best');
    end
    grid on;
end

sgtitle('Gazebo vs MATLAB (rk4\_ff) 비교');

%% ========== 7. 오차 시계열 플롯 ==========
figure('Name', '오차 시계열', 'Position', [100 100 1200 400]);

subplot(1,2,1);
plot(t_gz, tau_diff, 'k-', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Torque Diff [Nm]');
title('토크 명령 차이 (Gazebo - MATLAB)');
grid on;

subplot(1,2,2);
theta_err = theta_gz - theta_mat_interp;
plot(t_gz, rad2deg(theta_err), 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('Error [deg]');
title('관절각 오차');
legend('J1','J2','J3','J4','J5','J6','J7', 'Location', 'best');
grid on;

sgtitle('오차 시계열');