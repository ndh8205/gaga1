%% compare_gazebo_simscape.m
% Gazebo vs Simscape 비교
% 
% =========================================================================
% 비교 대상:
%   1. Gazebo: CSV 데이터
%   2. Simscape: 참조 데이터
% =========================================================================

clear; clc; close all;
addpath(genpath('D:\pj2025\space_challenge'));

r2d = 180/pi;
d2r = 1/r2d;

%% ========== 경로 설정 ==========
simscape_path = 'D:\pj2025\space_challenge\ref\simscape\';

%% ========== Gazebo 데이터 로드 ==========
% 최신 CSV 파일 자동 검색
files = dir([simscape_path, 'gazebo_log_*.csv']);
if isempty(files)
    error('gazebo_log_*.csv 파일을 찾을 수 없습니다. %s 폴더를 확인하세요.', simscape_path);
end
[~, idx] = max([files.datenum]);
gazebo_file = fullfile(simscape_path, files(idx).name);
fprintf('로드 파일: %s\n', gazebo_file);

gz_data = readtable(gazebo_file);

time_gz = gz_data.time;
theta_gz = [gz_data.q1, gz_data.q2, gz_data.q3, gz_data.q4, gz_data.q5, gz_data.q6, gz_data.q7];
theta_dot_gz = [gz_data.qd1, gz_data.qd2, gz_data.qd3, gz_data.qd4, gz_data.qd5, gz_data.qd6, gz_data.qd7];
r_gz = [gz_data.sat_x, gz_data.sat_y, gz_data.sat_z];
quat_gz = [gz_data.sat_qw, gz_data.sat_qx, gz_data.sat_qy, gz_data.sat_qz];

N_gz = length(time_gz);
fprintf('Gazebo 데이터: %d 샘플, %.2f ~ %.2f sec\n', N_gz, time_gz(1), time_gz(end));

%% ========== Simscape 데이터 로드 ==========
Joint1_q = load([simscape_path, 'Joint1_q.mat']); Joint1_q = Joint1_q.ans;
Joint2_q = load([simscape_path, 'Joint2_q.mat']); Joint2_q = Joint2_q.ans;
Joint3_q = load([simscape_path, 'Joint3_q.mat']); Joint3_q = Joint3_q.ans;
Joint4_q = load([simscape_path, 'Joint4_q.mat']); Joint4_q = Joint4_q.ans;
Joint5_q = load([simscape_path, 'Joint5_q.mat']); Joint5_q = Joint5_q.ans;
Joint6_q = load([simscape_path, 'Joint6_q.mat']); Joint6_q = Joint6_q.ans;
Joint7_q = load([simscape_path, 'Joint7_q.mat']); Joint7_q = Joint7_q.ans;

base_x = load([simscape_path, 'base_x.mat']); base_x = base_x.ans;
base_y = load([simscape_path, 'base_y.mat']); base_y = base_y.ans;
base_z = load([simscape_path, 'base_z.mat']); base_z = base_z.ans;
base_quat = load([simscape_path, 'base_quat.mat']); base_quat = base_quat.ans;

T_max = 5;
idx_end = find(Joint1_q.Time <= T_max, 1, 'last');
time_sim = Joint1_q.Time(1:idx_end);
N_sim = length(time_sim);

theta_sim = zeros(N_sim, 7);
theta_sim(:,1) = Joint1_q.Data(1:idx_end);
theta_sim(:,2) = Joint2_q.Data(1:idx_end);
theta_sim(:,3) = Joint3_q.Data(1:idx_end);
theta_sim(:,4) = Joint4_q.Data(1:idx_end);
theta_sim(:,5) = Joint5_q.Data(1:idx_end);
theta_sim(:,6) = Joint6_q.Data(1:idx_end);
theta_sim(:,7) = Joint7_q.Data(1:idx_end);

r_sim = zeros(N_sim, 3);
r_sim(:,1) = base_x.Data(1:idx_end);
r_sim(:,2) = base_y.Data(1:idx_end);
r_sim(:,3) = base_z.Data(1:idx_end);

% Simscape quaternion 처리 (4x1x1001 -> Nx4)
quat_sim_raw = squeeze(base_quat.Data);  % 4x1001
quat_sim = quat_sim_raw(:, 1:idx_end)';  % 1001x4 -> idx_end x 4
% 순서 확인: [qw, qx, qy, qz] 맞추기
if abs(quat_sim(1,1) - 1) > 0.1 && abs(quat_sim(1,4) - 1) < 0.1
    quat_sim = [quat_sim(:,4), quat_sim(:,1:3)];  % [x,y,z,w] -> [w,x,y,z]
end

fprintf('Simscape 데이터: %d 샘플\n', N_sim);
fprintf('\n=== Gazebo vs Simscape 비교 ===\n\n');

%% ========== 스파이크 제거 ==========
threshold_deg = 1.0;  % 0.01초에 1도 이상 변화는 비정상
threshold_rad = deg2rad(threshold_deg);

% 관절각 스파이크 제거
for j = 1:7
    for i = 2:N_gz-1
        diff_prev = abs(theta_gz(i,j) - theta_gz(i-1,j));
        diff_next = abs(theta_gz(i+1,j) - theta_gz(i,j));
        if diff_prev > threshold_rad && diff_next > threshold_rad
            theta_gz(i,j) = (theta_gz(i-1,j) + theta_gz(i+1,j)) / 2;
        end
    end
end

% 위성 위치 스파이크 제거 (1mm 이상 변화)
threshold_pos = 0.001;  % 1mm
for j = 1:3
    for i = 2:N_gz-1
        diff_prev = abs(r_gz(i,j) - r_gz(i-1,j));
        diff_next = abs(r_gz(i+1,j) - r_gz(i,j));
        if diff_prev > threshold_pos && diff_next > threshold_pos
            r_gz(i,j) = (r_gz(i-1,j) + r_gz(i+1,j)) / 2;
        end
    end
end

% Quaternion 스파이크 제거 (0.01 이상 변화)
threshold_quat = 0.01;
for j = 1:4
    for i = 2:N_gz-1
        diff_prev = abs(quat_gz(i,j) - quat_gz(i-1,j));
        diff_next = abs(quat_gz(i+1,j) - quat_gz(i,j));
        if diff_prev > threshold_quat && diff_next > threshold_quat
            quat_gz(i,j) = (quat_gz(i-1,j) + quat_gz(i+1,j)) / 2;
        end
    end
end
% Quaternion 재정규화
for i = 1:N_gz
    quat_gz(i,:) = quat_gz(i,:) / norm(quat_gz(i,:));
end

%% ========== Interpolation ==========
theta_interp_gz = interp1(time_gz, theta_gz, time_sim, 'linear', 'extrap');
r_interp_gz = interp1(time_gz, r_gz, time_sim, 'linear', 'extrap');
quat_interp_gz = interp1(time_gz, quat_gz, time_sim, 'linear', 'extrap');

% Quaternion 정규화
for i = 1:N_sim
    quat_interp_gz(i,:) = quat_interp_gz(i,:) / norm(quat_interp_gz(i,:));
end

%% ========== 초기 오프셋 보정 ==========
r_offset = r_interp_gz(1,:) - r_sim(1,:);
r_interp_gz_corrected = r_interp_gz - r_offset;

%% ========== 오차 계산 ==========
theta_err = theta_interp_gz - theta_sim;
r_err = r_interp_gz_corrected - r_sim;

% Quaternion 오차 (각도 오차로 변환) - 사용자 함수 사용
quat_err_angle = zeros(N_sim, 1);
for i = 1:N_sim
    q1 = quat_interp_gz(i,:)';  % [w;x;y;z] 열벡터
    q2 = quat_sim(i,:)';
    % q_err = q1 * q2^(-1)
    q2_inv = inv_q(q2);
    q_err = q2q_mult(q1, q2_inv);
    % 각도 오차
    [angle, ~] = Quat2AngleAxis(q_err);
    quat_err_angle(i) = angle;
end

%% ========== 결과 출력 ==========
fprintf('============================================================\n');
fprintf('                     최대 오차 비교\n');
fprintf('============================================================\n');

fprintf('\n--- 관절각 오차 [deg] ---\n');
fprintf('Joint |  Gazebo vs Simscape\n');
fprintf('------+--------------------\n');
for i = 1:7
    fprintf('  J%d  |     %9.6f\n', i, rad2deg(max(abs(theta_err(:,i)))));
end

fprintf('\n--- 위성 위치 오차 [mm] ---\n');
fprintf(' Axis |  Gazebo vs Simscape\n');
fprintf('------+--------------------\n');
fprintf('  x   |     %9.6f\n', max(abs(r_err(:,1)))*1000);
fprintf('  y   |     %9.6f\n', max(abs(r_err(:,2)))*1000);
fprintf('  z   |     %9.6f\n', max(abs(r_err(:,3)))*1000);

fprintf('\n--- Quaternion 오차 [deg] ---\n');
fprintf('  Max |     %9.6f\n', rad2deg(max(quat_err_angle)));

%% ========== 플롯 1: 관절각 비교 ==========
figure('Name', '관절각 비교', 'Position', [50, 50, 1400, 800]);

for i = 1:7
    subplot(2,4,i);
    plot(time_sim, rad2deg(theta_interp_gz(:,i)), 'b-', 'LineWidth', 1.5);
    hold on;
    plot(time_sim, rad2deg(theta_sim(:,i)), 'r--', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Angle [deg]');
    title(sprintf('Joint %d', i));
    if i == 1
        legend('Gazebo', 'Simscape', 'Location', 'best');
    end
    grid on;
end

subplot(2,4,8);
bar(1:7, rad2deg(max(abs(theta_err))));
xlabel('Joint'); ylabel('Max Error [deg]');
title('관절각 최대 오차');
grid on;

sgtitle('관절각 비교: Gazebo vs Simscape');

%% ========== 플롯 2: 위성 위치 ==========
figure('Name', '위성 위치 비교', 'Position', [100, 100, 1400, 500]);

subplot(1,3,1);
plot(time_sim, r_interp_gz_corrected*1000, '-', 'LineWidth', 1.5);
hold on;
plot(time_sim, r_sim*1000, '--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Position [mm]');
title('Gazebo vs Simscape');
legend('x(GZ)','y(GZ)','z(GZ)','x(S)','y(S)','z(S)', 'Location', 'best');
grid on;

subplot(1,3,2);
plot(time_sim, r_err*1000, '-', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Error [mm]');
title('위치 오차');
legend('x','y','z', 'Location', 'best');
grid on;

subplot(1,3,3);
plot3(r_interp_gz_corrected(:,1)*1000, r_interp_gz_corrected(:,2)*1000, r_interp_gz_corrected(:,3)*1000, 'b-', 'LineWidth', 1.5);
hold on;
plot3(r_sim(:,1)*1000, r_sim(:,2)*1000, r_sim(:,3)*1000, 'r--', 'LineWidth', 1.5);
xlabel('X [mm]'); ylabel('Y [mm]'); zlabel('Z [mm]');
title('3D 궤적');
legend('Gazebo', 'Simscape', 'Location', 'best');
grid on;
axis equal;

sgtitle('위성 위치 비교: Gazebo vs Simscape');

%% ========== 플롯 3: Quaternion 비교 ==========
figure('Name', 'Quaternion 비교', 'Position', [150, 150, 1400, 500]);

subplot(1,3,1);
plot(time_sim, quat_interp_gz, '-', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Quaternion');
title('Gazebo Quaternion');
legend('qw','qx','qy','qz', 'Location', 'best');
grid on;

subplot(1,3,2);
plot(time_sim, quat_sim, '--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Quaternion');
title('Simscape Quaternion');
legend('qw','qx','qy','qz', 'Location', 'best');
grid on;

subplot(1,3,3);
plot(time_sim, rad2deg(quat_err_angle), 'k-', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Error [deg]');
title('Quaternion 각도 오차');
grid on;

sgtitle('Quaternion 비교: Gazebo vs Simscape');

%% ========== t=5초 상세 출력 ==========
fprintf('\n============================================================\n');
fprintf('                    t=5초 상세 비교\n');
fprintf('============================================================\n');

fprintf('\n--- 관절각 [deg] ---\n');
fprintf('Joint |   Gazebo   | Simscape  |   Error\n');
fprintf('------+------------+-----------+-----------\n');
for i = 1:7
    fprintf('  J%d  | %10.4f | %9.4f | %9.4f\n', i, ...
        rad2deg(theta_interp_gz(end,i)), ...
        rad2deg(theta_sim(end,i)), ...
        rad2deg(theta_err(end,i)));
end

fprintf('\n--- 위성 위치 [mm] ---\n');
fprintf(' Axis |   Gazebo   | Simscape  |   Error\n');
fprintf('------+------------+-----------+-----------\n');
fprintf('  x   | %10.4f | %9.4f | %9.4f\n', r_interp_gz_corrected(end,1)*1000, r_sim(end,1)*1000, r_err(end,1)*1000);
fprintf('  y   | %10.4f | %9.4f | %9.4f\n', r_interp_gz_corrected(end,2)*1000, r_sim(end,2)*1000, r_err(end,2)*1000);
fprintf('  z   | %10.4f | %9.4f | %9.4f\n', r_interp_gz_corrected(end,3)*1000, r_sim(end,3)*1000, r_err(end,3)*1000);

fprintf('\n--- Quaternion ---\n');
fprintf('      |   Gazebo   | Simscape  \n');
fprintf('------+------------+-----------\n');
fprintf('  qw  | %10.6f | %9.6f\n', quat_interp_gz(end,1), quat_sim(end,1));
fprintf('  qx  | %10.6f | %9.6f\n', quat_interp_gz(end,2), quat_sim(end,2));
fprintf('  qy  | %10.6f | %9.6f\n', quat_interp_gz(end,3), quat_sim(end,3));
fprintf('  qz  | %10.6f | %9.6f\n', quat_interp_gz(end,4), quat_sim(end,4));
fprintf('  Err | %10.6f deg\n', rad2deg(quat_err_angle(end)));

fprintf('\n============================================================\n');