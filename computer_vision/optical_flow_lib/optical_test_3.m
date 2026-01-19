clc; clear all; close all;
addpath(genpath('C:\Users\USER\Desktop\relative2'));

%% 데이터 로드
data_dir = fullfile(pwd, 'sim_data');
files = dir(fullfile(data_dir, 'sim_data_ver2_*.mat'));
latest_file = fullfile(data_dir, files(end).name);
load(latest_file);

fprintf('=== 6-DOF 옵티컬 플로우 (병진 + 회전) ===\n\n');

%% 카메라 파라미터
K = camera.params.intrinsic.K_ideal;
K_inv = inv(K);
R_B2C = [1, 0, 0; 0, 0, -1; 0, 1, 0];
R_C2B = R_B2C';

fx = K(1,1);
fy = K(2,2);
cx = K(1,3);
cy = K(2,3);

fprintf('카메라: fx=%.2f, fy=%.2f\n', fx, fy);

%% Point Cloud
points_A = camera.pointcloud;
fprintf('특징점: %d개\n\n', size(points_A, 2));

%% 6-DOF 옵티컬 플로우 추정
n_cam = length(camera.times_s);
results = struct();
valid_count = 0;

for i = 1:n_cam-1
    t1 = camera.times_s(i);
    t2 = camera.times_s(i+1);
    dt = t2 - t1;
    
    z1 = camera.z_measured{i};
    z2 = camera.z_measured{i+1};
    
    if isempty(z1) || isempty(z2)
        continue;
    end
    
    n_pts = min(size(z1, 2), size(z2, 2));
    if n_pts < 6; continue; end
    
    % Ground truth
    [~, idx1] = min(abs(time.t_s - t1));
    [~, idx2] = min(abs(time.t_s - t2));
    
    % ω^B_{B/I}: I 기준 B가 회전, B frame 관측
    omega_sIeB_observB_true = quat.omega_sIeB_observB(:, idx1);
    
    % v^L_{B/L}: L 기준 B가 병진, LVLH frame 관측
    r_B_L_1 = traj.r_B_L(:, idx1);
    r_B_L_2 = traj.r_B_L(:, idx2);
    v_sLeB_observL_true = (r_B_L_2 - r_B_L_1) / dt;
    
    %% 좌표 변환 체인 구성
    r_A_I = traj.r_A_I(:, idx1);
    v_A_I = traj.v_A_I(:, idx1);
    
    r_hat = r_A_I / norm(r_A_I);
    h_vec = cross(r_A_I, v_A_I);
    h_hat = h_vec / norm(h_vec);
    t_hat = cross(h_hat, r_hat);
    R_L2I = [r_hat, t_hat, h_hat];
    R_I2L = R_L2I';
    
    q_L2A = quat.q_L2A(:, idx1);
    q_I2B = quat.q_I2B(:, idx1);
    
    R_L2A = GetDCM_QUAT(q_L2A);
    R_A2L = R_L2A';
    R_I2B = GetDCM_QUAT(q_I2B);
    R_L2B = R_I2B * R_L2I;
    R_L2C = R_B2C * R_L2B;
    R_C2L = R_L2C';
    
    % 특징점: A → LVLH → Camera
    points_L = R_A2L * points_A(:, 1:n_pts);
    points_C = zeros(3, n_pts);
    for j = 1:n_pts
        points_C(:, j) = R_L2C * (points_L(:, j) - r_B_L_1);
    end
    
    %% 옵티컬 플로우
    uv1 = z1(1:2, 1:n_pts);
    uv2 = z2(1:2, 1:n_pts);
    uv_dot = (uv2 - uv1) / dt;  % [px/s]
    
    %% 6-DOF 최소자승
    % x = [ω_C; v_C]  (6×1, Camera frame)
    % ω_C: ω^C_{B/I} (I 기준 B가 회전, C frame 관측)
    % v_C: v^C_{B/L} (L 기준 B가 병진, C frame 관측)
    
    A = zeros(2*n_pts, 6);
    b = zeros(2*n_pts, 1);
    
    valid = 0;
    
    for j = 1:n_pts
        P_C = points_C(:, j);
        depth = P_C(3);
        
        if depth < 0.01; continue; end
        
        X = P_C(1);
        Y = P_C(2);
        Z = P_C(3);
        
        % 정규화 좌표
        x_n = X / Z;
        y_n = Y / Z;
        
        % 옵티컬 플로우 자코비안
        % u̇ = fx * [회전 항] + fx * [병진 항]
        % v̇ = fy * [회전 항] + fy * [병진 항]
        
        % 회전 항: -ω × [x_n; y_n; 1]의 x,y 성분
        J_rot_u = fx * [x_n*y_n, -(1+x_n^2), y_n];      % 1×3
        J_rot_v = fy * [1+y_n^2, -x_n*y_n, -x_n];       % 1×3
        
        % 병진 항: (1/Z)[I - p_n * p_n']의 x,y row
        J_trans_u = (fx/Z) * [1, 0, -x_n];              % 1×3
        J_trans_v = (fy/Z) * [0, 1, -y_n];              % 1×3
        
        valid = valid + 1;
        row = 2*(valid-1) + 1;
        
        A(row,   1:3) = J_rot_u;
        A(row,   4:6) = J_trans_u;
        A(row+1, 1:3) = J_rot_v;
        A(row+1, 4:6) = J_trans_v;
        
        b(row:row+1) = uv_dot(:, j);
    end
    
    if valid < 6; continue; end
    
    A = A(1:2*valid, :);
    b = b(1:2*valid);
    
    % SVD 체크
    [U, S, V] = svd(A' * A);
    s = diag(S);
    cond_num = s(1) / s(end);
    
    if cond_num > 1e12 || any(isnan(s))
        fprintf('  프레임 %d: 수치 불안정\n', i);
        continue;
    end
    
    % 최소자승 해
    if cond_num > 1e8
        lambda = 1e-8 * s(1);
        x_est = (A' * A + lambda * eye(6)) \ (A' * b);
    else
        x_est = (A' * A) \ (A' * b);
    end
    
    % 추정값 (Camera frame)
    omega_sIeB_observC_est = x_est(1:3);
    v_sLeB_observC_est = x_est(4:6);
    
    % Camera → Body/LVLH frame 변환
    omega_sIeB_observB_est = R_C2B * omega_sIeB_observC_est;
    v_sLeB_observL_est = R_C2L * v_sLeB_observC_est;
    
    % Ground truth (Camera frame)
    omega_sIeB_observC_true = R_B2C * omega_sIeB_observB_true;
    v_sLeB_observC_true = R_L2C * v_sLeB_observL_true;
    
    % 저장
    valid_count = valid_count + 1;
    results(valid_count).time = t1;
    results(valid_count).omega_sIeB_observB_est = omega_sIeB_observB_est;
    results(valid_count).omega_sIeB_observB_true = omega_sIeB_observB_true;
    results(valid_count).v_sLeB_observL_est = v_sLeB_observL_est;
    results(valid_count).v_sLeB_observL_true = v_sLeB_observL_true;
    results(valid_count).omega_error = omega_sIeB_observB_est - omega_sIeB_observB_true;
    results(valid_count).v_error = v_sLeB_observL_est - v_sLeB_observL_true;
    results(valid_count).omega_error_norm = norm(omega_sIeB_observB_est - omega_sIeB_observB_true);
    results(valid_count).v_error_norm = norm(v_sLeB_observL_est - v_sLeB_observL_true);
    results(valid_count).n_points = valid;
    results(valid_count).cond_num = cond_num;
    
    if mod(valid_count, 5) == 0
        fprintf('  프레임 %d: ω_err=%.4f deg/s, v_err=%.4f m/s, cond=%.2e\n', ...
            valid_count, results(valid_count).omega_error_norm*180/pi, ...
            results(valid_count).v_error_norm*1000, cond_num);
    end
end

fprintf('완료: %d개 프레임\n\n', valid_count);

%% 결과 분석
if valid_count > 0
    times = [results.time]' / 60;
    
    omega_est = [results.omega_sIeB_observB_est];
    omega_true = [results.omega_sIeB_observB_true];
    omega_err = [results.omega_error];
    omega_err_norm = [results.omega_error_norm] * 180/pi;
    
    v_est = [results.v_sLeB_observL_est];
    v_true = [results.v_sLeB_observL_true];
    v_err = [results.v_error];
    v_err_norm = [results.v_error_norm] * 1000;  % m/s
    
    fprintf('=== 각속도 추정 성능 (ω^B_{B/I}) ===\n');
    fprintf('RMS:  %.4f deg/s\n', rms(omega_err_norm));
    fprintf('평균: %.4f deg/s\n', mean(omega_err_norm));
    fprintf('최대: %.4f deg/s\n', max(omega_err_norm));
    
    fprintf('\n=== 병진속도 추정 성능 (v^L_{B/L}) ===\n');
    fprintf('RMS:  %.4f m/s\n', rms(v_err_norm));
    fprintf('평균: %.4f m/s\n', mean(v_err_norm));
    fprintf('최대: %.4f m/s\n', max(v_err_norm));
    
    %% 시각화
    figure('Position', [100, 100, 1600, 1000]);
    
    % 각속도 (Body frame)
    for ax = 1:3
        subplot(4, 3, ax);
        plot(times, omega_est(ax,:)*180/pi, 'r-', 'LineWidth', 2);
        hold on;
        plot(times, omega_true(ax,:)*180/pi, 'k--', 'LineWidth', 1.5);
        ylabel('[deg/s]'); grid on;
        title(sprintf('\\omega^B_{B/I,%s}', char('x'+ax-1)));
        if ax==1; legend('Est', 'True', 'Location', 'best'); end
    end
    
    % 각속도 에러
    for ax = 1:3
        subplot(4, 3, 3+ax);
        plot(times, omega_err(ax,:)*180/pi, 'LineWidth', 1.5);
        ylabel('[deg/s]'); grid on;
        title(sprintf('\\omega Error (%s)', char('x'+ax-1)));
    end
    
    % 병진속도 (LVLH frame)
    for ax = 1:3
        subplot(4, 3, 6+ax);
        plot(times, v_est(ax,:)*1000, 'b-', 'LineWidth', 2);
        hold on;
        plot(times, v_true(ax,:)*1000, 'k--', 'LineWidth', 1.5);
        ylabel('[m/s]'); grid on;
        title(sprintf('v^L_{B/L,%s}', char('x'+ax-1)));
        if ax==1; legend('Est', 'True', 'Location', 'best'); end
    end
    
    % 병진속도 에러
    for ax = 1:3
        subplot(4, 3, 9+ax);
        plot(times, v_err(ax,:)*1000, 'LineWidth', 1.5);
        xlabel('Time [min]'); ylabel('[m/s]'); grid on;
        title(sprintf('v Error (%s)', char('x'+ax-1)));
    end
    
    sgtitle('6-DOF Optical Flow Estimation', 'FontSize', 14, 'FontWeight', 'bold');
    
    %% 비교
    figure('Position', [150, 150, 1400, 500]);
    
    subplot(1, 3, 1);
    plot(times, omega_err_norm, 'r-', 'LineWidth', 2);
    ylabel('Error [deg/s]'); xlabel('Time [min]');
    title('\omega^B_{B/I} Error'); grid on;
    
    subplot(1, 3, 2);
    plot(times, v_err_norm, 'b-', 'LineWidth', 2);
    ylabel('Error [m/s]'); xlabel('Time [min]');
    title('v^L_{B/L} Error'); grid on;
    
    subplot(1, 3, 3);
    gyro_err = vecnorm(imu.gyro_meas - imu.gyro_true, 2, 1) * 180/pi;
    bar([rms(gyro_err), rms(omega_err_norm)]);
    set(gca, 'XTickLabel', {'IMU Gyro', '6-DOF OF'});
    ylabel('RMS Error [deg/s]');
    title('Sensor Comparison'); grid on;
    
    fprintf('\nIMU 자이로 RMS: %.6f deg/s\n', rms(gyro_err));
    fprintf('6-DOF OF RMS:   %.6f deg/s\n', rms(omega_err_norm));
    
else
    warning('추정 실패');
end

fprintf('\n완료.\n');