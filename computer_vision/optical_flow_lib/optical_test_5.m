clc; clear all; close all;
addpath(genpath('C:\Users\USER\Desktop\relative2'));

%% 데이터 로드
data_dir = fullfile(pwd, 'sim_data');
files = dir(fullfile(data_dir, 'sim_data_ver2_*.mat'));
latest_file = fullfile(data_dir, files(end).name);
load(latest_file);

fprintf('=== 6-DOF 옵티컬 플로우 [필터 방식 정합] ===\n\n');

%% 카메라 파라미터
K = camera.params.intrinsic.K_ideal;
R_B2C = [1, 0, 0; 0, 0, -1; 0, 1, 0];
R_C2B = R_B2C';

fx = K(1,1);
fy = K(2,2);
cx = K(1,3);
cy = K(2,3);

fprintf('카메라: fx=%.2f, fy=%.2f\n', fx, fy);

%% Point Cloud
points_A = camera.pointcloud;
N_total = size(points_A, 2);
fprintf('특징점: %d개\n\n', N_total);

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
    
    %% 공통 가시 특징점 추출
    vis1 = logical(z1(3, :));
    vis2 = logical(z2(3, :));
    common_vis = vis1 & vis2;
    valid_idx = find(common_vis);
    
    n_pts = length(valid_idx);
    if n_pts < 6
        fprintf('  프레임 %d: 공통 특징점 부족 (%d개)\n', i, n_pts);
        continue;
    end
    
    % 매칭된 픽셀과 3D 포인트
    uv1 = z1(1:2, valid_idx);
    uv2 = z2(1:2, valid_idx);
    points_matched = points_A(:, valid_idx);
    
    % 시간 인덱스
    [~, idx1] = min(abs(time.t_s - t1));
    [~, idx2] = min(abs(time.t_s - t2));
    
    %% === Frame 1 변환 (필터 방식) ===
    q_I2A_1 = quat.q_I2A(:, idx1);
    q_I2B_1 = quat.q_I2B(:, idx1);
    q_L2B_1 = quat.q_L2B(:, idx1);
    
    % q_B2A = q_I2A * inv(q_I2B)
    q_B2A_1 = q2q_mult(q_I2A_1, inv_q(q_I2B_1));
    q_B2A_1 = q_B2A_1 / norm(q_B2A_1);
    
    R_B2A_1 = GetDCM_QUAT(q_B2A_1);
    R_A2B_1 = R_B2A_1';
    
    % t_A_in_B = -R_L2B * r_B_L
    R_L2B_1 = GetDCM_QUAT(q_L2B_1);
    r_B_L_1 = traj.r_B_L(:, idx1);
    t_A_in_B_1 = -R_L2B_1 * r_B_L_1;
    
    %% 특징점 투영 (Frame 1)
    points_C = zeros(3, n_pts);
    points_B = zeros(3, n_pts);  % 중간 단계 저장
    
    for j = 1:n_pts
        p_A = points_matched(:, j);
        p_B = R_A2B_1 * p_A;
        p_C = R_B2C * (p_B + t_A_in_B_1);
        
        points_B(:, j) = p_B;
        points_C(:, j) = p_C;
    end
    
    %% 옵티컬 플로우
    uv_dot = (uv2 - uv1) / dt;
    
    %% === Ground Truth 속도 (Frame 1, Body frame) ===
    
    % (1) ω^B_{A/B}: Deputy body에서 본 Chief의 각속도 (Deputy 대비)
    omega_sBeA_observB_true = quat.omega_sBeA_observB(:, idx1);
    omega_A2B_B = -omega_sBeA_observB_true;  % ω_A/B = -ω_B/A
    
    % (2) dt_A_in_B/dt 계산
    % t_A_in_B = -R_L2B * r_B_L
    % dt_A_in_B/dt = -d/dt(R_L2B * r_B_L)
    %              = -[ω_L2B^B ×](R_L2B * r_B_L) - R_L2B * v_B_L
    
    omega_sLeB_observL = quat.omega_sLeB_observL(:, idx1);
    omega_L2B_B = R_L2B_1 * omega_sLeB_observL;
    
    v_B_L_1 = traj.v_B_L(:, idx1);
    
    dt_A_in_B = -cross(omega_L2B_B, R_L2B_1 * r_B_L_1) - R_L2B_1 * v_B_L_1;
    
    %% === Ground Truth: Camera frame 속도 ===
    
    % v_P^C = R_B2C * ([ω_A2B^B ×] p_B + dt_A_in_B/dt)
    % 여기서 p_B = R_A2B * p_A + t_A_in_B
    
    % 하지만 옵티컬 플로우는 상대 운동만 필요:
    % ω_C/A^C = R_B2C * ω_A/B^B = R_B2C * (-ω_B/A^B)
    omega_C2A_C_true = R_B2C * omega_A2B_B;
    
    % v_A_in_C = R_B2C * dt_A_in_B/dt
    v_A_in_C_true = R_B2C * dt_A_in_B;
    
    %% 이론 검증: 옵티컬 플로우
    uv_dot_theory = zeros(2, n_pts);
    
    for j = 1:n_pts
        P_C = points_C(:, j);
        
        if P_C(3) < 0.01
            continue;
        end
        
        X = P_C(1);
        Y = P_C(2);
        Z = P_C(3);
        
        x_n = X / Z;
        y_n = Y / Z;
        
        % v_P^C 계산
        p_B = points_B(:, j);
        v_P_C = R_B2C * (cross(omega_A2B_B, p_B + t_A_in_B_1) + dt_A_in_B);
        
        % 옵티컬 플로우
        u_dot_th = fx * (v_P_C(1)/Z - x_n*v_P_C(3)/Z);
        v_dot_th = fy * (v_P_C(2)/Z - y_n*v_P_C(3)/Z);
        
        uv_dot_theory(:, j) = [u_dot_th; v_dot_th];
    end
    
    %% 6-DOF 최소자승
    A = zeros(2*n_pts, 6);
    b = zeros(2*n_pts, 1);
    
    valid = 0;
    
    for j = 1:n_pts
        P_C = points_C(:, j);
        depth = P_C(3);
        
        if depth < 0.01
            continue;
        end
        
        X = P_C(1);
        Y = P_C(2);
        Z = P_C(3);
        
        x_n = X / Z;
        y_n = Y / Z;
        
        % 옵티컬 플로우 자코비안
        % v_P^C = [ω_C/A^C ×] P_C + v_A_in_C
        
        J_rot_u = -fx * [-x_n*y_n, (1+x_n^2), -y_n];
        J_rot_v = -fy * [-(1+y_n^2), x_n*y_n, x_n];
        
        J_trans_u = (fx/Z) * [1, 0, -x_n];
        J_trans_v = (fy/Z) * [0, 1, -y_n];
        
        valid = valid + 1;
        row = 2*(valid-1) + 1;
        
        A(row,   1:3) = J_rot_u;
        A(row,   4:6) = J_trans_u;
        A(row+1, 1:3) = J_rot_v;
        A(row+1, 4:6) = J_trans_v;
        
        b(row:row+1) = uv_dot(:, j);
    end
    
    if valid < 6
        continue;
    end
    
    A = A(1:2*valid, :);
    b = b(1:2*valid);
    
    % SVD 체크
    [U, S, V] = svd(A' * A);
    s = diag(S);
    cond_num = s(1) / s(end);
    
    if cond_num > 1e10 || any(isnan(s))
        fprintf('  프레임 %d: 수치 불안정 (cond=%.2e)\n', i, cond_num);
        continue;
    end
    
    % 최소자승 해
    if cond_num > 1e6
        lambda = 1e-6 * s(1);
        x_est = (A' * A + lambda * eye(6)) \ (A' * b);
    else
        x_est = (A' * A) \ (A' * b);
    end
    
    % 추정값 (Camera frame)
    omega_C2A_C_est = x_est(1:3);
    v_A_in_C_est = x_est(4:6);
    
    % Camera → Body frame
    omega_A2B_B_est = R_C2B * omega_C2A_C_est;
    omega_sBeA_observB_est = -omega_A2B_B_est;
    
    % v_A_in_C → v^L_{B/L}
    % v_A_in_C = R_B2C * dt_A_in_B/dt
    %          = R_B2C * (-[ω_L2B^B ×](R_L2B * r_B_L) - R_L2B * v_B_L)
    
    dt_A_in_B_est = R_C2B * v_A_in_C_est;
    
    % 역산: dt_A_in_B = -[ω_L2B^B ×](R_L2B * r_B_L) - R_L2B * v_B_L
    % → R_L2B * v_B_L = -dt_A_in_B - [ω_L2B^B ×](R_L2B * r_B_L)
    
    R_L2B_r = R_L2B_1 * r_B_L_1;
    v_B_L_est = R_L2B_1' * (-dt_A_in_B_est - cross(omega_L2B_B, R_L2B_r));
    v_sBeA_observL_est = -v_B_L_est;
    
    % Ground Truth (LVLH)
    v_sBeA_observL_true = -v_B_L_1;
    
    % 저장
    valid_count = valid_count + 1;
    results(valid_count).time = t1;
    results(valid_count).n_matched = n_pts;
    results(valid_count).omega_sBeA_observB_est = omega_sBeA_observB_est;
    results(valid_count).omega_sBeA_observB_true = omega_sBeA_observB_true;
    results(valid_count).v_sBeA_observL_est = v_sBeA_observL_est;
    results(valid_count).v_sBeA_observL_true = v_sBeA_observL_true;
    results(valid_count).omega_error = omega_sBeA_observB_est - omega_sBeA_observB_true;
    results(valid_count).v_error = v_sBeA_observL_est - v_sBeA_observL_true;
    results(valid_count).omega_error_norm = norm(omega_sBeA_observB_est - omega_sBeA_observB_true);
    results(valid_count).v_error_norm = norm(v_sBeA_observL_est - v_sBeA_observL_true);
    results(valid_count).n_points = valid;
    results(valid_count).cond_num = cond_num;
    
    % 첫 프레임 검증
    if valid_count == 1
        fprintf('=== 첫 프레임 검증 ===\n');
        fprintf('매칭된 특징점: %d개\n\n', n_pts);
        
        fprintf('[Ground Truth]\n');
        fprintf('ω^B_{A/B} = [%.6f, %.6f, %.6f] deg/s\n', omega_sBeA_observB_true*180/pi);
        fprintf('v^L_{A/B} = [%.6f, %.6f, %.6f] m/s\n', v_sBeA_observL_true*1000);
        
        fprintf('\n[Estimated]\n');
        fprintf('ω^B_{A/B} = [%.6f, %.6f, %.6f] deg/s\n', omega_sBeA_observB_est*180/pi);
        fprintf('v^L_{A/B} = [%.6f, %.6f, %.6f] m/s\n', v_sBeA_observL_est*1000);
        
        % 픽셀 속도 검증
        uv_diff = uv_dot - uv_dot_theory;
        uv_dot_meas_mag = vecnorm(uv_dot, 2, 1);
        uv_dot_th_mag = vecnorm(uv_dot_theory, 2, 1);
        uv_diff_mag = vecnorm(uv_diff, 2, 1);
        
        fprintf('\n[픽셀 속도 검증]\n');
        fprintf('측정 평균:   %.4f px/s\n', mean(uv_dot_meas_mag));
        fprintf('이론 평균:   %.4f px/s\n', mean(uv_dot_th_mag));
        fprintf('오차 평균:   %.4f px/s\n', mean(uv_diff_mag));
        fprintf('상대 오차:   %.2f%%\n\n', 100*mean(uv_diff_mag)/mean(uv_dot_meas_mag));
    end
    
    if mod(valid_count, 5) == 0
        fprintf('  프레임 %d: n=%d, ω_err=%.4f deg/s, v_err=%.4f m/s\n', ...
            valid_count, n_pts, results(valid_count).omega_error_norm*180/pi, ...
            results(valid_count).v_error_norm*1000);
    end
end

fprintf('완료: %d개 프레임\n\n', valid_count);

%% 결과 분석
if valid_count > 0
    times = [results.time]' / 60;
    
    omega_est = [results.omega_sBeA_observB_est];
    omega_true = [results.omega_sBeA_observB_true];
    omega_err = [results.omega_error];
    omega_err_norm = [results.omega_error_norm] * 180/pi;
    
    v_est = [results.v_sBeA_observL_est];
    v_true = [results.v_sBeA_observL_true];
    v_err = [results.v_error];
    v_err_norm = [results.v_error_norm] * 1000;
    
    fprintf('=== 각속도 추정 성능 (ω^B_{A/B}) ===\n');
    fprintf('RMS:  %.4f deg/s\n', rms(omega_err_norm));
    fprintf('평균: %.4f deg/s\n', mean(omega_err_norm));
    fprintf('최대: %.4f deg/s\n', max(omega_err_norm));
    fprintf('STD:  %.4f deg/s\n', std(omega_err_norm));
    
    fprintf('\n=== 병진속도 추정 성능 (v^L_{A/B}) ===\n');
    fprintf('RMS:  %.4f m/s\n', rms(v_err_norm));
    fprintf('평균: %.4f m/s\n', mean(v_err_norm));
    fprintf('최대: %.4f m/s\n', max(v_err_norm));
    fprintf('STD:  %.4f m/s\n', std(v_err_norm));
    
    % 시각화
    figure('Position', [100, 100, 1600, 1000]);
    
    for ax = 1:3
        subplot(4, 3, ax);
        plot(times, omega_est(ax,:)*180/pi, 'r-', 'LineWidth', 2);
        hold on;
        plot(times, omega_true(ax,:)*180/pi, 'k--', 'LineWidth', 1.5);
        ylabel('[deg/s]'); grid on;
        title(sprintf('\\omega^B_{A/B,%s}', char('x'+ax-1)));
        if ax==1; legend('Est', 'True', 'Location', 'best'); end
    end
    
    for ax = 1:3
        subplot(4, 3, 3+ax);
        plot(times, omega_err(ax,:)*180/pi, 'LineWidth', 1.5);
        ylabel('[deg/s]'); grid on;
        title(sprintf('\\omega Error (%s)', char('x'+ax-1)));
    end
    
    for ax = 1:3
        subplot(4, 3, 6+ax);
        plot(times, v_est(ax,:)*1000, 'b-', 'LineWidth', 2);
        hold on;
        plot(times, v_true(ax,:)*1000, 'k--', 'LineWidth', 1.5);
        ylabel('[m/s]'); grid on;
        title(sprintf('v^L_{A/B,%s}', char('x'+ax-1)));
        if ax==1; legend('Est', 'True', 'Location', 'best'); end
    end
    
    for ax = 1:3
        subplot(4, 3, 9+ax);
        plot(times, v_err(ax,:)*1000, 'LineWidth', 1.5);
        xlabel('Time [min]'); ylabel('[m/s]'); grid on;
        title(sprintf('v Error (%s)', char('x'+ax-1)));
    end
    
    sgtitle('6-DOF Optical Flow (Filter-Aligned)', 'FontSize', 14, 'FontWeight', 'bold');
    
    omega_rel_true_mag = vecnorm(omega_true, 2, 1) * 180/pi;
    v_rel_true_mag = vecnorm(v_true, 2, 1) * 1000;
    
    fprintf('\n=== Ground Truth 통계 ===\n');
    fprintf('상대 각속도: %.6f deg/s (평균)\n', mean(omega_rel_true_mag));
    fprintf('상대 병진속도: %.6f m/s (평균)\n', mean(v_rel_true_mag));
    fprintf('\n=== 상대 오차 ===\n');
    fprintf('각속도: %.2f%%\n', 100*rms(omega_err_norm)/mean(omega_rel_true_mag));
    fprintf('병진속도: %.2f%%\n', 100*rms(v_err_norm)/mean(v_rel_true_mag));
    
else
    warning('추정 실패');
end

fprintf('\n완료.\n');