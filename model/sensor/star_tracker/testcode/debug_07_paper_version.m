%% debug_07_paper_version.m
% Paper 버전 전체 테스트
clear; close all; clc;

fprintf('=== Paper 버전 테스트 ===\n\n');

%% 1. 빠른 카탈로그 로드
if ~exist('star_catalog_fast.mat', 'file')
    error('먼저 build_and_save_catalog_once.m 실행');
end

tic;
load('star_catalog_fast.mat');
t_load = toc;
fprintf('카탈로그 로드: %.2f ms\n\n', t_load*1000);

%% 2. 센서 파라미터
sensor_params.f = 0.01042;
sensor_params.myu = 2e-6;
sensor_params.l = 1280;
sensor_params.w = 720;
sensor_params.sigma = 50e-6;
sensor_params.k_multiplier = 6.4;

%% 3. 데이터 로드
data_dir = fullfile(pwd, 'sim_data');
star_data_dir = fullfile(data_dir, 'star_data');

% sim_data 로드
sim_files = dir(fullfile(data_dir, 'sim_data_ver2_*.mat'));
if isempty(sim_files)
    error('sim_data 파일 없음');
end
[~, latest_sim_idx] = max([sim_files.datenum]);
load(fullfile(data_dir, sim_files(latest_sim_idx).name));
fprintf('로드 (sim): %s\n', sim_files(latest_sim_idx).name);

% star_data 로드
star_files = dir(fullfile(star_data_dir, 'star_data_occluded_*.mat'));
if isempty(star_files)
    error('star_data 파일 없음');
end
[~, latest_star_idx] = max([star_files.datenum]);
load(fullfile(star_data_dir, star_files(latest_star_idx).name));
fprintf('로드 (star): %s\n', star_files(latest_star_idx).name);
fprintf('총 프레임: %d\n\n', length(star_data_all));

%% 4. 별 많은 프레임 선택
num_stars = [star_data_all.num_stars];
[~, sorted_idx] = sort(num_stars, 'descend');

n_test_frames = min(10, sum(num_stars >= 4));
test_frames = sorted_idx(1:n_test_frames);

fprintf('테스트 프레임: %d개 (별 >= 4개)\n\n', n_test_frames);

%% 5. 통계 변수
success_count = 0;
attitude_errors = [];
execution_times = [];

results = struct('frame', {}, 'n_stars', {}, 'success', {}, ...
                 'n_identified', {}, 'att_error', {}, 'exec_time', {});

%% 6. 각 프레임 테스트
R_B2ST = [1, 0, 0; 0, 0, 1; 0,-1, 0];
R_ST2B = R_B2ST';

for test_idx = 1:n_test_frames
    k = test_frames(test_idx);
    frame_data = star_data_all(k);
    
    fprintf('--- Frame %d/%d (총 별: %d개) ---\n', test_idx, n_test_frames, frame_data.num_stars);
    
    pixel_coords = frame_data.pixel_coords;
    magnitudes = frame_data.magnitudes;
    frame_idx = frame_data.frame;
    
    % Ground truth 자세
    q_true = quat.q_I2B(:, frame_idx);
    R_I2B_true = GetDCM_QUAT(q_true);
    
    % Paper 버전 실행
    tic;
    [star_ids, attitude, pyramid_stats] = pyramid_star_id_paper(...
        pixel_coords, magnitudes, catalog_fast, sensor_params);
    exec_time = toc;
    
    % 결과 저장
    results(test_idx).frame = k;
    results(test_idx).n_stars = frame_data.num_stars;
    results(test_idx).success = pyramid_stats.success;
    results(test_idx).exec_time = exec_time;
    
    if pyramid_stats.success
        success_count = success_count + 1;
        execution_times = [execution_times; exec_time];
        
        results(test_idx).n_identified = pyramid_stats.n_identified;
        
        % 자세 오차
        if ~isempty(attitude)
            R_I2ST_est = attitude.R;
            R_I2B_est = R_ST2B * R_I2ST_est;
            
            R_err = R_I2B_est * R_I2B_true';
            q_err = DCM2Quat(R_err);
            angle_err = 2 * acos(min(1, abs(q_err(1))));
            
            results(test_idx).att_error = rad2deg(angle_err);
            attitude_errors = [attitude_errors; rad2deg(angle_err)];
            
            fprintf('  ✓ 성공: %d개 식별, 자세오차 %.4f°, %.2f ms\n', ...
                pyramid_stats.n_identified, rad2deg(angle_err), exec_time*1000);
        else
            results(test_idx).att_error = NaN;
            fprintf('  ✓ 식별 성공: %d개, %.2f ms\n', ...
                pyramid_stats.n_identified, exec_time*1000);
        end
    else
        results(test_idx).n_identified = 0;
        results(test_idx).att_error = NaN;
        fprintf('  ✗ 실패: %.2f ms\n', exec_time*1000);
    end
end

%% 7. 통계 요약
fprintf('\n=== 전체 통계 ===\n');
fprintf('성공률: %d/%d (%.1f%%)\n', success_count, n_test_frames, 100*success_count/n_test_frames);

if ~isempty(execution_times)
    fprintf('실행시간: %.2f ± %.2f ms\n', mean(execution_times)*1000, std(execution_times)*1000);
end

if ~isempty(attitude_errors)
    fprintf('자세 오차: %.4f ± %.4f° (%.2f ~ %.2f°)\n', ...
        mean(attitude_errors), std(attitude_errors), ...
        min(attitude_errors), max(attitude_errors));
    
    high_precision = sum(attitude_errors < 0.1);
    medium_precision = sum(attitude_errors >= 0.1 & attitude_errors < 1.0);
    low_precision = sum(attitude_errors >= 1.0);
    
    fprintf('\n자세 정밀도 분포:\n');
    fprintf('  고정밀 (<0.1°): %d (%.1f%%)\n', high_precision, 100*high_precision/length(attitude_errors));
    fprintf('  중정밀 (0.1~1°): %d (%.1f%%)\n', medium_precision, 100*medium_precision/length(attitude_errors));
    fprintf('  저정밀 (>1°): %d (%.1f%%)\n', low_precision, 100*low_precision/length(attitude_errors));
end

%% 8. 시각화
if success_count > 0
    figure('Name', 'Paper Version Performance', 'Position', [100, 100, 1200, 800]);
    
    subplot(2,2,1);
    if ~isempty(attitude_errors)
        histogram(attitude_errors*60, 20, 'FaceColor', [0.2 0.6 0.8]);
        grid on;
        xlabel('Attitude Error (arcmin)');
        ylabel('Count');
        title(sprintf('Attitude Error (mean: %.2f arcmin)', mean(attitude_errors)*60));
    end
    
    subplot(2,2,2);
    histogram(execution_times*1000, 20, 'FaceColor', [0.8 0.4 0.2]);
    grid on;
    xlabel('Execution Time (ms)');
    ylabel('Count');
    title(sprintf('Execution Time (mean: %.2f ms)', mean(execution_times)*1000));
    
    subplot(2,2,3);
    if ~isempty(attitude_errors)
        n_stars_success = [results([results.success]).n_stars];
        scatter(n_stars_success, attitude_errors*60, 50, 'filled');
        grid on;
        xlabel('Number of Stars');
        ylabel('Attitude Error (arcmin)');
        title('Error vs Star Count');
    end
    
    subplot(2,2,4);
    scatter(n_stars_success, execution_times*1000, 50, 'filled');
    grid on;
    xlabel('Number of Stars');
    ylabel('Execution Time (ms)');
    title('Runtime vs Star Count');
end

fprintf('\n=== 테스트 완료 ===\n');

