%% test_pyramid_full.m
% pyramid_star_id_liebe2002 전체 테스트
clear; close all; clc;

fprintf('=== Pyramid 알고리즘 전체 테스트 ===\n\n');

%% 1. 데이터 로드
data_dir = fullfile(pwd, 'sim_data');
star_data_noise_dir = fullfile(data_dir, 'star_data', 'noise');

sim_files = dir(fullfile(data_dir, 'sim_data_lqr_*.mat'));
[~, latest_sim_idx] = max([sim_files.datenum]);
load(fullfile(data_dir, sim_files(latest_sim_idx).name));

star_files = dir(fullfile(star_data_noise_dir, 'star_data_noise*.mat'));
[~, latest_star_idx] = max([star_files.datenum]);
load(fullfile(star_data_noise_dir, star_files(latest_star_idx).name));

load('star_catalog_fast.mat');

fprintf('[로드 완료]\n');
fprintf('  총 프레임: %d\n', length(star_data_noise_all));
fprintf('  카탈로그: %d stars\n\n', catalog_fast.N_stars);

%% 2. 센서 파라미터
sensor_params.f = 0.01042;
sensor_params.myu = 2e-6;
sensor_params.l = 1280;
sensor_params.w = 720;
sensor_params.sigma = 50e-6;
sensor_params.k_multiplier = 6.4;

%% 3. 테스트 설정
n_test_frames = min(200, length(star_data_noise_all));
test_frames = randsample(length(star_data_noise_all), n_test_frames);

fprintf('[테스트 설정]\n');
fprintf('  테스트 프레임: %d개\n\n', n_test_frames);

%% 4. 통계 변수
stats.total_frames = 0;
stats.success = 0;
stats.failure = 0;
stats.n_stars_obs = [];
stats.n_stars_identified = [];
stats.execution_times = [];
stats.failed_frames = [];

%% 5. 프레임별 테스트
fprintf('=== 테스트 진행 ===\n');
for f_idx = 1:n_test_frames
    frame = test_frames(f_idx);
    frame_data = star_data_noise_all(frame);
    
    if mod(f_idx, 50) == 0
        fprintf('  진행: %d/%d\n', f_idx, n_test_frames);
    end
    
    % 별 4개 미만 스킵
    if frame_data.num_stars < 4
        continue;
    end
    
    stats.total_frames = stats.total_frames + 1;
    stats.n_stars_obs = [stats.n_stars_obs; frame_data.num_stars];
    
    % Pyramid 실행
    tic;
    [star_ids, attitude, result] = pyramid_star_id_liebe2002(...
        frame_data.pixel_coords, ...
        frame_data.magnitudes, ...
        catalog_fast, ...
        sensor_params);
    exec_time = toc;
    
    stats.execution_times = [stats.execution_times; exec_time];
    
    if result.success
        stats.success = stats.success + 1;
        stats.n_stars_identified = [stats.n_stars_identified; result.n_identified];
        
        % Ground truth와 비교
        gt_ids = frame_data.star_ids;
        identified_correct = sum(ismember(star_ids, gt_ids));
        
    else
        stats.failure = stats.failure + 1;
        stats.failed_frames = [stats.failed_frames; frame];
    end
end

%% 6. 결과 리포트
fprintf('\n=== 테스트 결과 ===\n');
fprintf('총 프레임: %d개\n', stats.total_frames);
fprintf('성공: %d (%.1f%%)\n', stats.success, 100*stats.success/stats.total_frames);
fprintf('실패: %d (%.1f%%)\n\n', stats.failure, 100*stats.failure/stats.total_frames);

if ~isempty(stats.n_stars_identified)
    fprintf('[식별 통계]\n');
    fprintf('관측 별 수: %.1f (평균), %d (중간), %d (최대)\n', ...
        mean(stats.n_stars_obs), median(stats.n_stars_obs), max(stats.n_stars_obs));
    fprintf('식별 별 수: %.1f (평균), %d (중간), %d (최대)\n\n', ...
        mean(stats.n_stars_identified), median(stats.n_stars_identified), max(stats.n_stars_identified));
end

fprintf('[실행시간]\n');
fprintf('평균: %.1f ms\n', mean(stats.execution_times)*1000);
fprintf('중간: %.1f ms\n', median(stats.execution_times)*1000);
fprintf('최대: %.1f ms\n\n', max(stats.execution_times)*1000);

if ~isempty(stats.failed_frames)
    fprintf('[실패 프레임 (최대 10개)]\n');
    for i = 1:min(10, length(stats.failed_frames))
        frame = stats.failed_frames(i);
        fprintf('  Frame %d: %d stars\n', frame, star_data_noise_all(frame).num_stars);
    end
end

%% 7. 시각화
figure('Position', [100, 100, 1200, 400]);

subplot(1,3,1);
histogram(stats.n_stars_obs, 15);
xlabel('관측 별 수');
ylabel('빈도');
title('관측 별 분포');
grid on;

subplot(1,3,2);
if ~isempty(stats.n_stars_identified)
    histogram(stats.n_stars_identified, 15);
    xlabel('식별 별 수');
    ylabel('빈도');
    title('식별 별 분포');
    grid on;
end

subplot(1,3,3);
histogram(stats.execution_times*1000, 30);
xlabel('실행시간 (ms)');
ylabel('빈도');
title('실행시간 분포');
grid on;

fprintf('\n=== 테스트 완료 ===\n');