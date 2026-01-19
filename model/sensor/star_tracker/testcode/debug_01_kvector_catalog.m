%% debug_01_kvector_catalog.m
% k-vector 카탈로그 구축 및 검증
clear; close all; clc;

%% 설정
catalog_file = 'D:\star_tracker_test\main_pj_code\model\sensor\star_tracker\filtered_catalogue\Hipparcos_Below_6.0.csv';
FOV_deg = 14;
mag_threshold = 6.0;

%% 1. k-vector 카탈로그 구축
fprintf('=== k-vector 카탈로그 구축 테스트 ===\n\n');
catalog_data = build_star_catalog_kvector(catalog_file, FOV_deg, mag_threshold);

%% 2. 기본 정보 출력
fprintf('\n=== 카탈로그 정보 ===\n');
fprintf('총 별 개수: %d\n', catalog_data.N_stars);
fprintf('FOV 내 별쌍: %d\n', catalog_data.N_pairs);
fprintf('각거리 범위: %.4f ~ %.4f rad (%.2f ~ %.2f deg)\n', ...
    min(catalog_data.sorted_angles), max(catalog_data.sorted_angles), ...
    rad2deg(min(catalog_data.sorted_angles)), rad2deg(max(catalog_data.sorted_angles)));

%% 3. k-vector 분포 확인
figure('Name', 'k-vector Distribution', 'Position', [100, 100, 1200, 400]);

% k-vector vs index
subplot(1,3,1);
plot(1:catalog_data.N_pairs, catalog_data.k_vector, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Index');
ylabel('k-vector Value');
title('k-vector Distribution');

% 정렬된 각거리
subplot(1,3,2);
plot(1:catalog_data.N_pairs, rad2deg(catalog_data.sorted_angles), 'r-', 'LineWidth', 1);
grid on;
xlabel('Index');
ylabel('Angle (deg)');
title('Sorted Inter-star Angles');

% 각거리 히스토그램
subplot(1,3,3);
histogram(rad2deg(catalog_data.sorted_angles), 50, 'FaceColor', [0.2 0.6 0.8]);
grid on;
xlabel('Angle (deg)');
ylabel('Count');
title('Angle Distribution');

%% 4. k-vector 선형성 체크
expected_linear = linspace(0, catalog_data.N_pairs, catalog_data.N_pairs)';
deviation = catalog_data.k_vector - expected_linear;
max_deviation = max(abs(deviation));

fprintf('\n=== k-vector 선형성 검증 ===\n');
fprintf('최대 편차: %.2f (%.2f%%)\n', max_deviation, 100*max_deviation/catalog_data.N_pairs);

if max_deviation < 0.1 * catalog_data.N_pairs
    fprintf('✓ k-vector 선형성 양호\n');
else
    fprintf('✗ k-vector 비선형성 감지 - 검토 필요\n');
end

%% 5. k-vector 검색 성능 테스트
fprintf('\n=== k-vector 검색 성능 테스트 ===\n');

% 랜덤 각도 선택
test_angles = linspace(min(catalog_data.sorted_angles), ...
                       max(catalog_data.sorted_angles), 100);
sigma = 50e-6; % 50 microrad
k_multiplier = 6.4;
tolerance = k_multiplier * sigma;

search_times = zeros(length(test_angles), 1);
candidate_counts = zeros(length(test_angles), 1);

for i = 1:length(test_angles)
    tic;
    [candidates, ~, ~] = kvector_range_search(test_angles(i), tolerance, catalog_data);
    search_times(i) = toc;
    candidate_counts(i) = length(candidates);
end

fprintf('평균 검색 시간: %.6f ms\n', mean(search_times)*1000);
fprintf('평균 후보 개수: %.1f\n', mean(candidate_counts));
fprintf('최대 후보 개수: %d\n', max(candidate_counts));

%% 6. 특정 각도 검색 상세 테스트
fprintf('\n=== 특정 각도 검색 상세 ===\n');
test_angle = deg2rad(5.0); % 5도
fprintf('검색 각도: %.4f rad (%.2f deg)\n', test_angle, rad2deg(test_angle));
fprintf('허용오차: %.6f rad (%.4f arcsec)\n', tolerance, rad2deg(tolerance)*3600);

[candidates, k_start, k_end] = kvector_range_search(test_angle, tolerance, catalog_data);

fprintf('k_start: %d, k_end: %d\n', k_start, k_end);
fprintf('후보 개수: %d\n', length(candidates));

if ~isempty(candidates)
    angles_deg = rad2deg([candidates.angle]);
    fprintf('후보 각도 범위: %.4f ~ %.4f deg\n', min(angles_deg), max(angles_deg));
    
    % 처음 5개 출력
    fprintf('\n처음 5개 후보:\n');
    for i = 1:min(5, length(candidates))
        fprintf('  [%d] Star %d - %d: %.4f deg\n', ...
            i, candidates(i).I, candidates(i).J, rad2deg(candidates(i).angle));
    end
end

%% 7. 저장
save_file = 'star_catalog_kvector.mat';
save(save_file, 'catalog_data', '-v7.3');
fprintf('\n저장 완료: %s\n', save_file);
fprintf('파일 크기: %.2f MB\n', dir(save_file).bytes/1e6);

fprintf('\n=== 테스트 완료 ===\n');