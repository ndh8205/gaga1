%% build_and_save_catalog_once.m (NEW - 1회만 실행)
% 카탈로그 빌드 후 최적화된 형태로 저장
clear; close all; clc;

catalog_file = 'D:\star_tracker_test\main_pj_code\model\sensor\star_tracker\filtered_catalogue\Hipparcos_Below_6.0.csv';
FOV_deg = 14;
mag_threshold = 6.0;

fprintf('=== 카탈로그 최종 빌드 ===\n');

%% 1. k-vector 카탈로그 구축
catalog_data = build_star_catalog_kvector(catalog_file, FOV_deg, mag_threshold);

%% 2. 배열로 변환 (구조체보다 빠름)
% sorted_pairs → 배열
n_pairs = length(catalog_data.sorted_pairs);
pairs_I = zeros(n_pairs, 1, 'uint16');
pairs_J = zeros(n_pairs, 1, 'uint16');
pairs_angle = zeros(n_pairs, 1, 'single');

for i = 1:n_pairs
    pairs_I(i) = catalog_data.sorted_pairs(i).I;
    pairs_J(i) = catalog_data.sorted_pairs(i).J;
    pairs_angle(i) = catalog_data.sorted_pairs(i).angle;
end

%% 3. 최적화 구조체
catalog_fast = struct();
catalog_fast.N_stars = catalog_data.N_stars;
catalog_fast.N_pairs = catalog_data.N_pairs;
catalog_fast.FOV_rad = catalog_data.FOV_rad;

% 별 정보 (float32)
catalog_fast.star_RA = single(catalog_data.star_catalog.RA);
catalog_fast.star_DEC = single(catalog_data.star_catalog.DEC);
catalog_fast.star_Mag = single(catalog_data.star_catalog.Magnitude);
catalog_fast.star_ID = uint32(catalog_data.star_catalog.ID);

% 단위벡터 (float32)
catalog_fast.r_I = single(catalog_data.r_I);

% k-vector 데이터
catalog_fast.pairs_I = pairs_I;
catalog_fast.pairs_J = pairs_J;
catalog_fast.pairs_angle = pairs_angle;
catalog_fast.k_vector = uint32(catalog_data.k_vector);
catalog_fast.m = catalog_data.m;
catalog_fast.q = catalog_data.q;

%% 4. 저장 (v7.3 + 압축)
save_file = 'star_catalog_fast.mat';
save(save_file, 'catalog_fast', '-v7.3');

fprintf('\n저장 완료: %s\n', save_file);
fprintf('파일 크기: %.2f MB\n', dir(save_file).bytes/1e6);

% 원본과 비교
if exist('star_catalog_kvector.mat', 'file')
    old_size = dir('star_catalog_kvector.mat').bytes/1e6;
    fprintf('원본 크기: %.2f MB\n', old_size);
    fprintf('압축률: %.1f%%\n', 100*(1 - dir(save_file).bytes/dir('star_catalog_kvector.mat').bytes));
end