clear; clc; close all;

% 경로 설정
img_dir = 'D:\slam_datasets\gco_set\img_set\1hz';

% 이미지 파일 목록
img_files = dir(fullfile(img_dir, '*.png'));

% 파일 개수 확인
total_images = length(img_files);
fprintf('총 이미지 수: %d\n', total_images);

% 1분(60프레임) 간격 샘플링
sampling_interval = 1;  % 일단 냅두기
sampled_indices = 1:sampling_interval:total_images;
num_sampled = length(sampled_indices);

fprintf('샘플링 간격: %d프레임 (1분)\n', sampling_interval);
fprintf('샘플링된 이미지 수: %d\n', num_sampled);
fprintf('시간 범위: 0 ~ %.1f분\n', (total_images-1)/60);

% 샘플링된 이미지 로드
sampled_images = cell(num_sampled, 1);
sampled_filenames = cell(num_sampled, 1);

fprintf('\n이미지 로딩 중...\n');
for i = 1:num_sampled
    idx = sampled_indices(i);
    img_path = fullfile(img_dir, img_files(idx).name);
    sampled_images{i} = imread(img_path);
    sampled_filenames{i} = img_files(idx).name;
    
    if mod(i, 10) == 0
        fprintf('  %d/%d 완료 (프레임 %d, 시간: %.1f분)\n', ...
            i, num_sampled, idx, (idx-1)/60);
    end
end

% 시각화 (처음 9개)
figure('Position', [100, 100, 1500, 1000]);
display_count = min(9, num_sampled);
for i = 1:display_count
    subplot(3, 3, i);
    imshow(sampled_images{i});
    time_min = (sampled_indices(i)-1) / 60;
    title(sprintf('t=%.1f분\n%s', time_min, sampled_filenames{i}), ...
        'Interpreter', 'none', 'FontSize', 8);
end