%% 이미지 로딩 테스트
clear; clc; close all;

% 경로 설정
img_dir = 'D:\slam_datasets\gco_set\img_set\1hz';

% 이미지 파일 목록 가져오기
img_files = dir(fullfile(img_dir, '*.png'));  % PNG 파일
if isempty(img_files)
    img_files = dir(fullfile(img_dir, '*.jpg'));  % JPG 파일
end

% 파일 개수 확인
fprintf('총 이미지 수: %d\n', length(img_files));

if isempty(img_files)
    error('이미지 파일을 찾을 수 없습니다.');
end

% 첫 몇 개 이미지 테스트
test_count = min(5, length(img_files));

figure('Position', [100, 100, 1500, 800]);
for i = 1:test_count
    % 이미지 읽기
    img_path = fullfile(img_dir, img_files(i).name);
    img = imread(img_path);
    
    % 이미지 정보
    fprintf('이미지 %d: %s\n', i, img_files(i).name);
    fprintf('  크기: %d x %d\n', size(img, 2), size(img, 1));
    fprintf('  채널: %d\n', size(img, 3));
    fprintf('  타입: %s\n', class(img));
    
    % 시각화
    subplot(2, 3, i);
    imshow(img);
    title(sprintf('Frame %d: %s', i, img_files(i).name), 'Interpreter', 'none');
end

% 전체 이미지 빠른 로딩 체크
fprintf('\n전체 이미지 로딩 테스트...\n');
tic;
for i = 1:length(img_files)
    img = imread(fullfile(img_dir, img_files(i).name));
    if mod(i, 100) == 0
        fprintf('  %d/%d 완료\n', i, length(img_files));
    end
end
elapsed = toc;
fprintf('전체 로딩 시간: %.2f초 (평균: %.4f초/이미지)\n', elapsed, elapsed/length(img_files));