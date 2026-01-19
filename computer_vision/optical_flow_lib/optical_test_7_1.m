clear; clc; close all;

%% 1. 이미지 로드
img_dir = 'D:\slam_datasets\gco_set\img_set\1hz';
img_files = dir(fullfile(img_dir, '*.png'));
total_images = length(img_files);

% 파라미터
params.max_features = 200;        % 최대 특징점 수
params.quality_level = 0.01;      % Shi-Tomasi 품질
params.min_distance = 30;         % 특징점 간 최소 거리
params.filter_size = 7;           % 코너 검출 필터 크기
params.window_size = [21, 21];    % LK 윈도우 크기
params.max_iterations = 30;       % LK 반복 횟수

%% 2. 첫 프레임 특징점 검출
img1 = imread(fullfile(img_dir, img_files(1).name));
if size(img1, 3) == 3
    gray1 = rgb2gray(img1);
else
    gray1 = img1;
end

% Shi-Tomasi 코너 검출
corners = detectMinEigenFeatures(gray1, ...
    'MinQuality', params.quality_level, ...
    'FilterSize', params.filter_size);

% 특징점 선택
points = corners.selectStrongest(params.max_features);
points_prev = points.Location;
num_features = size(points_prev, 1);

fprintf('초기 특징점 개수: %d\n', num_features);

% 추적 결과 저장
tracks = cell(total_images, 1);
tracks{1} = points_prev;

%% 3. KLT 추적 메인 루프
figure('Position', [100, 100, 1200, 800]);
point_tracker = vision.PointTracker('MaxIterations', params.max_iterations, ...
    'BlockSize', params.window_size, ...
    'NumPyramidLevels', 3, ...
    'MaxBidirectionalError', 1);

initialize(point_tracker, points_prev, gray1);
gray_prev = gray1;

for k = 2:min(100, total_images)
    % 현재 프레임 로드
    img2 = imread(fullfile(img_dir, img_files(k).name));
    if size(img2, 3) == 3
        gray2 = rgb2gray(img2);
    else
        gray2 = img2;
    end
    
    % KLT 추적
    [points_curr, validity, scores] = point_tracker(gray2);
    
    % 추적 전 특징점 저장 (flow 계산용)
    points_before = points_prev;
    
    % 유효한 특징점만 유지
    valid_idx = validity & scores > 0.8;
    points_curr_valid = points_curr(valid_idx, :);
    
    % 특징점 보충
    if sum(valid_idx) < params.max_features * 0.5
        % 새로운 특징점 검출
        new_corners = detectMinEigenFeatures(gray2, ...
            'MinQuality', params.quality_level, ...
            'FilterSize', params.filter_size);
        
        if ~isempty(new_corners)
            new_points = new_corners.selectStrongest(params.max_features * 2);
            new_points_location = new_points.Location;
            
            % 기존 특징점과의 거리 체크
            keep_new = true(size(new_points_location, 1), 1);
            for i = 1:size(new_points_location, 1)
                if ~isempty(points_curr_valid)
                    dists = sqrt(sum((points_curr_valid - new_points_location(i,:)).^2, 2));
                    if any(dists < params.min_distance)
                        keep_new(i) = false;
                    end
                end
            end
            
            new_points_filtered = new_points_location(keep_new, :);
            num_new = min(params.max_features - size(points_curr_valid, 1), ...
                         size(new_points_filtered, 1));
            
            if num_new > 0
                points_curr_valid = [points_curr_valid; new_points_filtered(1:num_new, :)];
                
                % Tracker 재초기화
                release(point_tracker);
                initialize(point_tracker, points_curr_valid, gray2);
            end
        end
    else
        % 특징점 보충 없이 tracker 업데이트
        setPoints(point_tracker, points_curr_valid);
    end
    
    % 저장 및 업데이트
    tracks{k} = points_curr_valid;
    points_prev = points_curr_valid;
    gray_prev = gray2;
    
    % 시각화
    if mod(k, 10) == 0 || k == 2
        clf;
        imshow(img2); hold on;
        
        % 현재 특징점
        plot(points_curr_valid(:,1), points_curr_valid(:,2), 'g+', ...
            'MarkerSize', 10, 'LineWidth', 2);
        
        % Optical flow 벡터 (유효한 점들만)
        if sum(valid_idx) > 0
            points_before_valid = points_before(valid_idx, :);
            flow = points_curr(valid_idx, :) - points_before_valid;
            
            % 큰 움직임만 표시
            flow_mag = sqrt(sum(flow.^2, 2));
            show_flow = flow_mag > 0.5;
            
            if any(show_flow)
                quiver(points_before_valid(show_flow,1), points_before_valid(show_flow,2), ...
                       flow(show_flow,1), flow(show_flow,2), 0, 'y', 'LineWidth', 1.5);
            end
        end
        
        title(sprintf('Frame %d/%d | Features: %d | Valid: %d', ...
            k, total_images, size(points_curr_valid, 1), sum(valid_idx)));
        drawnow;
    end
    
    if mod(k, 20) == 0
        fprintf('프레임 %d/%d 처리 완료 | 특징점: %d\n', ...
            k, min(100, total_images), size(points_curr_valid, 1));
    end
end

%% 4. 추적 통계
track_lengths = zeros(min(100, total_images), 1);
for k = 1:length(track_lengths)
    if ~isempty(tracks{k})
        track_lengths(k) = size(tracks{k}, 1);
    end
end

figure;
plot(track_lengths, 'b-', 'LineWidth', 2);
xlabel('Frame'); ylabel('특징점 개수');
title('KLT 추적 특징점 수 변화');
grid on;

fprintf('\n=== KLT 추적 완료 ===\n');
fprintf('평균 특징점 수: %.1f\n', mean(track_lengths(track_lengths>0)));
fprintf('최소 특징점 수: %d\n', min(track_lengths(track_lengths>0)));
fprintf('최대 특징점 수: %d\n', max(track_lengths));