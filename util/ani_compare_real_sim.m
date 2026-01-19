function ani_compare_real_sim(sensor_data, sampled_images, camera_params, colors, t1)
% 실제 이미지와 시뮬레이션 재투영 비교 애니메이션

if ~exist('sampled_images', 'var') || isempty(sampled_images)
    error('실제 이미지가 필요합니다.');
end

fig = figure('Name', 'Real vs Simulated Camera Views', 'Position', [100, 100, 1600, 700], 'Color', 'k');
fprintf('실제 vs 시뮬레이션 비교 애니메이션 시작...\n');

% 카메라 측정이 있는 프레임 찾기
cam_frames = find(arrayfun(@(s) ~isempty(s.z_measured), sensor_data));
num_cam_frames = length(cam_frames);

% 샘플 이미지 개수와 매칭
num_real_images = length(sampled_images);
if num_real_images ~= num_cam_frames
    warning('이미지 개수 불일치: 실제 %d, 시뮬레이션 %d', num_real_images, num_cam_frames);
    num_frames = min(num_real_images, num_cam_frames);
else
    num_frames = num_cam_frames;
end

K = camera_params.intrinsic.K_ideal;
img_width = camera_params.intrinsic.image_width;
img_height = camera_params.intrinsic.image_height;

for i = 1:num_frames
    k = cam_frames(i);
    current_time = sensor_data(k).time;
    z_measured = sensor_data(k).z_measured;
    visible_measured = z_measured(3, :);
    pixels_measured = z_measured(1:2, :);
    
    % --- 왼쪽: 실제 이미지 ---
    subplot(1, 2, 1);
    imshow(sampled_images{i});
    hold on;
    
    % 이미지 프레임 표시
    rectangle('Position', [0, 0, img_width, img_height], ...
              'EdgeColor', 'g', 'LineWidth', 2);
    
    % 중심점 표시
    cx = K(1,3); cy = K(2,3);
    plot(cx, cy, 'g+', 'MarkerSize', 15, 'LineWidth', 2);
    
    hold off;
    title(sprintf('Real Image (t=%.1f min)', current_time/60), ...
          'Color', 'w', 'FontSize', 12);
    xlabel('u [pixels]', 'Color', 'w');
    ylabel('v [pixels]', 'Color', 'w');
    
    % --- 오른쪽: 시뮬레이션 재투영 ---
    subplot(1, 2, 2);
    
    % 지구 배경이 있으면 표시
    if isfield(sensor_data(k), 'earth_image') && ~isempty(sensor_data(k).earth_image)
        imshow(sensor_data(k).earth_image);
        hold on;
    else
        % 검은 배경
        cla;
        hold on;
        set(gca, 'Color', 'k');
        xlim([0, img_width]);
        ylim([0, img_height]);
    end
    
    % 재투영된 포인트 표시
    visible_idx = find(visible_measured > 0.5);
    if ~isempty(visible_idx)
        plot(pixels_measured(1, visible_idx), pixels_measured(2, visible_idx), ...
             '.', 'Color', colors.meas_proj, 'MarkerSize', 3);
    end
    
    % 프레임과 중심점
    rectangle('Position', [0, 0, img_width, img_height], ...
              'EdgeColor', 'w', 'LineWidth', 2);
    plot(cx, cy, 'w+', 'MarkerSize', 15, 'LineWidth', 2);
    
    hold off;
    set(gca, 'YDir', 'reverse');
    axis equal;
    xlim([0, img_width]);
    ylim([0, img_height]);
    
    title(sprintf('Simulated View\nVisible: %d pts', sum(visible_measured > 0.5)), ...
          'Color', 'w', 'FontSize', 12);
    xlabel('u [pixels]', 'Color', 'w');
    ylabel('v [pixels]', 'Color', 'w');
    set(gca, 'XColor', 'w', 'YColor', 'w');
    
    % 전체 타이틀
    sgtitle(sprintf('Frame %d/%d | Time: %.1f sec (%.2f min)', ...
            i, num_frames, current_time, current_time/60), ...
            'FontSize', 14, 'FontWeight', 'bold', 'Color', 'w');
    
    drawnow;
    
    % 진행 상황 출력
    if mod(i, 10) == 0 || i == 1
        fprintf('Frame %d/%d: t=%.1fs, Visible=%d pts\n', ...
                i, num_frames, current_time, sum(visible_measured > 0.5));
    end
end

fprintf('비교 애니메이션 완료!\n');
end