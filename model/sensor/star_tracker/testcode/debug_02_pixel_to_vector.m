%% debug_02_pixel_to_vector.m (완전 수정판)
% 픽셀 좌표 → 단위벡터 변환 검증
clear; close all; clc;

%% 설정 (stella_check4와 동일)
myu = 2e-6;
f = 0.01042;
l = 1280;
w = 720;

fprintf('=== 픽셀→단위벡터 변환 테스트 ===\n\n');
fprintf('센서 파라미터:\n');
fprintf('  픽셀 크기: %.1f μm\n', myu*1e6);
fprintf('  초점거리: %.2f mm\n', f*1000);
fprintf('  이미지 크기: %d x %d\n', l, w);

FOVx = rad2deg(2 * atan((myu*l/2) / f));
FOVy = rad2deg(2 * atan((myu*w/2) / f));
fprintf('  FOV: %.2f x %.2f deg\n\n', FOVx, FOVy);

%% 1. 테스트 케이스: 중심, 모서리, 대각선
test_cases = {
    'Center',      [0, 0];
    'Top-Left',    [-l/2, -w/2];
    'Top-Right',   [l/2, -w/2];
    'Bottom-Left', [-l/2, w/2];
    'Bottom-Right',[l/2, w/2];
    'Top-Center',  [0, -w/2];
    'Right-Center',[l/2, 0];
};

fprintf('=== 기본 테스트 케이스 ===\n');
for i = 1:size(test_cases, 1)
    name = test_cases{i,1};
    pixel_coords = test_cases{i,2};
    
    b_ST = pixel_to_unit_vector(pixel_coords, f, myu, l, w);
    
    % 각도 계산 (광축으로부터)
    angle_from_optical = acos(b_ST(3));
    
    fprintf('%15s: pixel[%6.1f, %6.1f] → b_ST=[%7.4f, %7.4f, %7.4f], angle=%.2f deg, norm=%.6f\n', ...
        name, pixel_coords(1), pixel_coords(2), ...
        b_ST(1), b_ST(2), b_ST(3), rad2deg(angle_from_optical), norm(b_ST));
end

%% 2. 실제 별 데이터로 테스트
fprintf('\n=== 실제 별 데이터 테스트 ===\n');

% 최신 star_data 로드
data_dir = fullfile(pwd, 'sim_data');
star_data_dir = fullfile(data_dir, 'star_data');

% 파일 존재 확인
if ~exist(star_data_dir, 'dir')
    fprintf('⚠ star_data 디렉토리 없음. 간단한 시뮬레이션으로 대체\n\n');
    
    % 시뮬레이션: 랜덤 별 생성
    n_stars = 15;
    pixel_coords = [(rand(n_stars,1)-0.5)*l, (rand(n_stars,1)-0.5)*w];
    
    fprintf('시뮬레이션: %d개 랜덤 별 생성\n', n_stars);
else
    star_files = dir(fullfile(star_data_dir, 'star_data_occluded_*.mat'));
    if isempty(star_files)
        fprintf('⚠ star_data 파일 없음. 간단한 시뮬레이션으로 대체\n\n');
        n_stars = 15;
        pixel_coords = [(rand(n_stars,1)-0.5)*l, (rand(n_stars,1)-0.5)*w];
        fprintf('시뮬레이션: %d개 랜덤 별 생성\n', n_stars);
    else
        [~, latest_idx] = max([star_files.datenum]);
        load(fullfile(star_data_dir, star_files(latest_idx).name));
        fprintf('로드: %s\n', star_files(latest_idx).name);
        
        % 별이 많은 프레임 선택
        num_stars = [star_data_all.num_stars];
        [n_stars, max_idx] = max(num_stars);
        fprintf('선택 프레임: %d (별 개수: %d)\n\n', max_idx, n_stars);
        
        pixel_coords = star_data_all(max_idx).pixel_coords;
    end
end

%% 픽셀 → 단위벡터 변환 (핵심!)
b_ST = pixel_to_unit_vector(pixel_coords, f, myu, l, w);

fprintf('변환 결과:\n');
fprintf('  별 개수: %d\n', size(b_ST, 2));
fprintf('  norm 범위: %.6f ~ %.6f\n', min(vecnorm(b_ST)), max(vecnorm(b_ST)));

% norm이 1인지 확인
norm_check = vecnorm(b_ST);
if all(abs(norm_check - 1) < 1e-10)
    fprintf('  ✓ 모든 벡터 정규화 완료\n');
else
    fprintf('  ✗ 정규화 오류 감지!\n');
end

%% 3. 각거리 분포 확인
fprintf('\n=== 별쌍 각거리 분포 ===\n');
angles = compute_interstar_angles(b_ST);
angles_deg = rad2deg([angles.angle]);

fprintf('별쌍 개수: %d\n', length(angles));
if ~isempty(angles_deg)
    fprintf('각거리 범위: %.2f ~ %.2f deg\n', min(angles_deg), max(angles_deg));
    fprintf('평균 각거리: %.2f deg\n', mean(angles_deg));

    % FOV 확인
    if max(angles_deg) <= sqrt(FOVx^2 + FOVy^2)
        fprintf('✓ 모든 별쌍이 FOV 내\n');
    else
        fprintf('✗ FOV 초과 별쌍 존재\n');
    end
else
    fprintf('⚠ 별이 1개 이하 - 각거리 계산 불가\n');
end

%% 4. 시각화
if n_stars > 1
    figure('Name', 'Pixel to Vector Validation', 'Position', [100, 100, 1400, 500]);

    % 4-1. 픽셀 좌표
    subplot(1,3,1);
    plot(pixel_coords(:,1), pixel_coords(:,2), 'b.', 'MarkerSize', 10);
    hold on;
    rectangle('Position', [-l/2, -w/2, l, w], 'EdgeColor', 'r', 'LineWidth', 2);
    plot(0, 0, 'r+', 'MarkerSize', 20, 'LineWidth', 2);
    axis equal; grid on;
    xlabel('u [pixels]'); ylabel('v [pixels]');
    title(sprintf('Pixel Coordinates (%d stars)', n_stars));
    set(gca, 'YDir', 'reverse');

    % 4-2. 단위벡터 (3D)
    subplot(1,3,2);
    quiver3(zeros(1,n_stars), zeros(1,n_stars), zeros(1,n_stars), ...
            b_ST(1,:), b_ST(2,:), b_ST(3,:), 0.5, 'b');
    hold on;
    % 광축
    quiver3(0, 0, 0, 0, 0, 1, 0.7, 'r', 'LineWidth', 2);
    axis equal; grid on;
    xlabel('X_{ST}'); ylabel('Y_{ST}'); zlabel('Z_{ST} (optical)');
    title('Unit Vectors (ST Frame)');
    view(45, 30);

    % 4-3. 각거리 히스토그램
    subplot(1,3,3);
    if ~isempty(angles_deg)
        histogram(angles_deg, 30, 'FaceColor', [0.2 0.6 0.8]);
        grid on;
        xlabel('Inter-star Angle (deg)');
        ylabel('Count');
        title(sprintf('Angular Separation (mean: %.2f deg)', mean(angles_deg)));
    else
        text(0.5, 0.5, '데이터 부족', 'HorizontalAlignment', 'center');
        axis off;
    end
end

fprintf('\n=== 테스트 완료 ===\n');