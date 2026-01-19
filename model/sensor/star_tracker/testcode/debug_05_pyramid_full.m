%% debug_05_pyramid_full.m
% Pyramid 알고리즘 전체 테스트
clear; close all; clc;
dhzp
fprintf('=== Pyramid 알고리즘 전체 테스트 ===\n\n');

%% 1. 카탈로그 로드
load('star_catalog_kvector.mat');
fprintf('카탈로그 로드: %d개 별\n\n', catalog_data.N_stars);

%% 2. 센서 파라미터
sensor_params.f = 0.01042;
sensor_params.myu = 2e-6;
sensor_params.l = 1280;
sensor_params.w = 720;
sensor_params.sigma = 50e-6;
sensor_params.k_multiplier = 6.4;

FOVx = rad2deg(2*atan((sensor_params.myu*sensor_params.l/2)/sensor_params.f));
FOVy = rad2deg(2*atan((sensor_params.myu*sensor_params.w/2)/sensor_params.f));

fprintf('센서 파라미터:\n');
fprintf('  FOV: %.2f x %.2f deg\n', FOVx, FOVy);
fprintf('  σ: %.1f μrad\n', sensor_params.sigma*1e6);
fprintf('  k: %.1f\n\n', sensor_params.k_multiplier);

%% 3. 시뮬레이션: FOV 내 별 생성
rng(456);
center_ra = rand() * 2*pi;
center_dec = (rand()-0.5) * pi;

center_dir = [cos(center_ra)*cos(center_dec); 
              sin(center_ra)*cos(center_dec); 
              sin(center_dec)];

FOV_half = catalog_data.FOV_rad / 2;
angles_from_center = acos(catalog_data.r_I' * center_dir);
in_fov_idx = find(angles_from_center < FOV_half);

fprintf('시뮬레이션:\n');
fprintf('  중심: RA=%.2f°, DEC=%.2f°\n', rad2deg(center_ra), rad2deg(center_dec));
fprintf('  FOV 내 별: %d개\n\n', length(in_fov_idx));

if length(in_fov_idx) < 4
    error('FOV 내 별이 4개 미만');
end

%% 4. 관측 시뮬레이션 (6~8개 별 선택)
n_stars = min(8, length(in_fov_idx));
selected_idx = in_fov_idx(randperm(length(in_fov_idx), n_stars));

fprintf('선택된 별: %d개\n', n_stars);
for i = 1:n_stars
    fprintf('  Star %d: ID=%d, mag=%.2f\n', i, ...
        catalog_data.star_catalog.ID(selected_idx(i)), ...
        catalog_data.star_catalog.Magnitude(selected_idx(i)));
end
fprintf('\n');

%% 5. 자세 생성
z_body = center_dir;
x_body = cross([0;0;1], z_body);
if norm(x_body) < 1e-6
    x_body = [1;0;0];
end
x_body = x_body / norm(x_body);
y_body = cross(z_body, x_body);
R_I2B_true = [x_body, y_body, z_body]';
q_true = DCM2Quat(R_I2B_true);

%% 6. 픽셀 좌표 생성
pixel_coords = zeros(n_stars, 2);

for i = 1:n_stars
    r_I = catalog_data.r_I(:, selected_idx(i));
    
    % 관측 벡터 (노이즈 추가)
    b = R_I2B_true * r_I + sensor_params.sigma * randn(3,1);
    b = b / norm(b);
    
    % 픽셀 좌표
    x_sensor = b(1) * sensor_params.f / b(3);
    y_sensor = b(2) * sensor_params.f / b(3);
    
    pixel_coords(i,1) = x_sensor / sensor_params.myu;
    pixel_coords(i,2) = y_sensor / sensor_params.myu;
end

fprintf('픽셀 좌표 범위:\n');
fprintf('  u: [%.1f, %.1f]\n', min(pixel_coords(:,1)), max(pixel_coords(:,1)));
fprintf('  v: [%.1f, %.1f]\n\n', min(pixel_coords(:,2)), max(pixel_coords(:,2)));

%% 7. Pyramid Star-ID 실행
fprintf('=== Pyramid Star-ID 실행 ===\n');
[star_ids, attitude, pyramid_stats] = pyramid_star_id(pixel_coords, catalog_data, sensor_params);

%% 8. 결과 출력
fprintf('\n=== 결과 ===\n');
fprintf('성공: %s\n', char(10004*pyramid_stats.success + 10008*(~pyramid_stats.success)));
fprintf('실행시간: %.2f ms\n', pyramid_stats.execution_time*1000);
fprintf('테스트한 Triangle: %d\n', pyramid_stats.triangles_tested);
fprintf('테스트한 Pyramid: %d\n', pyramid_stats.pyramids_tested);

if pyramid_stats.success
    fprintf('식별된 별: %d개 (관측: %d개)\n', pyramid_stats.n_identified, n_stars);
    fprintf('Frequency: %.2e\n', pyramid_stats.frequency);
    fprintf('Confidence: %.2f%%\n\n', pyramid_stats.confidence*100);
    
    % 매칭 확인
    fprintf('=== 매칭 검증 ===\n');
    n_correct = 0;
    for i = 1:length(star_ids)
        is_correct = any(selected_idx == star_ids(i));
        n_correct = n_correct + is_correct;
        fprintf('  ID %d: %s\n', star_ids(i), char(10004*is_correct + 10008*~is_correct));
    end
    fprintf('정확도: %d/%d (%.1f%%)\n\n', n_correct, length(star_ids), ...
        100*n_correct/length(star_ids));
    
    % 자세 오차
    if ~isempty(attitude)
        R_err = attitude.R * R_I2B_true';
        q_err = DCM2Quat(R_err);
        angle_err = 2 * acos(min(1, abs(q_err(1))));
        
        fprintf('=== 자세 추정 ===\n');
        fprintf('자세 오차: %.4f° (%.2f arcmin)\n', ...
            rad2deg(angle_err), rad2deg(angle_err)*60);
        
        if angle_err < deg2rad(0.1)
            fprintf('✓ 고정밀 자세 추정 성공\n');
        elseif angle_err < deg2rad(1.0)
            fprintf('✓ 자세 추정 성공\n');
        else
            fprintf('⚠ 자세 오차 큼\n');
        end
    end
else
    fprintf('✗ Star-ID 실패\n');
end

fprintf('\n=== 테스트 완료 ===\n');