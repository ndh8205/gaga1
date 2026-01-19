%% debug_05b_pyramid_failure_analysis.m
% Pyramid가 왜 실패했는지 분석
clear; close all; clc;

fprintf('=== Pyramid 실패 원인 분석 ===\n\n');

%% 1. 카탈로그 로드
load('star_catalog_kvector.mat');

%% 2. 센서 파라미터
sensor_params.f = 0.01042;
sensor_params.myu = 2e-6;
sensor_params.l = 1280;
sensor_params.w = 720;
sensor_params.sigma = 50e-6;
sensor_params.k_multiplier = 6.4;

%% 3. 같은 시드로 재현
rng(456);
center_ra = rand() * 2*pi;
center_dec = (rand()-0.5) * pi;

center_dir = [cos(center_ra)*cos(center_dec); 
              sin(center_ra)*cos(center_dec); 
              sin(center_dec)];

FOV_half = catalog_data.FOV_rad / 2;
angles_from_center = acos(catalog_data.r_I' * center_dir);
in_fov_idx = find(angles_from_center < FOV_half);

n_stars = min(8, length(in_fov_idx));
selected_idx = in_fov_idx(randperm(length(in_fov_idx), n_stars));

fprintf('선택된 별: %d개\n', n_stars);

%% 4. 자세 및 관측 생성
z_body = center_dir;
x_body = cross([0;0;1], z_body);
if norm(x_body) < 1e-6
    x_body = [1;0;0];
end
x_body = x_body / norm(x_body);
y_body = cross(z_body, x_body);
R_I2B_true = [x_body, y_body, z_body]';

pixel_coords = zeros(n_stars, 2);
b_vectors_true = zeros(3, n_stars);

for i = 1:n_stars
    r_I = catalog_data.r_I(:, selected_idx(i));
    b = R_I2B_true * r_I + sensor_params.sigma * randn(3,1);
    b = b / norm(b);
    b_vectors_true(:,i) = b;
    
    x_sensor = b(1) * sensor_params.f / b(3);
    y_sensor = b(2) * sensor_params.f / b(3);
    
    pixel_coords(i,1) = x_sensor / sensor_params.myu;
    pixel_coords(i,2) = y_sensor / sensor_params.myu;
end

%% 5. Triangle 찾기
b_vectors = pixel_to_unit_vector(pixel_coords, sensor_params.f, ...
    sensor_params.myu, sensor_params.l, sensor_params.w);

triangle_seq = generate_smart_triangle_sequence(n_stars);

fprintf('\n=== Triangle 매칭 ===\n');
successful_triangles = [];

for t = 1:size(triangle_seq, 1)
    i = triangle_seq(t, 1);
    j = triangle_seq(t, 2);
    k = triangle_seq(t, 3);
    
    match = match_triangle(b_vectors(:,i), b_vectors(:,j), b_vectors(:,k), ...
        catalog_data, sensor_params.sigma, sensor_params.k_multiplier);
    
    if match.success && match.unique
        % Ground truth 확인
        is_correct = (match.I == selected_idx(i) && ...
                      match.J == selected_idx(j) && ...
                      match.K == selected_idx(k));
        
        successful_triangles = [successful_triangles; t, i, j, k, ...
            match.I, match.J, match.K, is_correct, match.frequency];
        
        fprintf('Triangle %d [%d,%d,%d]: ID[%d,%d,%d] freq=%.2e %s\n', ...
            t, i, j, k, match.I, match.J, match.K, match.frequency, ...
            char(10004*is_correct + 10008*~is_correct));
    end
end

if isempty(successful_triangles)
    fprintf('✗ Triangle 매칭 실패\n');
    return;
end

%% 6. 첫 번째 성공 Triangle로 Pyramid 시도
fprintf('\n=== Pyramid 확인 (첫 번째 Triangle) ===\n');

t_idx = successful_triangles(1, 1);
i = successful_triangles(1, 2);
j = successful_triangles(1, 3);
k = successful_triangles(1, 4);

match = match_triangle(b_vectors(:,i), b_vectors(:,j), b_vectors(:,k), ...
    catalog_data, sensor_params.sigma, sensor_params.k_multiplier);
match.b_i = b_vectors(:,i);
match.b_j = b_vectors(:,j);
match.b_k = b_vectors(:,k);

fprintf('기본 Triangle: [%d,%d,%d] → ID[%d,%d,%d]\n', i, j, k, match.I, match.J, match.K);

remaining = setdiff(1:n_stars, [i, j, k]);
fprintf('나머지 별: %d개\n\n', length(remaining));

pyramid_found = false;

for r_idx = remaining
    fprintf('  4번째 별 후보: %d (true ID: %d)\n', r_idx, selected_idx(r_idx));
    
    % Pyramid 확인
    pyramid = confirm_pyramid(match, b_vectors(:,r_idx), ...
        catalog_data, sensor_params.sigma, sensor_params.k_multiplier);
    
    if pyramid.success
        is_correct = (pyramid.R == selected_idx(r_idx));
        fprintf('    → Pyramid ID: %d, freq=%.2e, conf=%.2f%% %s\n', ...
            pyramid.R, pyramid.frequency, pyramid.confidence*100, ...
            char(10004*is_correct + 10008*~is_correct));
        
        if pyramid.frequency < 1e-7
            fprintf('    ✓ Frequency threshold 통과!\n');
            pyramid_found = true;
        else
            fprintf('    ✗ Frequency 너무 높음 (threshold: 1e-7)\n');
        end
    else
        fprintf('    ✗ Pyramid 매칭 실패\n');
    end
end

if ~pyramid_found
    fprintf('\n⚠ Pyramid threshold 통과 실패\n');
    fprintf('\n원인 분석:\n');
    fprintf('  1. Frequency threshold (1e-7)가 너무 엄격할 수 있음\n');
    fprintf('  2. 별 개수가 적어 frequency가 높을 수 있음\n');
    fprintf('  3. 각거리 측정 노이즈로 인한 후보 증가\n');
end

fprintf('\n=== 분석 완료 ===\n');