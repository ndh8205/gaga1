%% pyramid_star_id_fast.m (NEW - 메인 최적화)
function [star_ids, attitude, pyramid_stats] = pyramid_star_id_fast(pixel_coords, magnitudes, catalog_data, sensor_params)
% Pyramid 고속 버전
%
% Inputs:
%   pixel_coords  - Nx2
%   magnitudes    - Nx1 (NEW!)
%   catalog_data  - k-vector 카탈로그
%   sensor_params - struct

star_ids = [];
attitude = [];
pyramid_stats.success = false;
pyramid_stats.n_stars_obs = size(pixel_coords, 1);
pyramid_stats.triangles_tested = 0;
pyramid_stats.pyramids_tested = 0;
pyramid_stats.execution_time = 0;

t_start = tic;

n = size(pixel_coords, 1);

%% 1. 픽셀 → 단위벡터
b_vectors = pixel_to_unit_vector(pixel_coords, sensor_params.f, ...
    sensor_params.myu, sensor_params.l, sensor_params.w);

%% 2. 각거리 사전 계산 - O(n²) 1회
angle_matrix = precompute_angle_matrix(b_vectors);

%% 3. n=3 특별 케이스
if n == 3
    fprintf('  Warning: n=3 케이스\n');
    
    match = match_triangle_fast(1, 2, 3, angle_matrix, b_vectors, ...
        catalog_data, sensor_params.sigma, sensor_params.k_multiplier);
    
    pyramid_stats.triangles_tested = 1;
    
    if match.success && match.unique
        star_ids = [match.I, match.J, match.K];
        
        r_cat = catalog_data.r_I(:, star_ids);
        [R, q] = quest_attitude(b_vectors, r_cat);
        
        attitude.R = R;
        attitude.q = q;
        pyramid_stats.success = true;
        pyramid_stats.confidence = match.confidence;
    end
    
    pyramid_stats.execution_time = toc(t_start);
    return;
end

%% 4. Smart triangle selection
max_triangles = 10;  % 최대 10개만
triangle_list = smart_triangle_selection(n, magnitudes, angle_matrix, max_triangles);

fprintf('  Smart selection: %d triangles (총 %d 중)\n', ...
    size(triangle_list,1), nchoosek(n,3));

%% 5. Triangle 시도
for t = 1:size(triangle_list, 1)
    i = triangle_list(t, 1);
    j = triangle_list(t, 2);
    k = triangle_list(t, 3);
    
    match = match_triangle_fast(i, j, k, angle_matrix, b_vectors, ...
        catalog_data, sensor_params.sigma, sensor_params.k_multiplier);
    
    % 관측 인덱스 저장
    match.i_obs = i;
    match.j_obs = j;
    match.k_obs = k;
    
    pyramid_stats.triangles_tested = pyramid_stats.triangles_tested + 1;
    
    if ~match.success || ~match.unique
        continue;
    end
    
    %% 6. Pyramid 확인
    remaining = setdiff(1:n, [i, j, k]);
    
    for r_idx = remaining
        pyramid = confirm_pyramid_fast(match, r_idx, angle_matrix, b_vectors, ...
            catalog_data, sensor_params.sigma, sensor_params.k_multiplier);
        
        pyramid_stats.pyramids_tested = pyramid_stats.pyramids_tested + 1;
        
        if pyramid.success && pyramid.frequency < 1e-4
            %% Pyramid 성공!
            star_ids = [match.I, match.J, match.K, pyramid.R];
            
            % 나머지 별 식별
            identified_indices = [i, j, k, r_idx];
            remaining_all = setdiff(1:n, identified_indices);
            
            for p = remaining_all
                b_p = b_vectors(:,p);
                
                P_id = identify_remaining_star(b_p, star_ids, b_vectors(:,identified_indices), ...
                    catalog_data, sensor_params.sigma, sensor_params.k_multiplier);
                
                if ~isempty(P_id)
                    star_ids = [star_ids, P_id];
                    identified_indices = [identified_indices, p];
                end
            end
            
            %% QUEST
            r_cat = catalog_data.r_I(:, star_ids);
            b_obs = b_vectors(:, identified_indices(1:length(star_ids)));
            
            [R, q] = quest_attitude(b_obs, r_cat);
            
            attitude.R = R;
            attitude.q = q;
            
            pyramid_stats.success = true;
            pyramid_stats.n_identified = length(star_ids);
            pyramid_stats.confidence = pyramid.confidence;
            pyramid_stats.frequency = pyramid.frequency;
            pyramid_stats.execution_time = toc(t_start);
            
            return;
        end
    end
end

%% 7. Fallback - Triangle만
fprintf('  Warning: Pyramid 실패, Triangle만 반환\n');

for t = 1:size(triangle_list, 1)
    i = triangle_list(t, 1);
    j = triangle_list(t, 2);
    k = triangle_list(t, 3);
    
    match = match_triangle_fast(i, j, k, angle_matrix, b_vectors, ...
        catalog_data, sensor_params.sigma, sensor_params.k_multiplier);
    
    if match.success && match.unique
        star_ids = [match.I, match.J, match.K];
        
        r_cat = catalog_data.r_I(:, star_ids);
        [R, q] = quest_attitude(b_vectors(:, [i,j,k]), r_cat);
        
        attitude.R = R;
        attitude.q = q;
        pyramid_stats.success = true;
        pyramid_stats.n_identified = length(star_ids);
        pyramid_stats.confidence = match.confidence;
        pyramid_stats.frequency = match.frequency;
        break;
    end
end

pyramid_stats.execution_time = toc(t_start);

end