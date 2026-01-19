function [star_ids, attitude, pyramid_stats] = pyramid_star_id(pixel_coords, catalog_data, sensor_params)
% pyramid_star_id - Pyramid 알고리즘 메인 함수
%
% Inputs:
%   pixel_coords  - Nx2 픽셀 좌표
%   catalog_data  - k-vector 카탈로그
%   sensor_params - struct with f, myu, l, w, sigma, k_multiplier
%
% Outputs:
%   star_ids      - 식별된 별 카탈로그 인덱스
%   attitude      - struct with R, q
%   pyramid_stats - 통계 정보

%% 초기화
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

%% 2. n=3 특별 케이스
if n == 3
    fprintf('  Warning: n=3 케이스 (위험)\n');
    
    match = match_triangle(b_vectors(:,1), b_vectors(:,2), b_vectors(:,3), ...
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

%% 3. n >= 4: Smart triangle scanning
triangle_seq = generate_smart_triangle_sequence(n);

for t = 1:size(triangle_seq, 1)
    i = triangle_seq(t, 1);
    j = triangle_seq(t, 2);
    k = triangle_seq(t, 3);
    
    % Triangle 매칭
    match = match_triangle(b_vectors(:,i), b_vectors(:,j), b_vectors(:,k), ...
        catalog_data, sensor_params.sigma, sensor_params.k_multiplier);
    
    match.b_i = b_vectors(:,i);
    match.b_j = b_vectors(:,j);
    match.b_k = b_vectors(:,k);
    
    pyramid_stats.triangles_tested = pyramid_stats.triangles_tested + 1;
    
    if ~match.success || ~match.unique
        continue;
    end
    
    %% 4. Pyramid 확인 (4번째 별 찾기)
    remaining = setdiff(1:n, [i, j, k]);
    
    for r_idx = remaining
        pyramid = confirm_pyramid(match, b_vectors(:,r_idx), ...
            catalog_data, sensor_params.sigma, sensor_params.k_multiplier);
        
        pyramid_stats.pyramids_tested = pyramid_stats.pyramids_tested + 1;
        
        if pyramid.success && pyramid.frequency < 1e-4
            %% Pyramid 발견!
            star_ids = [match.I, match.J, match.K, pyramid.R];
            
            % 나머지 별 식별
            identified_indices = [i, j, k, r_idx];
            remaining_all = setdiff(1:n, identified_indices);
            
            for p = remaining_all
                b_p = b_vectors(:,p);
                
                % 기존 식별된 별들과의 각거리로 식별
                P_id = identify_remaining_star(b_p, star_ids, b_vectors(:,identified_indices), ...
                    catalog_data, sensor_params.sigma, sensor_params.k_multiplier);
                
                if ~isempty(P_id)
                    star_ids = [star_ids, P_id];
                    identified_indices = [identified_indices, p];
                end
            end
            
            %% QUEST 자세 추정
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

%% 5. Pyramid 실패 - Triangle만 반환
fprintf('  Warning: Pyramid 실패, Triangle만 반환\n');

for t = 1:size(triangle_seq, 1)
    i = triangle_seq(t, 1);
    j = triangle_seq(t, 2);
    k = triangle_seq(t, 3);
    
    match = match_triangle(b_vectors(:,i), b_vectors(:,j), b_vectors(:,k), ...
        catalog_data, sensor_params.sigma, sensor_params.k_multiplier);
    
    if match.success && match.unique
        star_ids = [match.I, match.J, match.K];
        
        r_cat = catalog_data.r_I(:, star_ids);
        [R, q] = quest_attitude(b_vectors(:, [i,j,k]), r_cat);
        
        attitude.R = R;
        attitude.q = q;
        pyramid_stats.success = true;
        pyramid_stats.n_identified = length(star_ids);  % 추가!
        pyramid_stats.confidence = match.confidence;
        pyramid_stats.frequency = match.frequency;      % 추가!
        break;
    end
end

pyramid_stats.execution_time = toc(t_start);

end
