function [star_ids, attitude, stats] = pyramid_star_KF(pixel_coords, magnitudes, catalog, sensor_params)
% pyramid_star_KF - ESKF 통합용 Pyramid 알고리즘
%
% Outputs:
%   star_ids - 식별된 별의 카탈로그 ID (1xN)
%   attitude - 자세 정보 (R, q)
%   stats    - 통계 정보
%       .success          - 성공 여부
%       .n_identified     - 식별된 별 개수
%       .obs_indices      - [NEW] 관측 인덱스 (1xN)
%       .cat_indices      - [NEW] 카탈로그 인덱스 (1xN)
%       .confidence       - 신뢰도
%       .execution_time   - 실행 시간

star_ids = [];
attitude = [];
stats.success = false;
stats.n_stars_obs = size(pixel_coords, 1);
stats.triangles_tested = 0;
stats.pyramids_tested = 0;
stats.execution_time = 0;
stats.obs_indices = [];      % [NEW]
stats.cat_indices = [];      % [NEW]

t_start = tic;
n = size(pixel_coords, 1);

if n < 4
    stats.execution_time = toc(t_start);
    return;
end

%% Step 1: 픽셀 → 단위벡터
b_vectors = pixel_to_unit_vector(pixel_coords, sensor_params.f, ...
    sensor_params.myu, sensor_params.l, sensor_params.w);

%% Step 2: 각거리 행렬
angle_matrix = precompute_angle_matrix(b_vectors);

%% Step 3: Triangle 시퀀스
triangle_list = smart_triangle_selection(n, magnitudes, angle_matrix, 10);

%% Step 4: Triangle → Pyramid Loop
for t = 1:size(triangle_list, 1)
    i = triangle_list(t, 1);
    j = triangle_list(t, 2);
    k = triangle_list(t, 3);
    
    % Triangle matching
    match = match_triangle_fast(i, j, k, angle_matrix, b_vectors, ...
        catalog, sensor_params.sigma, sensor_params.k_multiplier);
    
    stats.triangles_tested = stats.triangles_tested + 1;
    
    if ~match.success
        continue;
    end

    % 모든 후보 순회
    for c_idx = 1:match.n_matches
        
        % 후보 1개에 대한 임시 구조체
        temp_match = struct();
        temp_match.success = true;
        temp_match.I = match.candidates(c_idx, 1);
        temp_match.J = match.candidates(c_idx, 2);
        temp_match.K = match.candidates(c_idx, 3);
        temp_match.i_obs = i;
        temp_match.j_obs = j;
        temp_match.k_obs = k;
        temp_match.frequency = match.frequency;

        % Pyramid confirmation
        remaining = setdiff(1:n, [i, j, k]);
        
        for r_idx = remaining
            pyramid = confirm_pyramid_fast(temp_match, r_idx, angle_matrix, b_vectors, ...
                catalog, sensor_params.sigma, sensor_params.k_multiplier);
            
            stats.pyramids_tested = stats.pyramids_tested + 1;
            
            if pyramid.success && pyramid.frequency < 1e-4
                
                %% ========== Algorithm 시작 ==========
                
                % Pyramid: 4개 별 식별됨
                obs_indices = [i, j, k, r_idx];
                cat_indices = [temp_match.I, temp_match.J, temp_match.K, pyramid.R];
                
                % Step 4.1: 초기 자세 추정 (4개)
                b_4 = b_vectors(:, obs_indices);
                r_4 = catalog.r_I(:, cat_indices);
                
                R_I2B = quest_attitude_NR(b_4, r_4);
           
                % Step 4.2: Residual 계산
                residuals_4 = zeros(4, 1);
                for m = 1:4
                    r_pred = R_I2B * r_4(:,m);
                    cos_angle = b_4(:,m)' * r_pred;
                    cos_angle = max(-1, min(1, cos_angle));
                    residuals_4(m) = acos(cos_angle);
                end
                
                % Step 4.3: σ 추정
                sigma_attitude = sqrt(mean(residuals_4.^2));
                sigma_sensor = sensor_params.sigma;
                sigma_total = sqrt(sigma_attitude^2 + sigma_sensor^2);
                
                % Step 4.4: Projection tolerance
                k_tol = 3.0;
                tolerance = k_tol * sigma_total;
                
                % Step 4.5: Remaining stars projection
                remaining_all = setdiff(1:n, obs_indices);
                identified_obs = obs_indices;
                identified_cat = cat_indices;
                
                for p_idx = remaining_all
                    b_p = b_vectors(:, p_idx);
                    
                    % Body → Inertial 예측
                    r_pred = R_I2B' * b_p;
                    
                    % 카탈로그에서 가장 가까운 별
                    cos_angles = catalog.r_I' * r_pred;
                    cos_angles = max(-1, min(1, cos_angles));
                    angles = acos(cos_angles);
                    
                    [min_angle, best_idx] = min(angles);
                    
                    if min_angle < tolerance
                        identified_obs = [identified_obs, p_idx];
                        identified_cat = [identified_cat, best_idx];
                    end
                end
                
                % Step 4.6: 전체 자세 추정
                b_all = b_vectors(:, identified_obs);
                r_all = catalog.r_I(:, identified_cat);
                
                R_all = quest_attitude_NR(b_all, r_all);
                
                % Step 4.7: Residual 계산 (전체)
                n_id = length(identified_obs);
                residuals_all = zeros(n_id, 1);
                for m = 1:n_id
                    r_pred = R_all * r_all(:,m);
                    cos_angle = b_all(:,m)' * r_pred;
                    cos_angle = max(-1, min(1, cos_angle));
                    residuals_all(m) = acos(cos_angle);
                end
                
                % Step 4.8: Outlier 제거
                sigma_final = sqrt(mean(residuals_all.^2));
                threshold = 3 * sigma_final;
                
                inliers = residuals_all < threshold;
                
                if sum(inliers) >= 4
                    identified_obs = identified_obs(inliers);
                    identified_cat = identified_cat(inliers);
                    
                    % 최종 자세
                    b_final = b_vectors(:, identified_obs);
                    r_final = catalog.r_I(:, identified_cat);
                    R_final = quest_attitude_NR(b_final, r_final);
                    q_final = DCM2Quat(R_final);
                else
                    R_final = R_all;
                    q_final = DCM2Quat(R_all);
                end
                
                %% ========== Algorithm 끝 ==========
                
                % 카탈로그 인덱스 → Star ID 변환
                star_ids = catalog.star_ID(identified_cat);
                
                attitude.R = R_final;
                attitude.q = q_final;
                
                stats.success = true;
                stats.n_identified = length(star_ids);
                stats.obs_indices = identified_obs;    % [NEW]
                stats.cat_indices = identified_cat;    % [NEW]
                stats.confidence = pyramid.confidence;
                stats.frequency = pyramid.frequency;
                stats.execution_time = toc(t_start);
                
                return;
            end
        end % r_idx loop
    end % c_idx loop
end % triangle loop

%% Fallback
stats.execution_time = toc(t_start);

end