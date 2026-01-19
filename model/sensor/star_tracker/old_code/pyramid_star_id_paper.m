% %% pyramid_star_id_paper.m (논문 스타일 - 완전 수정)
function [star_ids, attitude, stats] = pyramid_star_id_paper(pixel_coords, magnitudes, catalog, sensor_params)

star_ids = [];
attitude = [];
stats.success = false;
stats.n_stars_obs = size(pixel_coords, 1);
stats.triangles_tested = 0;
stats.pyramids_tested = 0;
stats.execution_time = 0;

t_start = tic;
n = size(pixel_coords, 1);

%% 1. 픽셀→단위벡터
b_vectors = pixel_to_unit_vector(pixel_coords, sensor_params.f, ...
    sensor_params.myu, sensor_params.l, sensor_params.w);

%% 2. 각거리 사전계산
angle_matrix = precompute_angle_matrix(b_vectors);

%% 3. Smart triangle selection
max_triangles = 10;
triangle_list = smart_triangle_selection(n, magnitudes, angle_matrix, max_triangles);

%% 4. Triangle → Pyramid
for t = 1:size(triangle_list, 1)
    i = triangle_list(t, 1);
    j = triangle_list(t, 2);
    k = triangle_list(t, 3);
    
    match = match_triangle_fast(i, j, k, angle_matrix, b_vectors, ...
        catalog, sensor_params.sigma, sensor_params.k_multiplier);
    
    match.i_obs = i;
    match.j_obs = j;
    match.k_obs = k;
    
    stats.triangles_tested = stats.triangles_tested + 1;
    
    if ~match.success || ~match.unique
        continue;
    end
    
    %% 5. Pyramid 확인
    remaining = setdiff(1:n, [i, j, k]);
    
    for r_idx = remaining
        pyramid = confirm_pyramid_fast(match, r_idx, angle_matrix, b_vectors, ...
            catalog, sensor_params.sigma, sensor_params.k_multiplier);
        
        stats.pyramids_tested = stats.pyramids_tested + 1;
        
        % if pyramid.success && pyramid.frequency < 1e-4
        %     %% ============ 논문 스타일 시작 ============
        % 
        %     identified_4 = [i, j, k, r_idx];
        %     star_ids_4 = [match.I, match.J, match.K, pyramid.R];
        % 
        %     %% STEP 1: 초기 4개 QUEST + Residual
        %     r_cat_4 = catalog.r_I(:, star_ids_4);
        %     b_obs_4 = b_vectors(:, identified_4);
        %     [R_init, ~, residuals_init] = quest_attitude_with_residuals(b_obs_4, r_cat_4);
        % 
        %     %% STEP 2: 초기 정확도 추정 (논문 핵심!)
        %     sigma_estimated = sqrt(mean(residuals_init.^2));
        % 
        %     %% STEP 3: Adaptive Projection Tolerance
        %     tolerance_projection = max(3 * sigma_estimated, deg2rad(0.01));
        % 
        %     %% STEP 4: Projection으로 나머지 식별
        %     remaining_all = setdiff(1:n, identified_4);
        % 
        %     if ~isempty(remaining_all)
        %         b_remaining = b_vectors(:, remaining_all);
        % 
        %         [star_ids_rem, indices_rem] = identify_remaining_by_projection(...
        %             R_init, b_remaining, remaining_all, catalog, tolerance_projection);
        % 
        %         % 합치기
        %         star_ids = [star_ids_4(:); star_ids_rem(:)];
        %         identified_indices = [identified_4(:); indices_rem(:)];
        %     else
        %         star_ids = star_ids_4(:);
        %         identified_indices = identified_4(:);
        %     end
        % 
        %     %% STEP 5: 전체 QUEST + Residuals
        %     r_cat_all = catalog.r_I(:, star_ids);
        %     b_obs_all = b_vectors(:, identified_indices);
        %     [R_all, q_all, residuals_all] = quest_attitude_with_residuals(b_obs_all, r_cat_all);
        % 
        %     %% STEP 6: Adaptive Outlier Threshold (논문!)
        %     if length(residuals_all) > 4
        %         sigma_all = sqrt(mean(residuals_all.^2));
        %         threshold_outlier = 3 * sigma_all;
        % 
        %         inliers = residuals_all < threshold_outlier;
        % 
        %         % 최소 4개는 유지
        %         if sum(inliers) >= 4
        %             star_ids = star_ids(inliers);
        %             identified_indices = identified_indices(inliers);
        % 
        %             % 최종 QUEST
        %             r_cat_final = catalog.r_I(:, star_ids);
        %             b_obs_final = b_vectors(:, identified_indices);
        %             [R_final, q_final] = quest_attitude(b_obs_final, r_cat_final);
        %         else
        %             R_final = R_all;
        %             q_final = q_all;
        %         end
        %     else
        %         R_final = R_all;
        %         q_final = q_all;
        %     end

        if pyramid.success && pyramid.frequency < 1e-4
            identified_4 = [i, j, k, r_idx];
            star_ids_4 = [match.I, match.J, match.K, pyramid.R];
            
            %% STEP 1: 초기 4개 QUEST + Residual
            r_cat_4 = catalog.r_I(:, star_ids_4);
            b_obs_4 = b_vectors(:, identified_4);
            [R_init, ~, residuals_init] = quest_attitude_with_residuals(b_obs_4, r_cat_4);
            
            %% ★ DEBUG
            sigma_estimated = sqrt(mean(residuals_init.^2));
            fprintf('    [DEBUG] 초기 4개: σ=%.2f", residuals=[%.2f %.2f %.2f %.2f]"\n', ...
                sigma_estimated*206265, residuals_init*206265);
            
            tolerance_projection = max(3 * sigma_estimated, deg2rad(0.01));
            fprintf('    [DEBUG] Projection tolerance: %.2f" (%.4f°)\n', ...
                tolerance_projection*3600, rad2deg(tolerance_projection));
            
            %% STEP 4: Projection
            remaining_all = setdiff(1:n, identified_4);
            
            if ~isempty(remaining_all)
                b_remaining = b_vectors(:, remaining_all);
                
                [star_ids_rem, indices_rem] = identify_remaining_by_projection(...
                    R_init, b_remaining, remaining_all, catalog, tolerance_projection);
                
                fprintf('    [DEBUG] Projection: %d → %d identified\n', ...
                    length(remaining_all), length(star_ids_rem));
                
                star_ids = [star_ids_4(:); star_ids_rem(:)];
                identified_indices = [identified_4(:); indices_rem(:)];
            else
                star_ids = star_ids_4(:);
                identified_indices = identified_4(:);
            end
            
            %% STEP 5: 전체 QUEST
            r_cat_all = catalog.r_I(:, star_ids);
            b_obs_all = b_vectors(:, identified_indices);
            [R_all, q_all, residuals_all] = quest_attitude_with_residuals(b_obs_all, r_cat_all);
            
            fprintf('    [DEBUG] 전체 %d개: mean=%.2f", std=%.2f", max=%.2f"\n', ...
                length(residuals_all), mean(residuals_all)*206265, ...
                std(residuals_all)*206265, max(residuals_all)*206265);
            
            %% STEP 6: Outlier
            if length(residuals_all) > 4
                sigma_all = sqrt(mean(residuals_all.^2));
                threshold_outlier = 3 * sigma_all;
                
                inliers = residuals_all < threshold_outlier;
                
                fprintf('    [DEBUG] Outlier: %d/%d removed (threshold: %.2f")\n', ...
                    sum(~inliers), length(inliers), threshold_outlier*206265);
                
                if sum(inliers) >= 4
                    star_ids = star_ids(inliers);
                    identified_indices = identified_indices(inliers);
                    
                    r_cat_final = catalog.r_I(:, star_ids);
                    b_obs_final = b_vectors(:, identified_indices);
                    [R_final, q_final] = quest_attitude(b_obs_final, r_cat_final);
                else
                    R_final = R_all;
                    q_final = q_all;
                end
            else
                R_final = R_all;
                q_final = q_all;
            end
            
            %% ============ 논문 스타일 끝 ============
            
            attitude.R = R_final;
            attitude.q = q_final;
            
            stats.success = true;
            stats.n_identified = length(star_ids);
            stats.confidence = pyramid.confidence;
            stats.frequency = pyramid.frequency;
            stats.execution_time = toc(t_start);
            
            return;
        end
    end
end

%% Fallback - Triangle만
for t = 1:size(triangle_list, 1)
    i = triangle_list(t, 1);
    j = triangle_list(t, 2);
    k = triangle_list(t, 3);
    
    match = match_triangle_fast(i, j, k, angle_matrix, b_vectors, ...
        catalog, sensor_params.sigma, sensor_params.k_multiplier);
    
    if match.success && match.unique
        star_ids = [match.I; match.J; match.K];
        
        r_cat = catalog.r_I(:, star_ids);
        [R, q] = quest_attitude(b_vectors(:, [i,j,k]), r_cat);
        
        attitude.R = R;
        attitude.q = q;
        stats.success = true;
        stats.n_identified = length(star_ids);
        stats.confidence = match.confidence;
        stats.frequency = match.frequency;
        break;
    end
end

stats.execution_time = toc(t_start);

end