%% confirm_pyramid_fast.m (수정 - 배열 기반)
function pyramid_result = confirm_pyramid_fast(triangle_match, r_idx, angle_matrix, b_vectors, catalog, sigma, k_multiplier)

pyramid_result.success = false;
pyramid_result.R = [];
pyramid_result.frequency = inf;
pyramid_result.confidence = 0;

if ~triangle_match.success
    return;
end

tolerance = k_multiplier * sigma;

%% 1. Triangle 정보
I = triangle_match.I;
J = triangle_match.J;
K = triangle_match.K;

i_obs = triangle_match.i_obs;
j_obs = triangle_match.j_obs;
k_obs = triangle_match.k_obs;

%% 2. 각거리 lookup
theta_ir = angle_matrix(i_obs, r_idx);
theta_jr = angle_matrix(j_obs, r_idx);
theta_kr = angle_matrix(k_obs, r_idx);

%% 3. k-vector 검색 (배열 기반)
[I_ir, J_ir, ~] = kvector_range_search_fast(theta_ir, tolerance, catalog);
[I_jr, J_jr, ~] = kvector_range_search_fast(theta_jr, tolerance, catalog);
[I_kr, J_kr, ~] = kvector_range_search_fast(theta_kr, tolerance, catalog);

if isempty(I_ir) || isempty(I_jr) || isempty(I_kr)
    return;
end

%% 4. 공통 R 찾기
R_candidates = [];

for m = 1:length(I_ir)
    if I_ir(m) == I
        R_temp = J_ir(m);
    elseif J_ir(m) == I
        R_temp = I_ir(m);
    else
        continue;
    end
    
    % J-R 확인
    jr_match = any((I_jr == J & J_jr == R_temp) | ...
                   (I_jr == R_temp & J_jr == J));
    
    if ~jr_match
        continue;
    end
    
    % K-R 확인
    kr_match = any((I_kr == K & J_kr == R_temp) | ...
                   (I_kr == R_temp & J_kr == K));
    
    if kr_match
        R_candidates = [R_candidates; R_temp];
    end
end

R_candidates = unique(R_candidates);

if isempty(R_candidates)
    return;
end

%% 5. 성공
pyramid_result.success = true;
pyramid_result.R = R_candidates(1);
pyramid_result.n_candidates = length(R_candidates);

%% 6. Frequency
N = catalog.N_stars;
k_sigma = k_multiplier * sigma;
f_ijk = triangle_match.frequency;

pyramid_result.frequency = (N - 3) * (1 - cos(k_sigma)) / 2 * f_ijk;
pyramid_result.confidence = max(0, 1 - pyramid_result.frequency);

end