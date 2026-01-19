%% match_triangle_fast.m (Specular 순서 수정)
function match_result = match_triangle_fast(i, j, k, angle_matrix, b_vectors, catalog, sigma, k_multiplier)

match_result.success = false;
match_result.unique = false;
match_result.I = [];
match_result.J = [];
match_result.K = [];
match_result.frequency = inf;
match_result.confidence = 0;

tolerance = k_multiplier * sigma;

%% 1. k-vector 검색
theta_ij = angle_matrix(i, j);
theta_ik = angle_matrix(i, k);
theta_jk = angle_matrix(j, k);

[I_ij, J_ij, ~] = kvector_range_search_fast(theta_ij, tolerance, catalog);
[I_ik, J_ik, ~] = kvector_range_search_fast(theta_ik, tolerance, catalog);
[I_jk, J_jk, ~] = kvector_range_search_fast(theta_jk, tolerance, catalog);

if isempty(I_ij) || isempty(I_ik) || isempty(I_jk)
    return;
end

%% 2. Triangle 매칭 (순서 유지)
raw_triangles = [];

for m = 1:length(I_ij)
    star_a = I_ij(m);
    star_b = J_ij(m);
    
    % Case 1: star_a=I(i), star_b=J(j)
    ik_idx = find(I_ik == star_a | J_ik == star_a);
    for n = ik_idx'
        star_c = I_ik(n) + J_ik(n) - star_a;
        
        if any((I_jk == star_b & J_jk == star_c) | ...
               (I_jk == star_c & J_jk == star_b))
            % 순서 그대로: i->star_a, j->star_b, k->star_c
            raw_triangles = [raw_triangles; star_a, star_b, star_c];
        end
    end
    
    % Case 2: star_a=J(j), star_b=I(i)
    jk_idx = find(I_jk == star_a | J_jk == star_a);
    for n = jk_idx'
        star_c = I_jk(n) + J_jk(n) - star_a;
        
        if any((I_ik == star_b & J_ik == star_c) | ...
               (I_ik == star_c & J_ik == star_b))
            % 순서 그대로: i->star_b, j->star_a, k->star_c
            raw_triangles = [raw_triangles; star_b, star_a, star_c];
        end
    end
end

if isempty(raw_triangles)
    return;
end

%% 3. Specular 제거 (순서 유지 상태에서)
b_i = b_vectors(:, i);
b_j = b_vectors(:, j);
b_k = b_vectors(:, k);
cross_obs = cross(b_j, b_k);

if norm(cross_obs) < 1e-10
    return;
end

sign_obs = sign(b_i' * cross_obs);
valid_non_specular = [];

for t = 1:size(raw_triangles, 1)
    % (i, j, k) 순서에 대응하는 (I, J, K)
    I_cand = raw_triangles(t, 1);
    J_cand = raw_triangles(t, 2);
    K_cand = raw_triangles(t, 3);
    
    r_I = catalog.r_I(:, I_cand);
    r_J = catalog.r_I(:, J_cand);
    r_K = catalog.r_I(:, K_cand);
    
    cross_cat = cross(r_J, r_K);
    
    if norm(cross_cat) < 1e-10
        continue;
    end
    
    % 순서 보장된 상태에서 부호 비교 (Eq. 16)
    if sign(r_I' * cross_cat) == sign_obs
        % 비반사해 → 정렬해서 저장
        valid_non_specular = [valid_non_specular; sort([I_cand, J_cand, K_cand])];
    end
end

if isempty(valid_non_specular)
    return;
end

% 이제 중복 제거
final_matches = unique(valid_non_specular, 'rows');

%% 4. 결과
match_result.success = true;
match_result.unique = (size(final_matches, 1) == 1);
match_result.I = final_matches(1, 1);
match_result.J = final_matches(1, 2);
match_result.K = final_matches(1, 3);
match_result.candidates = final_matches;
match_result.n_matches = size(final_matches, 1);

%% 5. Frequency (수정)
N = catalog.N_stars;
k_sigma = k_multiplier * sigma;

% 꼭짓점 k 내각 (변 theta_ij 맞은편)
cos_theta_k_angle = (cos(theta_ij) - cos(theta_ik)*cos(theta_jk)) / ...
                    (sin(theta_ik)*sin(theta_jk));
cos_theta_k_angle = max(-1, min(1, cos_theta_k_angle));
sin_theta_k_angle = sin(acos(cos_theta_k_angle));

match_result.frequency = (N*(N-1)*(N-2)/pi) * (k_sigma)^3 * ...
                         (sin(theta_ij) / max(sin_theta_k_angle, 1e-10));
match_result.confidence = max(0, 1 - match_result.frequency);

end