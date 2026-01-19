function match_result = match_triangle(b_i, b_j, b_k, catalog_data, sigma, k_multiplier)
% match_triangle - 3개 별로 triangle 매칭
%
% Inputs:
%   b_i, b_j, b_k  - 관측 단위벡터 (3x1)
%   catalog_data   - k-vector 카탈로그
%   sigma          - 센서 정밀도 (rad)
%   k_multiplier   - tolerance 배수
%
% Output:
%   match_result   - struct
%       .success    - 매칭 성공 여부
%       .unique     - unique 매칭 여부
%       .I, J, K    - 카탈로그 인덱스
%       .frequency  - 랜덤 매칭 빈도
%       .confidence - 신뢰도

match_result.success = false;
match_result.unique = false;
match_result.I = [];
match_result.J = [];
match_result.K = [];
match_result.frequency = inf;
match_result.confidence = 0;

tolerance = k_multiplier * sigma;

%% 1. 관측 각거리 계산
theta_ij = acos(max(-1, min(1, b_i' * b_j)));
theta_ik = acos(max(-1, min(1, b_i' * b_k)));
theta_jk = acos(max(-1, min(1, b_j' * b_k)));

%% 2. k-vector로 각 변에 대한 후보 찾기
[cand_ij, ~, ~] = kvector_range_search(theta_ij, tolerance, catalog_data);
[cand_ik, ~, ~] = kvector_range_search(theta_ik, tolerance, catalog_data);
[cand_jk, ~, ~] = kvector_range_search(theta_jk, tolerance, catalog_data);

if isempty(cand_ij) || isempty(cand_ik) || isempty(cand_jk)
    return;
end

%% 3. Triangle 매칭 (수정됨)
valid_triangles = [];

for m = 1:length(cand_ij)
    I_cand = cand_ij(m).I;
    J_cand = cand_ij(m).J;
    
    % i-k에서 같은 I를 가진 것 찾기
    for n = 1:length(cand_ik)
        if cand_ik(n).I == I_cand
            K_cand = cand_ik(n).J;
            
            % j-k 확인
            jk_match = false;
            for p = 1:length(cand_jk)
                if (cand_jk(p).I == J_cand && cand_jk(p).J == K_cand) || ...
                   (cand_jk(p).I == K_cand && cand_jk(p).J == J_cand)
                    jk_match = true;
                    break;
                end
            end
            
            if jk_match
                % Triangle 발견!
                % 중복 체크
                is_duplicate = false;
                for t = 1:size(valid_triangles, 1)
                    if valid_triangles(t,1) == I_cand && ...
                       valid_triangles(t,2) == J_cand && ...
                       valid_triangles(t,3) == K_cand
                        is_duplicate = true;
                        break;
                    end
                end
                
                if ~is_duplicate
                    valid_triangles = [valid_triangles; I_cand, J_cand, K_cand];
                end
            end
        end
    end
end

if isempty(valid_triangles)
    return;
end

%% 4. Specular triangle 제거
r_I_all = catalog_data.r_I;
valid_non_specular = [];

for t = 1:size(valid_triangles, 1)
    I_cand = valid_triangles(t, 1);
    J_cand = valid_triangles(t, 2);
    K_cand = valid_triangles(t, 3);
    
    r_I_vec = r_I_all(:, I_cand);
    r_J_vec = r_I_all(:, J_cand);
    r_K_vec = r_I_all(:, K_cand);
    
    % Handedness 체크 (coplanar 체크 포함)
    cross_obs = cross(b_j, b_k);
    cross_cat = cross(r_J_vec, r_K_vec);
    
    if norm(cross_obs) < 1e-10 || norm(cross_cat) < 1e-10
        continue; % coplanar - skip
    end
    
    sign_obs = sign(b_i' * cross_obs);
    sign_cat = sign(r_I_vec' * cross_cat);
    
    if sign_obs == sign_cat && sign_obs ~= 0
        valid_non_specular = [valid_non_specular; I_cand, J_cand, K_cand];
    end
end

if isempty(valid_non_specular)
    return;
end

%% 5. Unique 체크 및 결과 저장
match_result.success = true;
match_result.unique = (size(valid_non_specular, 1) == 1);
match_result.I = valid_non_specular(1, 1);
match_result.J = valid_non_specular(1, 2);
match_result.K = valid_non_specular(1, 3);
match_result.n_matches = size(valid_non_specular, 1);

%% 6. Frequency 계산 (Pyramid 논문 식 13)
N = catalog_data.N_stars;
k_sigma = k_multiplier * sigma;

% Spherical triangle에서 sine law 사용
% sin(θ_ij) / sin(θ_k) 계산
% θ_k는 vertex k에서의 내각

% Spherical cosine law로 cos(θ_k) 계산
cos_theta_k = (cos(theta_jk) - cos(theta_ij)*cos(theta_ik)) / ...
              (sin(theta_ij)*sin(theta_ik));
cos_theta_k = max(-1, min(1, cos_theta_k)); % clipping

theta_k_angle = acos(cos_theta_k);
sin_theta_k = sin(theta_k_angle);

if sin_theta_k < 1e-10
    sin_theta_k = 1e-10; % singularity 방지
end

sin_theta_ij = sin(theta_ij);

% Triangle frequency (논문 식 13)
match_result.frequency = (N*(N-1)*(N-2)/pi) * (k_sigma)^3 * ...
                         (sin_theta_ij / sin_theta_k);

match_result.confidence = max(0, 1 - match_result.frequency);

end