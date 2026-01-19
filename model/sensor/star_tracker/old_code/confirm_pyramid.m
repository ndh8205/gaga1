function pyramid_result = confirm_pyramid(triangle_match, b_r, catalog_data, sigma, k_multiplier)
% confirm_pyramid - 4번째 별로 pyramid 확인
%
% Inputs:
%   triangle_match - match_triangle 결과
%   b_r            - 4번째 별 관측 벡터 (3x1)
%   catalog_data   - k-vector 카탈로그
%   sigma          - 센서 정밀도
%   k_multiplier   - tolerance 배수
%
% Output:
%   pyramid_result - struct
%       .success    - pyramid 확인 성공
%       .R          - 카탈로그 인덱스
%       .frequency  - 랜덤 매칭 빈도
%       .confidence - 신뢰도

pyramid_result.success = false;
pyramid_result.R = [];
pyramid_result.frequency = inf;
pyramid_result.confidence = 0;

if ~triangle_match.success
    return;
end

tolerance = k_multiplier * sigma;

%% 1. Triangle 벡터 가져오기
I = triangle_match.I;
J = triangle_match.J;
K = triangle_match.K;

r_I = catalog_data.r_I(:, I);
r_J = catalog_data.r_I(:, J);
r_K = catalog_data.r_I(:, K);

%% 2. 4번째 별까지의 각거리 계산 (관측)
b_i = triangle_match.b_i;
b_j = triangle_match.b_j;
b_k = triangle_match.b_k;

theta_ir = acos(max(-1, min(1, b_i' * b_r)));
theta_jr = acos(max(-1, min(1, b_j' * b_r)));
theta_kr = acos(max(-1, min(1, b_k' * b_r)));

%% 3. k-vector로 후보 R 찾기
[cand_ir, ~, ~] = kvector_range_search(theta_ir, tolerance, catalog_data);
[cand_jr, ~, ~] = kvector_range_search(theta_jr, tolerance, catalog_data);
[cand_kr, ~, ~] = kvector_range_search(theta_kr, tolerance, catalog_data);

if isempty(cand_ir) || isempty(cand_jr) || isempty(cand_kr)
    return;
end

%% 4. 공통 R 찾기
R_candidates = [];

for m = 1:length(cand_ir)
    % I-R 쌍에서 R 후보
    if cand_ir(m).I == I
        R_temp = cand_ir(m).J;
    elseif cand_ir(m).J == I
        R_temp = cand_ir(m).I;
    else
        continue;
    end
    
    % J-R 확인
    jr_match = false;
    for n = 1:length(cand_jr)
        if (cand_jr(n).I == J && cand_jr(n).J == R_temp) || ...
           (cand_jr(n).J == J && cand_jr(n).I == R_temp)
            jr_match = true;
            break;
        end
    end
    
    if ~jr_match
        continue;
    end
    
    % K-R 확인
    kr_match = false;
    for p = 1:length(cand_kr)
        if (cand_kr(p).I == K && cand_kr(p).J == R_temp) || ...
           (cand_kr(p).J == K && cand_kr(p).I == R_temp)
            kr_match = true;
            break;
        end
    end
    
    if kr_match
        R_candidates = [R_candidates; R_temp];
    end
end

% 중복 제거
R_candidates = unique(R_candidates);

if isempty(R_candidates)
    return;
end

%% 5. Pyramid 성공
pyramid_result.success = true;
pyramid_result.R = R_candidates(1);
pyramid_result.n_candidates = length(R_candidates);

%% 6. Frequency 계산 (Pyramid 식 15)
N = catalog_data.N_stars;
k_sigma = k_multiplier * sigma;

% Triangle frequency (이미 계산됨)
f_ijk = triangle_match.frequency;

% Pyramid frequency
% f_ijkr = (N-3) * (1 - cos(kσ))/2 * f_ijk
pyramid_result.frequency = (N - 3) * (1 - cos(k_sigma)) / 2 * f_ijk;
pyramid_result.confidence = max(0, 1 - pyramid_result.frequency);

end