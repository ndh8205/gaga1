function star_id = identify_remaining_star(b_p, identified_stars, b_identified, catalog_data, sigma, k_multiplier)
% identify_remaining_star - 이미 식별된 별들을 이용해 추가 별 식별
%
% Inputs:
%   b_p               - 식별할 별 벡터
%   identified_stars  - 이미 식별된 별들의 카탈로그 ID
%   b_identified      - 이미 식별된 별들의 관측 벡터
%   catalog_data      - k-vector 카탈로그
%   sigma, k_multiplier - 파라미터
%
% Output:
%   star_id - 카탈로그 인덱스 (실패 시 [])

star_id = [];
tolerance = k_multiplier * sigma;

% 최소 2개 별과 매칭
if length(identified_stars) < 2
    return;
end

%% 첫 2개 식별된 별과의 각거리
theta_p1 = acos(max(-1, min(1, b_p' * b_identified(:,1))));
theta_p2 = acos(max(-1, min(1, b_p' * b_identified(:,2))));

%% k-vector 검색
[cand_1, ~, ~] = kvector_range_search(theta_p1, tolerance, catalog_data);
[cand_2, ~, ~] = kvector_range_search(theta_p2, tolerance, catalog_data);

if isempty(cand_1) || isempty(cand_2)
    return;
end

%% 공통 별 찾기
star1_id = identified_stars(1);
star2_id = identified_stars(2);

candidates = [];

for m = 1:length(cand_1)
    if cand_1(m).I == star1_id
        P_temp = cand_1(m).J;
    elseif cand_1(m).J == star1_id
        P_temp = cand_1(m).I;
    else
        continue;
    end
    
    % cand_2 확인
    for n = 1:length(cand_2)
        if (cand_2(n).I == star2_id && cand_2(n).J == P_temp) || ...
           (cand_2(n).J == star2_id && cand_2(n).I == P_temp)
            candidates = [candidates; P_temp];
            break;
        end
    end
end

candidates = unique(candidates);

if ~isempty(candidates)
    star_id = candidates(1);
end

end