%% smart_triangle_selection.m (논문 Smart Loop )
function triangle_list = smart_triangle_selection(n, ~, ~, max_triangles)
%
% 이 함수는 논문에서 제안한 "smart technique" [cite: 210]을 구현합니다.
% "three inner loops concept" 을 기반으로 하며,
% 이 방식은 스파이크(잘못된 별)에 머무는 것을 방지합니다. [cite: 236]
%
% Inputs:
%   n            - 총 별 개수
%   magnitudes   - (사용 안 함. 논문 로직은 밝기 무관)
%   angle_matrix - (사용 안 함. 논문 로직은 각거리 무관)
%   max_triangles - 최대 시도 개수
%
% Output:
%   triangle_list - Mx3 [i,j,k]

if nargin < 4
    max_triangles = 10;  % 메인 루프에서 10개 시도
end

% nC3 (총 조합)이 max_triangles 보다 작을 수 있으므로
% 최대 개수를 동적으로 계산
total_combos = nchoosek(n, 3);
if total_combos < max_triangles
    max_triangles = total_combos;
end

triangle_list = zeros(max_triangles, 3); % 메모리 사전 할당
count = 0;

% --- 논문의 Pseudocode [cite: 212-221] ---
for dj = 1:(n-2)
    for dk = 1:(n-dj-1)
        for i = 1:(n-dj-dk)
            j = i + dj;
            k = j + dk;
            
            count = count + 1;
            
            triangle_list(count, :) = [i, j, k];

            if count >= max_triangles
                % 최대 개수 도달 시 모든 루프 종료
                return;
            end
        end
    end
end
% -----------------------------------------

end