function catalog_data = build_star_catalog_kvector(catalog_file, FOV_deg, mag_threshold)
% build_star_catalog_kvector - k-vector 별 카탈로그 DB 생성
%
% Inputs:
%   catalog_file   - Hipparcos CSV 파일 경로
%   FOV_deg        - Star Tracker FOV 대각선 (degrees)
%   mag_threshold  - 최대 magnitude
%
% Output:
%   catalog_data   - struct
%       .star_catalog   - 별 카탈로그 (RA, DEC, Mag, ID)
%       .pairs          - 별쌍 정보
%       .sorted_pairs   - 정렬된 별쌍
%       .k_vector       - k-vector
%       .m, q           - k-vector 파라미터
%       .FOV_rad        - FOV (radians)

fprintf('k-vector 카탈로그 구축 시작...\n');

%% 1. 카탈로그 로드
opts = detectImportOptions(catalog_file);
opts.VariableNamingRule = 'preserve';
star_table = readtable(catalog_file, opts);

% Magnitude 필터링
valid_idx = star_table.Magnitude <= mag_threshold;
star_catalog.RA = star_table.RA(valid_idx);
star_catalog.DEC = star_table.DE(valid_idx);
star_catalog.Magnitude = star_table.Magnitude(valid_idx);
star_catalog.ID = star_table.('Star ID')(valid_idx);

N_stars = length(star_catalog.ID);
fprintf('  총 별 개수: %d (magnitude <= %.1f)\n', N_stars, mag_threshold);

%% 2. RA/DEC → 단위벡터 변환
r_I = zeros(3, N_stars);
for i = 1:N_stars
    ra = star_catalog.RA(i);
    dec = star_catalog.DEC(i);
    r_I(:,i) = [cos(ra)*cos(dec); sin(ra)*cos(dec); sin(dec)];
end

%% 3. FOV 내 모든 별쌍 각거리 계산
FOV_rad = deg2rad(FOV_deg);
max_angle = FOV_rad;

fprintf('  별쌍 각거리 계산 중...\n');
pair_count = 0;
max_pairs = N_stars * (N_stars - 1) / 2;
pairs_data = struct('I', cell(max_pairs,1), 'J', cell(max_pairs,1), 'angle', cell(max_pairs,1));

for i = 1:N_stars-1
    if mod(i, 500) == 0
        fprintf('    진행: %d/%d\n', i, N_stars);
    end
    
    for j = i+1:N_stars
        % 각거리 계산
        cos_angle = r_I(:,i)' * r_I(:,j);
        cos_angle = max(-1, min(1, cos_angle)); % clipping
        angle = acos(cos_angle);
        
        % FOV 내 별쌍만 저장
        if angle <= max_angle
            pair_count = pair_count + 1;
            pairs_data(pair_count).I = i;
            pairs_data(pair_count).J = j;
            pairs_data(pair_count).angle = angle;
        end
    end
end

% 실제 사용된 크기로 trim
pairs_data = pairs_data(1:pair_count);
fprintf('  FOV 내 별쌍: %d개\n', pair_count);

%% 4. 각거리 기준 정렬
angles = [pairs_data.angle]';
[sorted_angles, sort_idx] = sort(angles);
sorted_pairs = pairs_data(sort_idx);

%% 5. k-vector 구축
fprintf('  k-vector 구축 중...\n');
n = pair_count;
ymin = sorted_angles(1);
ymax = sorted_angles(end);

% δε 계산
eps_machine = 2.22e-16;
delta_eps = (n - 1) * eps_machine;

% m, q 계산
m = (ymax - ymin + 2*delta_eps) / (n - 1);
q = ymin - m - delta_eps;

% k-vector 초기화
k_vector = zeros(n, 1);
k_vector(1) = 0;
k_vector(n) = n;

% k-vector 구축
for i = 2:n-1
    z_i = m * i + q;
    
    % z_i 이하의 원소 개수 찾기
    j = find(sorted_angles <= z_i, 1, 'last');
    if isempty(j)
        k_vector(i) = 0;
    else
        k_vector(i) = j;
    end
end

fprintf('  k-vector 구축 완료!\n');

%% 6. 출력 구조체 생성
catalog_data.star_catalog = star_catalog;
catalog_data.r_I = r_I;
catalog_data.pairs = pairs_data;
catalog_data.sorted_pairs = sorted_pairs;
catalog_data.sorted_angles = sorted_angles;
catalog_data.k_vector = k_vector;
catalog_data.m = m;
catalog_data.q = q;
catalog_data.FOV_rad = FOV_rad;
catalog_data.N_stars = N_stars;
catalog_data.N_pairs = pair_count;

% 고속 배열 기반 검색을 위한 필드 추가 (kvector_range_search_fast용)
catalog_data.pairs_I = [sorted_pairs.I]';
catalog_data.pairs_J = [sorted_pairs.J]';
catalog_data.pairs_angle = sorted_angles;

fprintf('k-vector 카탈로그 구축 완료!\n\n');
end