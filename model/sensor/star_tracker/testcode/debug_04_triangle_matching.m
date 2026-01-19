%% debug_04_triangle_matching_fixed_v2.m
clear; close all; clc;

fprintf('=== Triangle 매칭 테스트 (v2) ===\n\n');

%% 1. 카탈로그 로드
load('star_catalog_kvector.mat');
fprintf('카탈로그 로드: %d개 별\n\n', catalog_data.N_stars);

%% 2. 실제 FOV 내 별 3개 선택
rng(123);
center_ra = rand() * 2*pi;
center_dec = (rand()-0.5) * pi;

center_dir = [cos(center_ra)*cos(center_dec); 
              sin(center_ra)*cos(center_dec); 
              sin(center_dec)];

FOV_half = catalog_data.FOV_rad / 2;
angles_from_center = acos(catalog_data.r_I' * center_dir);
in_fov_idx = find(angles_from_center < FOV_half);

fprintf('중심 방향: RA=%.2f°, DEC=%.2f°\n', rad2deg(center_ra), rad2deg(center_dec));
fprintf('FOV 내 별: %d개\n\n', length(in_fov_idx));

if length(in_fov_idx) < 3
    error('FOV 내 별이 3개 미만');
end

% 3개 선택
selected_idx = in_fov_idx(randperm(length(in_fov_idx), 3));
I_true = selected_idx(1);
J_true = selected_idx(2);
K_true = selected_idx(3);

fprintf('선택된 별:\n');
fprintf('  I = %d (ID: %d, mag: %.2f)\n', I_true, ...
    catalog_data.star_catalog.ID(I_true), ...
    catalog_data.star_catalog.Magnitude(I_true));
fprintf('  J = %d (ID: %d, mag: %.2f)\n', J_true, ...
    catalog_data.star_catalog.ID(J_true), ...
    catalog_data.star_catalog.Magnitude(J_true));
fprintf('  K = %d (ID: %d, mag: %.2f)\n\n', K_true, ...
    catalog_data.star_catalog.ID(K_true), ...
    catalog_data.star_catalog.Magnitude(K_true));

%% 3. 실제 각거리
r_I = catalog_data.r_I(:, I_true);
r_J = catalog_data.r_I(:, J_true);
r_K = catalog_data.r_I(:, K_true);

theta_IJ = acos(r_I' * r_J);
theta_IK = acos(r_I' * r_K);
theta_JK = acos(r_J' * r_K);

fprintf('실제 각거리:\n');
fprintf('  θ_IJ = %.4f°\n', rad2deg(theta_IJ));
fprintf('  θ_IK = %.4f°\n', rad2deg(theta_IK));
fprintf('  θ_JK = %.4f°\n\n', rad2deg(theta_JK));

%% 4. 자세 생성
z_body = center_dir;
x_body = cross([0;0;1], z_body);
if norm(x_body) < 1e-6
    x_body = [1;0;0];
end
x_body = x_body / norm(x_body);
y_body = cross(z_body, x_body);

R_I2B = [x_body, y_body, z_body]';

%% 5. 관측 벡터
sigma = 50e-6;
k_mult = 6.4;

b_I = R_I2B * r_I + sigma * randn(3,1);
b_J = R_I2B * r_J + sigma * randn(3,1);
b_K = R_I2B * r_K + sigma * randn(3,1);

b_I = b_I / norm(b_I);
b_J = b_J / norm(b_J);
b_K = b_K / norm(b_K);

theta_ij_obs = acos(b_I' * b_J);
theta_ik_obs = acos(b_I' * b_K);
theta_jk_obs = acos(b_J' * b_K);

fprintf('관측 각거리:\n');
fprintf('  θ_ij = %.4f° (오차: %.2f arcsec)\n', ...
    rad2deg(theta_ij_obs), rad2deg(theta_ij_obs - theta_IJ)*3600);
fprintf('  θ_ik = %.4f° (오차: %.2f arcsec)\n', ...
    rad2deg(theta_ik_obs), rad2deg(theta_ik_obs - theta_IK)*3600);
fprintf('  θ_jk = %.4f° (오차: %.2f arcsec)\n\n', ...
    rad2deg(theta_jk_obs), rad2deg(theta_jk_obs - theta_JK)*3600);

%% 6. 중간 디버그: k-vector 후보 확인
tolerance = k_mult * sigma;
fprintf('=== k-vector 후보 확인 ===\n');
fprintf('Tolerance: %.6f rad (%.2f arcsec)\n\n', tolerance, rad2deg(tolerance)*3600);

[cand_ij, ~, ~] = kvector_range_search(theta_ij_obs, tolerance, catalog_data);
[cand_ik, ~, ~] = kvector_range_search(theta_ik_obs, tolerance, catalog_data);
[cand_jk, ~, ~] = kvector_range_search(theta_jk_obs, tolerance, catalog_data);

fprintf('θ_ij (%.4f°) 후보: %d개\n', rad2deg(theta_ij_obs), length(cand_ij));
fprintf('θ_ik (%.4f°) 후보: %d개\n', rad2deg(theta_ik_obs), length(cand_ik));
fprintf('θ_jk (%.4f°) 후보: %d개\n\n', rad2deg(theta_jk_obs), length(cand_jk));

% Ground truth가 후보에 있는지 확인
ij_found = false;
ik_found = false;
jk_found = false;

for m = 1:length(cand_ij)
    if (cand_ij(m).I == I_true && cand_ij(m).J == J_true) || ...
       (cand_ij(m).I == J_true && cand_ij(m).J == I_true)
        ij_found = true;
        break;
    end
end

for m = 1:length(cand_ik)
    if (cand_ik(m).I == I_true && cand_ik(m).J == K_true) || ...
       (cand_ik(m).I == K_true && cand_ik(m).J == I_true)
        ik_found = true;
        break;
    end
end

for m = 1:length(cand_jk)
    if (cand_jk(m).I == J_true && cand_jk(m).J == K_true) || ...
       (cand_jk(m).I == K_true && cand_jk(m).J == J_true)
        jk_found = true;
        break;
    end
end

fprintf('Ground truth 후보 포함 여부:\n');
fprintf('  θ_ij: %s\n', char(10004*ij_found + 10008*~ij_found));
fprintf('  θ_ik: %s\n', char(10004*ik_found + 10008*~ik_found));
fprintf('  θ_jk: %s\n\n', char(10004*jk_found + 10008*~jk_found));

%% 7. Triangle 매칭
fprintf('=== Triangle 매칭 ===\n');
tic;
match_result = match_triangle(b_I, b_J, b_K, catalog_data, sigma, k_mult);
t_match = toc;

fprintf('실행시간: %.2f ms\n', t_match*1000);
fprintf('매칭 성공: %s\n', char(10004*match_result.success + 10008*(~match_result.success)));

if match_result.success
    fprintf('Unique: %s\n', char(10004*match_result.unique + 10008*(~match_result.unique)));
    fprintf('매칭 개수: %d\n', match_result.n_matches);
    
    correct_I = (match_result.I == I_true);
    correct_J = (match_result.J == J_true);
    correct_K = (match_result.K == K_true);
    all_correct = correct_I && correct_J && correct_K;
    
    fprintf('\n매칭 결과:\n');
    fprintf('  I: %d (true: %d) %s\n', match_result.I, I_true, char(10004*correct_I + 10008*~correct_I));
    fprintf('  J: %d (true: %d) %s\n', match_result.J, J_true, char(10004*correct_J + 10008*~correct_J));
    fprintf('  K: %d (true: %d) %s\n', match_result.K, K_true, char(10004*correct_K + 10008*~correct_K));
    
    fprintf('\nFrequency: %.2e\n', match_result.frequency);
    fprintf('Confidence: %.2f%%\n\n', match_result.confidence*100);
    
    %% 8. QUEST 자세 추정
    if all_correct
        fprintf('=== QUEST 자세 추정 ===\n');
        
        b_obs = [b_I, b_J, b_K];
        r_cat = [r_I, r_J, r_K];
        
        [R_est, q_est] = quest_attitude(b_obs, r_cat);
        
        % 오차
        R_err = R_est * R_I2B';
        q_err = DCM2Quat(R_err);
        angle_err = 2 * acos(min(1, abs(q_err(1))));
        
        fprintf('자세 오차: %.4f° (%.2f arcmin)\n', ...
            rad2deg(angle_err), rad2deg(angle_err)*60);
        
        if angle_err < deg2rad(0.1)
            fprintf('✓ 고정밀 자세 추정 성공\n');
        elseif angle_err < deg2rad(1.0)
            fprintf('✓ 자세 추정 성공\n');
        else
            fprintf('⚠ 자세 오차 큼\n');
        end
    else
        fprintf('⚠ 잘못된 매칭 - QUEST 건너뜀\n');
    end
else
    fprintf('✗ 매칭 실패\n');
end

fprintf('\n=== 테스트 완료 ===\n');