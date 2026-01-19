%% extract_capsule_params.m
% STL 파일에서 캡슐(Capsule) 파라미터 추출
% + 캡슐 거리 함수 + Self-collision cost 함수 포함
%
% 실행 후 params.capsules 구조체 코드가 출력됨
% → params_init.m 또는 시뮬레이션 코드에 복사하여 사용

clear; clc; close all;

%% ========== 경로 설정 ==========
meshPath = 'D:\pj2025\space_challenge\model\modeling_3d\ASM_KARI_ARM\meshes';

% STL 파일 목록 (순서: base, link1~7)
stl_files = {
    'ASM_J0.STL';   % base
    'ASM_J1.STL';   % link1
    'ASM_J2.STL';   % link2
    'ASM_J3.STL';   % link3
    'ASM_J4.STL';   % link4
    'ASM_J5.STL';   % link5
    'ASM_J6.STL';   % link6
    'ASM_J7.STL';   % link7
};

link_names = {'base', 'link1', 'link2', 'link3', 'link4', 'link5', 'link6', 'link7'};
n_links = length(stl_files);

% 각 링크의 주축 설정
% [] = PCA 사용, [x,y,z] = 수동 지정
link_axes = {
    [0 0 1];    % base (J0): z축 고정
    [];         % link1 (J1): PCA
    [];         % link2 (J2): PCA
    [];         % link3 (J3): PCA
    [];         % link4 (J4): PCA
    [];         % link5 (J5): PCA
    [];         % link6 (J6): PCA
    [0 0 1];    % link7 (J7): z축 고정
};

%% ========== 캡슐 파라미터 추출 ==========
capsules = struct('name', {}, 'p_start', {}, 'p_end', {}, 'radius', {}, ...
                  'length', {}, 'axis', {}, 'center', {});

fprintf('캡슐 파라미터 추출 시작...\n');
fprintf('============================================================\n');

figure('Name', 'Capsule Fitting', 'Position', [100 100 1400 800]);

for i = 1:n_links
    stl_path = fullfile(meshPath, stl_files{i});
    
    if ~isfile(stl_path)
        warning('파일 없음: %s', stl_path);
        continue;
    end
    
    % STL 로드
    tr = stlread(stl_path);
    vertices = tr.Points;  % Nx3
    
    % 중심 계산 (vertices centroid)
    center = mean(vertices, 1);
    vertices_centered = vertices - center;
    
    % 주축 결정: 수동 지정 또는 PCA
    if ~isempty(link_axes{i})
        % 수동 지정 축 사용
        principal_axis = link_axes{i}(:)';
        principal_axis = principal_axis / norm(principal_axis);
    else
        % PCA로 주축 찾기
        [coeff, ~, latent] = pca(vertices_centered);
        principal_axis = coeff(:, 1)';
    end
    
    % 주축 방향으로 투영
    projections = vertices_centered * principal_axis';  % Nx1
    
    % 캡슐 길이 (주축 방향 범위)
    proj_min = min(projections);
    proj_max = max(projections);
    capsule_length = proj_max - proj_min;
    
    % 캡슐 중심선 양 끝점 (로컬 좌표계)
    p_start = center + proj_min * principal_axis;
    p_end = center + proj_max * principal_axis;
    
    % 반경 계산 (주축에 수직인 거리의 최대값)
    axis_points = vertices_centered * principal_axis' * principal_axis;
    perpendicular_dist = vecnorm(vertices_centered - axis_points, 2, 2);
    
    % 반경: 95 percentile 사용 (outlier 제거) + margin
    radius = prctile(perpendicular_dist, 95) * 1.05;
    
    % 결과 저장
    capsules(i).name = link_names{i};
    capsules(i).p_start = p_start(:)';
    capsules(i).p_end = p_end(:)';
    capsules(i).radius = radius;
    capsules(i).length = capsule_length;
    capsules(i).axis = principal_axis;
    capsules(i).center = center;
    
    % 출력
    fprintf('\n[%s] %s\n', stl_files{i}, link_names{i});
    fprintf('  p_start:  [%7.4f, %7.4f, %7.4f] m\n', p_start);
    fprintf('  p_end:    [%7.4f, %7.4f, %7.4f] m\n', p_end);
    fprintf('  Length:   %.4f m,  Radius: %.4f m\n', capsule_length, radius);
    
    % 시각화
    subplot(2, 4, i);
    trisurf(tr, 'FaceColor', [0.7 0.7 0.8], 'EdgeColor', 'none', ...
        'FaceAlpha', 0.3, 'FaceLighting', 'gouraud');
    hold on;
    draw_capsule(p_start, p_end, radius, [0.2 0.6 0.9], 0.5);
    plot3(p_start(1), p_start(2), p_start(3), 'go', 'MarkerSize', 8, 'LineWidth', 2);
    plot3(p_end(1), p_end(2), p_end(3), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
    axis equal; grid on;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title(sprintf('%s (L=%.3f, R=%.3f)', link_names{i}, capsule_length, radius));
    view(45, 25); light('Position', [1 1 1]);
end

sgtitle('STL → Capsule Fitting Results');

%% ========== Self-collision 체크 쌍 ==========
collision_pairs = [];
for i = 1:n_links
    for j = i+2:n_links  % 2칸 이상 떨어진 쌍
        collision_pairs = [collision_pairs; i, j];
    end
end

%% ========== params 구조체용 출력 ==========
fprintf('\n============================================================\n');
fprintf('아래 코드를 params_init.m 또는 시뮬레이션 코드에 복사하세요:\n');
fprintf('============================================================\n\n');

fprintf('%% ========== 캡슐 파라미터 (Self-collision용) ==========\n');
fprintf('params.capsules.n = %d;\n', n_links);
fprintf('params.capsules.names = {');
for i = 1:n_links
    if i > 1, fprintf(', '); end
    fprintf('''%s''', capsules(i).name);
end
fprintf('};\n\n');

fprintf('%% [p_start(3), p_end(3), radius]\n');
fprintf('params.capsules.data = [\n');
for i = 1:n_links
    fprintf('    %9.5f, %9.5f, %9.5f, %9.5f, %9.5f, %9.5f, %7.5f;  %% %s\n', ...
        capsules(i).p_start, capsules(i).p_end, capsules(i).radius, capsules(i).name);
end
fprintf('];\n\n');

fprintf('params.capsules.collision_pairs = [\n');
for k = 1:size(collision_pairs, 1)
    fprintf('    %d, %d;  %% %s - %s\n', collision_pairs(k, 1), collision_pairs(k, 2), ...
        capsules(collision_pairs(k,1)).name, capsules(collision_pairs(k,2)).name);
end
fprintf('];\n');

fprintf('params.capsules.d_safe = 0.02;  %% 안전 거리 [m]\n');
fprintf('params.capsules.w_collision = 1000;  %% Cost 가중치\n');

%% ========== MAT 파일 저장 ==========
save('capsule_params.mat', 'capsules', 'collision_pairs');
fprintf('\n캡슐 파라미터 저장: capsule_params.mat\n');

%% ========== 테스트: 캡슐 거리 계산 ==========
fprintf('\n============================================================\n');
fprintf('캡슐 거리 함수 테스트\n');
fprintf('============================================================\n');

if n_links >= 4
    [dist, pt1, pt2] = capsule_distance(...
        capsules(2).p_start, capsules(2).p_end, capsules(2).radius, ...
        capsules(5).p_start, capsules(5).p_end, capsules(5).radius);
    fprintf('link1-link4 로컬 거리: %.4f m\n', dist);
end

fprintf('\n완료!\n');


%% ==================== 함수 정의 ====================

function [dist, closest_pt1, closest_pt2] = capsule_distance(p1_start, p1_end, r1, p2_start, p2_end, r2)
% capsule_distance: 두 캡슐 사이의 최소 거리 계산

    p1_start = p1_start(:); p1_end = p1_end(:);
    p2_start = p2_start(:); p2_end = p2_end(:);
    
    [seg_dist, closest_pt1, closest_pt2] = segment_segment_distance(p1_start, p1_end, p2_start, p2_end);
    dist = seg_dist - (r1 + r2);
end


function [dist, closest_pt1, closest_pt2] = segment_segment_distance(p1, q1, p2, q2)
% segment_segment_distance: 두 선분 사이의 최소 거리

    d1 = q1 - p1;
    d2 = q2 - p2;
    r = p1 - p2;
    
    a = dot(d1, d1);
    e = dot(d2, d2);
    f = dot(d2, r);
    
    EPSILON = 1e-10;
    
    if a < EPSILON && e < EPSILON
        s = 0; t = 0;
        closest_pt1 = p1; closest_pt2 = p2;
        dist = norm(p1 - p2);
        return;
    end
    
    if a < EPSILON
        s = 0;
        t = clamp_val(f / e, 0, 1);
    else
        c = dot(d1, r);
        if e < EPSILON
            t = 0;
            s = clamp_val(-c / a, 0, 1);
        else
            b = dot(d1, d2);
            denom = a * e - b * b;
            
            if abs(denom) > EPSILON
                s = clamp_val((b * f - c * e) / denom, 0, 1);
            else
                s = 0;
            end
            
            t = (b * s + f) / e;
            
            if t < 0
                t = 0;
                s = clamp_val(-c / a, 0, 1);
            elseif t > 1
                t = 1;
                s = clamp_val((b - c) / a, 0, 1);
            end
        end
    end
    
    closest_pt1 = p1 + s * d1;
    closest_pt2 = p2 + t * d2;
    dist = norm(closest_pt1 - closest_pt2);
end


function v = clamp_val(v, lo, hi)
    v = max(lo, min(hi, v));
end


function [cost, min_dist, collision_info] = self_collision_cost(FK, params)
% self_collision_cost: Self-collision 비용 계산 (MPPI용)

    capsule_data = params.capsules.data;
    collision_pairs = params.capsules.collision_pairs;
    n_pairs = size(collision_pairs, 1);
    
    d_safe = params.capsules.d_safe;
    w_collision = params.capsules.w_collision;
    
    cost = 0;
    min_dist = inf;
    collision_info.pairs = collision_pairs;
    collision_info.distances = zeros(n_pairs, 1);
    
    % 각 링크 캡슐을 월드 좌표로 변환
    capsules_world = cell(params.capsules.n, 1);
    
    for i = 1:params.capsules.n
        p_start_local = capsule_data(i, 1:3)';
        p_end_local = capsule_data(i, 4:6)';
        radius = capsule_data(i, 7);
        
        if i == 1
            R = FK.R_base;
            p_origin = FK.p_mount;
        else
            link_idx = i - 1;
            if link_idx <= length(FK.R)
                R = FK.R{link_idx};
                p_origin = FK.p_joint{link_idx};
            else
                R = FK.R_base;
                p_origin = FK.p_base;
            end
        end
        
        capsules_world{i}.p_start = R * p_start_local + p_origin;
        capsules_world{i}.p_end = R * p_end_local + p_origin;
        capsules_world{i}.radius = radius;
    end
    
    % 링크 쌍 거리 계산
    for k = 1:n_pairs
        i = collision_pairs(k, 1);
        j = collision_pairs(k, 2);
        
        cap_i = capsules_world{i};
        cap_j = capsules_world{j};
        
        [dist, ~, ~] = capsule_distance(cap_i.p_start, cap_i.p_end, cap_i.radius, ...
                                        cap_j.p_start, cap_j.p_end, cap_j.radius);
        
        collision_info.distances(k) = dist;
        min_dist = min(min_dist, dist);
        
        if dist < d_safe
            penetration = d_safe - dist;
            cost = cost + w_collision * penetration^2;
        end
    end
    
    % 위성 본체와 링크 충돌 체크
    sat_radius = 0.6;
    sat_center = FK.p_base;
    
    for i = 3:params.capsules.n
        cap_i = capsules_world{i};
        [dist, ~, ~] = capsule_distance(cap_i.p_start, cap_i.p_end, cap_i.radius, ...
                                        sat_center, sat_center, sat_radius);
        
        if dist < d_safe
            cost = cost + w_collision * (d_safe - dist)^2;
        end
        min_dist = min(min_dist, dist);
    end
end


function draw_capsule(p1, p2, r, color, alpha)
% draw_capsule: 캡슐 시각화

    p1 = p1(:)'; p2 = p2(:)';
    v = p2 - p1;
    h = norm(v);
    if h < 1e-6, return; end
    v = v / h;
    
    z_axis = [0 0 1];
    if abs(dot(v, z_axis)) > 0.999
        R = eye(3) * sign(v(3) + 0.5);
    else
        rot_axis = cross(z_axis, v);
        rot_axis = rot_axis / norm(rot_axis);
        angle = acos(dot(z_axis, v));
        K = [0 -rot_axis(3) rot_axis(2); rot_axis(3) 0 -rot_axis(1); -rot_axis(2) rot_axis(1) 0];
        R = eye(3) + sin(angle)*K + (1-cos(angle))*(K*K);
    end
    
    % 원기둥
    [Xc, Yc, Zc] = cylinder(r, 20);
    Zc = Zc * h;
    for idx = 1:numel(Xc)
        pt = R * [Xc(idx); Yc(idx); Zc(idx)] + p1(:);
        Xc(idx) = pt(1); Yc(idx) = pt(2); Zc(idx) = pt(3);
    end
    surf(Xc, Yc, Zc, 'FaceColor', color, 'EdgeColor', 'none', 'FaceAlpha', alpha);
    
    % 반구
    [Xs, Ys, Zs] = sphere(15);
    Xs = Xs * r; Ys = Ys * r; Zs = Zs * r;
    
    Xs1 = Xs; Ys1 = Ys; Zs1 = Zs;
    for idx = 1:numel(Xs1)
        if Zs(idx) <= 0
            pt = R * [Xs1(idx); Ys1(idx); Zs1(idx)] + p1(:);
            Xs1(idx) = pt(1); Ys1(idx) = pt(2); Zs1(idx) = pt(3);
        else
            Xs1(idx) = NaN; Ys1(idx) = NaN; Zs1(idx) = NaN;
        end
    end
    surf(Xs1, Ys1, Zs1, 'FaceColor', color, 'EdgeColor', 'none', 'FaceAlpha', alpha);
    
    Xs2 = Xs; Ys2 = Ys; Zs2 = Zs;
    for idx = 1:numel(Xs2)
        if Zs(idx) >= 0
            pt = R * [Xs2(idx); Ys2(idx); Zs2(idx)] + p2(:);
            Xs2(idx) = pt(1); Ys2(idx) = pt(2); Zs2(idx) = pt(3);
        else
            Xs2(idx) = NaN; Ys2(idx) = NaN; Zs2(idx) = NaN;
        end
    end
    surf(Xs2, Ys2, Zs2, 'FaceColor', color, 'EdgeColor', 'none', 'FaceAlpha', alpha);
end