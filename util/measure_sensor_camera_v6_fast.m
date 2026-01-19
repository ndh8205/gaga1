function [z_measured, z_true, camera_params] = measure_sensor_camera_v6_fast(Xk, X_target, model_data, camera_params, orbit_normal_vec, dt)
% V6 FAST: 완전 벡터화된 고속 카메라 센서 측정 모델
%
% Inputs:
%   Xk                - Deputy 상태벡터 [r_B^I(3); v_B^I(3); q_I^B(4); omega_B/I^B(3)]
%   X_target          - Chief 상태벡터 [r_A^I(3); v_A^I(3); q_I^A(4); omega_A/I^A(3)]
%   model_data        - struct with .vertices (3xN) and .faces (Mx3)
%   camera_params     - 카메라 파라미터 구조체
%   orbit_normal_vec  - 궤도 평면 법선 벡터 (3x1)
%   dt                - 샘플 간격 [s]

% ---------- 파라미터 초기화 ----------
if nargin < 4 || isempty(camera_params)
    camera_params = initialize_camera_params();
end

% ---------- 모델 데이터 처리 ----------
if isstruct(model_data)
    points_chief_body = model_data.vertices;
    if isfield(model_data, 'faces')
        faces = model_data.faces;
        has_faces = true;
    else
        faces = [];
        has_faces = false;
    end
else
    points_chief_body = model_data;
    faces = [];
    has_faces = false;
end

% ---------- 상태 분해 (기존과 동일) ----------
r_B_I = Xk(1:3);        
v_B_I = Xk(4:6);        
q_I2B = Xk(7:10);       
q_I2B = q_I2B(:) / norm(q_I2B);
omega_BI_B = Xk(11:13); 

r_A_I = X_target(1:3);
v_A_I = X_target(4:6);
q_I2A = X_target(7:10);
q_I2A = q_I2A(:) / norm(q_I2A);
omega_AI_A = X_target(11:13);

% ---------- 좌표 변환 행렬 계산 ----------
r_hat = r_A_I / norm(r_A_I);
h_hat = cross(r_A_I, v_A_I) / norm(cross(r_A_I, v_A_I));
t_hat = cross(h_hat, r_hat);
DCM_L2I = [r_hat, t_hat, h_hat];
DCM_I2L = DCM_L2I';

r_rel_I = r_B_I - r_A_I;
r_rel_L = DCM_I2L * r_rel_I;
current_distance = norm(r_rel_L);

q_I2L = DCM2Quat(DCM_I2L);
q_L2B = q2q_mult(q_I2B, inv_q(q_I2L));
q_B2A = q2q_mult(q_I2A, inv_q(q_I2B));
q_B2A = q_B2A / norm(q_B2A);

R_B2A = GetDCM_QUAT(q_B2A);
R_A2B = R_B2A';

DCM_L2B = GetDCM_QUAT(q_L2B);
t_A_in_B = -DCM_L2B * r_rel_L;

R_B2C = [1, 0, 0; 0, 0, -1; 0, 1, 0];
R_A2C = R_B2C * R_A2B;
t_A_in_C = R_B2C * t_A_in_B;

% ---------- 카메라 파라미터 ----------
K_ideal = camera_params.intrinsic.K_ideal;
image_width = camera_params.intrinsic.image_width;
image_height = camera_params.intrinsic.image_height;

% ========== 벡터화된 Vertex 처리 ==========
num_points = size(points_chief_body, 2);

% 모든 vertices를 한번에 변환 (3xN 행렬 연산)
p_cam_all = R_A2C * points_chief_body + t_A_in_C;  % 3xN

% 벡터화된 투영 (element-wise 연산)
z_pos = p_cam_all(3,:) > 1e-6;  % depth check
u_all = zeros(1, num_points);
v_all = zeros(1, num_points);

% 유효한 depth만 투영
valid_idx = find(z_pos);
if ~isempty(valid_idx)
    u_all(valid_idx) = K_ideal(1,1) * p_cam_all(1,valid_idx) ./ p_cam_all(3,valid_idx) + K_ideal(1,3);
    v_all(valid_idx) = K_ideal(2,2) * p_cam_all(2,valid_idx) ./ p_cam_all(3,valid_idx) + K_ideal(2,3);
end

pixels_true = [u_all; v_all];
depths = p_cam_all(3,:);

% 벡터화된 FOV 체크
margin = 50;
visible_true = z_pos & ...
               (u_all >= -margin) & (u_all <= image_width+margin) & ...
               (v_all >= -margin) & (v_all <= image_height+margin);

% ========== 벡터화된 Face 처리 ==========
if has_faces && ~isempty(faces)
    num_faces = size(faces, 1);
    
    % Face vertex 인덱스 추출
    v1_idx = faces(:,1);
    v2_idx = faces(:,2);  
    v3_idx = faces(:,3);
    
    % 유효한 인덱스 체크
    valid_face_idx = (v1_idx <= num_points) & (v2_idx <= num_points) & (v3_idx <= num_points);
    v1_idx = v1_idx(valid_face_idx);
    v2_idx = v2_idx(valid_face_idx);
    v3_idx = v3_idx(valid_face_idx);
    num_valid_faces = length(v1_idx);
    
    if num_valid_faces > 0
        % 벡터화된 face vertices 추출 (3xM)
        v1 = p_cam_all(:, v1_idx);
        v2 = p_cam_all(:, v2_idx);
        v3 = p_cam_all(:, v3_idx);
        
        % 벡터화된 face 중심과 depth 계산
        face_centers = (v1 + v2 + v3) / 3;  % 3xM
        face_depths = face_centers(3,:);     % 1xM
        
        % 벡터화된 normal 계산
        edge1 = v2 - v1;  % 3xM
        edge2 = v3 - v1;  % 3xM
        
        % Cross product (벡터화)
        normals_x = edge1(2,:) .* edge2(3,:) - edge1(3,:) .* edge2(2,:);
        normals_y = edge1(3,:) .* edge2(1,:) - edge1(1,:) .* edge2(3,:);
        normals_z = edge1(1,:) .* edge2(2,:) - edge1(2,:) .* edge2(1,:);
        face_normals = [normals_x; normals_y; normals_z];  % 3xM
        
        % 벡터화된 backface culling
        dot_products = sum(face_normals .* face_centers, 1);  % 1xM
        
        % 벡터화된 vertex 가시성 체크
        v1_vis = visible_true(v1_idx);
        v2_vis = visible_true(v2_idx);
        v3_vis = visible_true(v3_idx);
        vertex_vis_count = v1_vis + v2_vis + v3_vis;
        
        % 최종 face 가시성 (모든 조건 동시 체크)
        face_visible_vec = (dot_products < 0) & ...
                          (face_depths > 0) & ...
                          (vertex_vis_count >= 2);
        
        % 전체 face에 대한 가시성 배열 생성
        face_visible = false(num_faces, 1);
        face_visible(valid_face_idx) = face_visible_vec;
        
        % Depth sorting (가시적인 faces만)
        visible_face_indices = find(face_visible);
        if ~isempty(visible_face_indices)
            [~, sort_idx] = sort(face_depths(face_visible_vec), 'descend');
            depth_order = visible_face_indices(sort_idx);
        else
            depth_order = [];
        end
        
        % Face 정보 저장
        face_info = struct();
        face_info.faces = faces;
        face_info.visible = face_visible;
        face_info.depths = zeros(num_faces, 1);
        face_info.depths(valid_face_idx) = face_depths;
        face_info.depth_order = depth_order;
        face_info.normals = zeros(3, num_faces);
        face_info.normals(:, valid_face_idx) = face_normals;
        
        if isfield(model_data, 'face_colors') && ~isempty(model_data.face_colors)
            face_info.face_colors = model_data.face_colors;
        else
            face_info.face_colors = repmat([0.7, 0.7, 0.8], num_faces, 1);
        end
        
        if isfield(model_data, 'vertex_colors') && ~isempty(model_data.vertex_colors)
            face_info.vertex_colors = model_data.vertex_colors;
        end

    else
        face_info = [];
    end
else
    face_info = [];
end

% ========== 간소화된 Occlusion (선택적) ==========
if current_distance < 0.5 && sum(visible_true) > 100  % 근거리에서만
    % Z-buffer 기반 간단한 occlusion
    occlusion_grid_size = 50;  % 50x50 그리드
    z_buffer = inf(occlusion_grid_size, occlusion_grid_size);
    
    % 그리드 좌표로 변환
    u_grid = floor(u_all(visible_true) * occlusion_grid_size / image_width) + 1;
    v_grid = floor(v_all(visible_true) * occlusion_grid_size / image_height) + 1;
    
    % 범위 제한
    u_grid = max(1, min(occlusion_grid_size, u_grid));
    v_grid = max(1, min(occlusion_grid_size, v_grid));
    
    % Z-buffer 업데이트 (벡터화)
    visible_idx = find(visible_true);
    for i = 1:length(visible_idx)
        idx = visible_idx(i);
        grid_idx = sub2ind([occlusion_grid_size, occlusion_grid_size], v_grid(i), u_grid(i));
        if depths(idx) < z_buffer(grid_idx) - 0.0001
            % 이미 더 가까운 점이 있음
            visible_true(idx) = false;
        else
            z_buffer(grid_idx) = depths(idx);
        end
    end
end

% ========== 센서 오차 모델 (벡터화) ==========
K_measured = K_ideal;
K_measured(1,1) = K_ideal(1,1) * (1 + camera_params.calib.fx_error);
K_measured(2,2) = K_ideal(2,2) * (1 + camera_params.calib.fy_error);
K_measured(1,3) = K_ideal(1,3) + camera_params.calib.cx_error;
K_measured(2,3) = K_ideal(2,3) + camera_params.calib.cy_error;

k1 = camera_params.distortion.k1;
k2 = camera_params.distortion.k2;
p1 = camera_params.distortion.p1;
p2 = camera_params.distortion.p2;

pixels_measured = zeros(2, num_points);
visible_measured = false(1, num_points);

% 벡터화된 왜곡 적용 (visible points만)
vis_idx = find(visible_true);
if ~isempty(vis_idx)
    cx = K_ideal(1, 3);
    cy = K_ideal(2, 3);
    fx = K_ideal(1, 1);
    fy = K_ideal(2, 2);
    
    x_norm = (pixels_true(1, vis_idx) - cx) / fx;
    y_norm = (pixels_true(2, vis_idx) - cy) / fy;
    r2 = x_norm.^2 + y_norm.^2;
    
    radial_factor = 1 + k1*r2 + k2*r2.^2;
    x_dist = x_norm .* radial_factor + 2*p1*x_norm.*y_norm + p2*(r2 + 2*x_norm.^2);
    y_dist = y_norm .* radial_factor + p1*(r2 + 2*y_norm.^2) + 2*p2*x_norm.*y_norm;
    
    u_meas = K_measured(1,1) * x_dist + K_measured(1,3);
    v_meas = K_measured(2,2) * y_dist + K_measured(2,3);
    
    % 노이즈 추가 (벡터화)
    noise = camera_params.noise.pixel_std * randn(2, length(vis_idx));
    pixels_measured(:, vis_idx) = [u_meas; v_meas] + noise;
    
    % 측정 가시성 체크
    visible_measured(vis_idx) = (pixels_measured(1, vis_idx) >= -10) & ...
                                (pixels_measured(1, vis_idx) <= image_width+10) & ...
                                (pixels_measured(2, vis_idx) >= -10) & ...
                                (pixels_measured(2, vis_idx) <= image_height+10);
end

% ---------- 파라미터 업데이트 (기존과 동일) ----------
tau_calib = camera_params.calib.drift_time_constant;
sigma_calib = sqrt(camera_params.calib.drift_noise_density);

camera_params.calib.fx_error = exp(-dt/tau_calib) * camera_params.calib.fx_error + ...
    sigma_calib * randn * sqrt(dt);
camera_params.calib.fy_error = exp(-dt/tau_calib) * camera_params.calib.fy_error + ...
    sigma_calib * randn * sqrt(dt);

% ---------- 출력 구조체 생성 ----------
if has_faces && ~isempty(face_info)
    z_true = struct();
    z_true.pixels = pixels_true;
    z_true.visible = visible_true;
    z_true.face_info = face_info;
    z_true.has_faces = true;
    
    z_measured = struct();
    z_measured.pixels = pixels_measured;
    z_measured.visible = visible_measured;
    z_measured.face_info = face_info;
    z_measured.has_faces = true;
else
    z_true = [pixels_true; visible_true];
    z_measured = [pixels_measured; visible_measured];
end

end

% ========================================================================
% 카메라 파라미터 초기화 (기존과 동일)
% ========================================================================
function params = initialize_camera_params()
    deg = pi/180;
    
    params.intrinsic.image_width = 1024;
    params.intrinsic.image_height = 1024;
    params.intrinsic.fov = 0.1;  % [rad]
    
    f = params.intrinsic.image_width / (2 * tan(params.intrinsic.fov/2));
    params.intrinsic.K_ideal = [f, 0, params.intrinsic.image_width/2;
                                 0, f, params.intrinsic.image_height/2;
                                 0, 0, 1];
    
    params.calib.fx_error = 1e-3 * randn;
    params.calib.fy_error = 1e-3 * randn;
    params.calib.cx_error = 1.0 * randn;
    params.calib.cy_error = 1.0 * randn;
    
    params.calib.drift_time_constant = 86400;
    params.calib.drift_noise_density = 1e-12;
    
    params.distortion.k1 = 1e-7;
    params.distortion.k2 = 1e-10;
    params.distortion.p1 = 1e-8;
    params.distortion.p2 = 1e-8;
    
    params.distortion.drift_time_constant = 864000;
    params.distortion.drift_noise_density = 1e-18;
    
    params.noise.pixel_std = 0.5;
    
    params.geom.r_BC_B = [0; 0; 0];
    params.geom.q_B_C = [1; 0; 0; 0];
end