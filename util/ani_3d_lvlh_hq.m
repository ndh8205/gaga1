function ani_3d_lvlh_hq_v2(sensor_data, camera_params, LVLH_pos, colors, r_init_B, t1, chief_obj_path)
% ANI_3D_LVLH_HQ_V2: 고퀄리티 OBJ/STL + hgtransform 기반 시각화
% 스케일 자동 보정 및 카메라 뷰 안정화 적용 버전

if nargin < 7 || isempty(chief_obj_path)
    % 테스트용 경로 (경로가 없으면 큐브로 대체됨)
    chief_obj_path = 'model\modeling_3d\1015_3U_CNU\1015_3U_CNU.obj';
end

%% 1. 파라미터 설정 (스케일 중요!)
skip_frame = 10;
fov_camera = camera_params.intrinsic.fov;

% [핵심] 시각화 스케일 팩터 (실제 크기보다 훨씬 키워야 km 단위 궤도에서 보임)
% r_init_B(km)가 0.5km(500m) 정도일 때, 위성이 보이려면 최소 10~20m 급으로 뻥튀기해야 함
% OBJ가 mm 단위라면: 0.001(m변환) * 0.001(km변환) * 10000(확대) 
viz_scale_factor = 0.001 * 0.001 * 20000; 

fprintf('=== 고퀄리티 LVLH 애니메이션 V2 시작 ===\n');
fprintf('Chief OBJ Path: %s\n', chief_obj_path);

%% 2. Figure 초기화
fig = figure('Name', 'LVLH Integrated HQ Animation', ...
    'Position', [50, 50, 1800, 900], 'Color', 'k', ...
    'Renderer', 'opengl', 'InvertHardcopy', 'off');

%% [Left Plot] 3D LVLH Orbit View
ax1 = subplot(1, 2, 1, 'Parent', fig);
set(ax1, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w', ...
    'DataAspectRatio', [1 1 1], 'NextPlot', 'add');
grid(ax1, 'off'); axis(ax1, 'off'); % 깔끔하게 축 제거
view(ax1, -45, 20);

% 축 범위 고정 (흔들림 방지)
limit_km = r_init_B * 1.2;
xlim(ax1, [-limit_km, limit_km]);
ylim(ax1, [-limit_km, limit_km]);
zlim(ax1, [-limit_km, limit_km]);
axis(ax1, 'vis3d', 'manual');

% 조명 설정
light('Parent', ax1, 'Position', [limit_km, limit_km, limit_km], 'Style', 'local', 'Color', [1 1 0.9]);
light('Parent', ax1, 'Position', [-limit_km, -limit_km, 0], 'Style', 'local', 'Color', [0.3 0.3 0.4]);
lighting(ax1, 'gouraud');

% LVLH Reference Frame 표시
axis_len = r_init_B * 0.5;
quiver3(ax1, 0,0,0, axis_len,0,0, 'Color', 'r', 'LineWidth', 1); text(ax1, axis_len,0,0,'X', 'Color','r');
quiver3(ax1, 0,0,0, 0,axis_len,0, 'Color', 'g', 'LineWidth', 1); text(ax1, 0,axis_len,0,'Y', 'Color','g');
quiver3(ax1, 0,0,0, 0,0,axis_len, 'Color', 'b', 'LineWidth', 1); text(ax1, 0,0,axis_len,'Z', 'Color','b');

% 궤도 선 (Trail)
h_orbit_trail = plot3(ax1, NaN, NaN, NaN, 'Color', colors.deputy, 'LineWidth', 1.5);
h_orbit_future = plot3(ax1, LVLH_pos(1,:), LVLH_pos(2,:), LVLH_pos(3,:), ...
    ':', 'Color', [0.3 0.3 0.3], 'LineWidth', 0.5); % 전체 예상 경로

%% [Right Plot] Sensor Camera View
ax2 = subplot(1, 2, 2, 'Parent', fig);
set(ax2, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w', ...
    'DataAspectRatio', [1 1 1], 'NextPlot', 'add');
grid(ax2, 'off'); axis(ax2, 'off');

% [핵심] 카메라는 원점에 고정하고, 물체를 움직임
% V2 코드 분석 결과: Body Y축이 Bore-sight(광축)임.
% 따라서 카메라를 원점에 두고 +Y 방향을 바라보게 설정.
campos(ax2, [0, 0, 0]);
camtarget(ax2, [0, 1, 0]); % +Y 방향을 바라봄
camup(ax2, [0, 0, 1]);     % +Z 방향이 위쪽

% FOV 설정
camva(ax2, rad2deg(fov_camera) * 1.2); % FOV 적용 (약간 여유있게)

% 조명 (카메라 위치에서 비추는 헤드라이트)
light('Parent', ax2, 'Position', [0, 0, 0], 'Style', 'local', 'Color', [1 1 1]);

%% 3. 객체 로드 및 hgtransform 생성

% --- 3.1 Chief Satellite (Left View & Right View) ---
% OBJ 로드
try
    [objData, objMats, objAssign] = readObjWithCache(chief_obj_path);
    is_obj_loaded = true;
catch
    warning('OBJ 로드 실패. 큐브로 대체합니다.');
    is_obj_loaded = false;
end

% (Left View용 Chief) - 원점에 위치, 회전만 함
hg_chief_L = hgtransform('Parent', ax1);
if is_obj_loaded
    renderObjToGroup(hg_chief_L, objData, objMats, objAssign, 1.0); % Alpha 1.0
else
    createFallbackCube(hg_chief_L, [1 0.8 0.2]);
end

% (Right View용 Chief) - 카메라 앞에서 움직임
hg_chief_R = hgtransform('Parent', ax2);
if is_obj_loaded
    renderObjToGroup(hg_chief_R, objData, objMats, objAssign, 1.0);
else
    createFallbackCube(hg_chief_R, [1 0.8 0.2]);
end


% --- 3.2 Deputy Satellite (Only Left View) ---
hg_deputy_L = hgtransform('Parent', ax1);
createFallbackCube(hg_deputy_L, colors.deputy); % Deputy는 파란 큐브

% Deputy Body Frame 표시 (Left View)
body_axis_len = r_init_B * 0.3;
h_body_axes = hgtransform('Parent', hg_deputy_L);
line('Parent', h_body_axes, 'XData', [0 body_axis_len], 'YData', [0 0], 'ZData', [0 0], 'Color', colors.sat_body_x, 'LineWidth', 2);
line('Parent', h_body_axes, 'XData', [0 0], 'YData', [0 body_axis_len], 'ZData', [0 0], 'Color', colors.sat_body_y, 'LineWidth', 2);
line('Parent', h_body_axes, 'XData', [0 0], 'YData', [0 0], 'ZData', [0 body_axis_len], 'Color', colors.sat_body_z, 'LineWidth', 2);

% FOV Cone (Left View)
cone_len = r_init_B * 0.5;
cone_r = cone_len * tan(fov_camera/2);
[cx, cy, cz] = cylinder([0 cone_r], 10);
cz = cz * cone_len; 
% 원통을 Y축(Boresight)으로 회전 (기본 cylinder는 Z축)
% X->X, Y->Z, Z->-Y 변환 필요하지만 hgtransform으로 처리
hg_fov = hgtransform('Parent', hg_deputy_L);
p_cone = patch('Parent', hg_fov, 'XData', cx, 'YData', cz, 'ZData', cy, ...
    'FaceColor', 'c', 'FaceAlpha', 0.1, 'EdgeColor', 'c', 'EdgeAlpha', 0.3);


%% 4. 애니메이션 루프
rate = rateControl(30); % 30 FPS 제한
t_text = annotation(fig, 'textbox', [0.4 0.9 0.2 0.1], 'String', 'Initializing...', ...
    'EdgeColor', 'none', 'Color', 'w', 'FontSize', 14, 'HorizontalAlignment', 'center');

fprintf('애니메이션 루프 진입...\n');

for k = 1:skip_frame:length(t1)
    if ~ishandle(fig), break; end
    
    % 데이터 추출
    curr_t = t1(k);
    r_deputy_L = LVLH_pos(:, k); % Deputy 위치 (LVLH Frame)
    q_L2A = sensor_data(k).q_L2A; % LVLH -> Chief Body
    q_L2B = sensor_data(k).q_L2B; % LVLH -> Deputy Body
    
    R_L2A = GetDCM_QUAT(q_L2A);
    R_L2B = GetDCM_QUAT(q_L2B);
    R_B2L = R_L2B'; % Deputy Body -> LVLH
    
    % ------------------------------------------------
    % 1. Left View 업데이트 (LVLH)
    % ------------------------------------------------
    
    % Chief: 원점에서 회전만 함 (LVLH 좌표계의 중심)
    set(hg_chief_L, 'Matrix', makehgtform('scale', viz_scale_factor) * dcm2hgtform(R_L2A));
    
    % Deputy: 위치 이동 + 자세 회전
    set(hg_deputy_L, 'Matrix', makehgtform('translate', r_deputy_L) * ...
                             dcm2hgtform(R_B2L) * ...
                             makehgtform('scale', viz_scale_factor * 0.6));
    
    % FOV Cone 방향 설정 (Body Frame Y축이 Boresight라고 가정)
    % cylinder 기본이 Z축이므로, 이를 Y축으로 눕히는 변환은 patch 생성시 좌표바꿈으로 해결했음.
    
    % 궤도 꼬리 업데이트
    start_idx = max(1, k - 500); % 최근 500개만 표시 (성능)
    set(h_orbit_trail, 'XData', LVLH_pos(1, start_idx:k), ...
                       'YData', LVLH_pos(2, start_idx:k), ...
                       'ZData', LVLH_pos(3, start_idx:k));
                   
    % ------------------------------------------------
    % 2. Right View 업데이트 (Camera Frame)
    % ------------------------------------------------
    % 핵심: 카메라는 원점에 고정. Chief를 Camera(Deputy) Frame으로 가져옴.
    
    % 상대 위치 (LVLH): r_rel = r_chief - r_deputy = [0;0;0] - r_deputy = -r_deputy
    r_rel_L = -r_deputy_L; 
    
    % 상대 위치 (Deputy Body Frame): r_rel_B = R_L2B * r_rel_L
    r_rel_B = R_L2B * r_rel_L;
    
    % 상대 자세 (Deputy Body Frame): R_chief_in_B = R_L2B * (R_L2A)^T
    % Chief의 Body가 Deputy Body에서 어떻게 보이는가
    R_chief_in_B = R_L2B * R_L2A'; 
    
    % hgtransform 적용 (순서: 스케일 -> 회전 -> 이동)
    M_cam = makehgtform('translate', r_rel_B) * ...
            dcm2hgtform(R_chief_in_B) * ...
            makehgtform('scale', viz_scale_factor);
            
    set(hg_chief_R, 'Matrix', M_cam);
    
    % ------------------------------------------------
    % 타이틀 및 프레임 처리
    % ------------------------------------------------
    set(t_text, 'String', sprintf('Time: %.1f s\nDist: %.1f m', curr_t, norm(r_deputy_L)*1000));
    title(ax2, sprintf('Sensor View (Visible: %s)', checkVis(r_rel_B, fov_camera)), 'Color', 'w');
    
    drawnow limitrate;
    waitfor(rate);
end

end

%% Helper Functions

function vis_str = checkVis(pos_B, fov)
    % 간단한 가시성 체크 (Y축이 Boresight인 경우)
    dist = norm(pos_B);
    if dist < 1e-3, vis_str = 'No'; return; end
    
    % Y축과의 각도 계산
    angle = acos(pos_B(2) / dist); % pos_B(2) is Y component
    if angle < fov/2 && pos_B(2) > 0
        vis_str = 'YES';
    else
        vis_str = 'NO';
    end
end

function M = dcm2hgtform(R)
    M = eye(4);
    M(1:3, 1:3) = R;
end

function createFallbackCube(parent, color)
    % 큐브 생성 (Fallback)
    patch('Parent', parent, ...
          'Vertices', [1 1 1; -1 1 1; -1 -1 1; 1 -1 1; 1 1 -1; -1 1 -1; -1 -1 -1; 1 -1 -1], ...
          'Faces', [1 2 3 4; 5 8 7 6; 1 5 6 2; 2 6 7 3; 3 7 8 4; 4 8 5 1], ...
          'FaceColor', color, 'EdgeColor', 'none', 'FaceLighting', 'gouraud');
end

function renderObjToGroup(parentHandle, obj, materials, materialAssignments, alphaVal)
    if nargin < 5, alphaVal = 1.0; end
    
    if isempty(materialAssignments)
        patch('Parent', parentHandle, 'Vertices', obj.v, 'Faces', obj.f.v, ...
              'FaceColor', [0.7 0.7 0.7], 'EdgeColor', 'none', ...
              'FaceLighting', 'gouraud', 'FaceAlpha', alphaVal);
    else
        for i = 1:length(materialAssignments)
            matGroup = materialAssignments(i);
            if isempty(matGroup.faceIndices), continue; end
            faces = obj.f.v(matGroup.faceIndices, :);
            
            color = [0.7 0.7 0.7];
            if isfield(materials, matGroup.material)
                mat = materials.(matGroup.material);
                if isfield(mat, 'Kd'), color = mat.Kd; end
            end
            
            p = patch('Parent', parentHandle, 'Vertices', obj.v, 'Faces', faces, ...
                  'FaceColor', color, 'EdgeColor', 'none', ...
                  'FaceLighting', 'gouraud', 'FaceAlpha', alphaVal);
            
            % 재질감 추가
            material(p, 'dull'); 
        end
    end
end