function visualizeObjWithColor(objFilePath)
    % 메인 시각화 함수 - 캐싱 버전
    [obj, materials, materialAssignments] = readObjWithCache(objFilePath);
    
    % 시각화
    figure('Name', 'OBJ Viewer', 'Color', 'w');
    hold on;
    
    % Material이 없으면 단일 patch로 빠르게 그리기
    if isempty(materialAssignments)
        patch('Vertices', obj.v, ...
              'Faces', obj.f.v, ...
              'FaceColor', [0.7 0.7 0.7], ...
              'EdgeColor', 'none', ...
              'FaceLighting', 'gouraud');
    else
        % Material별로 그리기 (최적화)
        for i = 1:length(materialAssignments)
            matGroup = materialAssignments(i);
            if isempty(matGroup.faceIndices)
                continue;
            end
            
            faces = obj.f.v(matGroup.faceIndices, :);
            
            % 색상 결정
            color = [0.7 0.7 0.7];  % 기본값
            if isfield(materials, matGroup.material)
                mat = materials.(matGroup.material);
                if isfield(mat, 'Kd')
                    color = mat.Kd;
                end
            end
            
            % 한 번에 그리기
            patch('Vertices', obj.v, ...
                  'Faces', faces, ...
                  'FaceColor', color, ...
                  'EdgeColor', 'none', ...
                  'FaceLighting', 'gouraud');
        end
    end
    
    % 빠른 설정
    axis equal;
    grid on;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    camlight('headlight');
    lighting gouraud;
    view(45, 30);
    rotate3d on;
    
    [~, name, ~] = fileparts(objFilePath);
    title(sprintf('Model: %s', name));
end