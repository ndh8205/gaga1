function plotObjModel(obj, materials, materialAssignments)
    hold on;
    
    % Material이 없는 경우 - 빠른 단일 렌더링
    if isempty(materialAssignments)
        patch('Vertices', obj.v, ...
              'Faces', obj.f.v, ...
              'FaceColor', [0.7 0.7 0.7], ...
              'EdgeColor', 'none', ...
              'FaceLighting', 'gouraud');
    else
        % Material별 그리기 - 색상 미리 계산
        colors = cell(length(materialAssignments), 1);
        faceGroups = cell(length(materialAssignments), 1);
        
        for i = 1:length(materialAssignments)
            matGroup = materialAssignments(i);
            if isempty(matGroup.faceIndices)
                continue;
            end
            
            faceGroups{i} = obj.f.v(matGroup.faceIndices, :);
            
            % 색상 결정 (한 번만)
            if isfield(materials, matGroup.material) && isfield(materials.(matGroup.material), 'Kd')
                colors{i} = materials.(matGroup.material).Kd;
            else
                % 기본 색상 매핑
                matName = lower(matGroup.material);
                if contains(matName, 'solar') || contains(matName, 'panel')
                    colors{i} = [0.1 0.1 0.8];
                elseif contains(matName, 'antenna')
                    colors{i} = [0.9 0.9 0.1];
                else
                    colors{i} = [0.7 0.7 0.7];
                end
            end
        end
        
        % 한 번에 그리기
        for i = 1:length(faceGroups)
            if ~isempty(faceGroups{i})
                patch('Vertices', obj.v, ...
                      'Faces', faceGroups{i}, ...
                      'FaceColor', colors{i}, ...
                      'EdgeColor', 'none', ...
                      'FaceLighting', 'gouraud');
            end
        end
    end
    
    axis equal;
    grid on;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    camlight('headlight');
    lighting gouraud;
    material shiny;
    rotate3d on;
end