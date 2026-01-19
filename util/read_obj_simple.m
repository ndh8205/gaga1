function [vertices, faces] = read_obj_simple(filename)
% 간단한 OBJ 파일 리더
% 
% Inputs:
%   filename - OBJ 파일 경로
%
% Outputs:
%   vertices - Nx3 버텍스 좌표
%   faces    - Mx3 face 인덱스

fid = fopen(filename, 'r');
if fid == -1
    error('파일을 열 수 없습니다: %s', filename);
end

vertices = [];
faces = [];

while ~feof(fid)
    line = fgetl(fid);
    if isempty(line) || line(1) == '#'
        continue;
    end
    
    tokens = strsplit(line);
    
    if strcmp(tokens{1}, 'v')
        % Vertex: v x y z
        v = [str2double(tokens{2}), str2double(tokens{3}), str2double(tokens{4})];
        vertices = [vertices; v];
        
    elseif strcmp(tokens{1}, 'f')
        % Face: f v1 v2 v3 또는 f v1/vt1/vn1 v2/vt2/vn2 v3/vt3/vn3
        face_idx = zeros(1, 3);
        for i = 1:3
            % '/' 구분자 처리
            parts = strsplit(tokens{i+1}, '/');
            face_idx(i) = str2double(parts{1});
        end
        faces = [faces; face_idx];
    end
end

fclose(fid);

fprintf('OBJ 로드 완료: %d vertices, %d faces\n', size(vertices, 1), size(faces, 1));

end