function materials = readMtlFast(filename)
    % 빠른 MTL 파서
    materials = struct();
    
    % 파일 전체 읽기
    fileContent = fileread(filename);
    lines = strsplit(fileContent, '\n');
    
    currentMat = '';
    
    for i = 1:length(lines)
        line = strtrim(lines{i});
        if isempty(line) || line(1) == '#'
            continue;
        end
        
        % 빠른 명령어 체크
        if startsWith(line, 'newmtl ')
            currentMat = strrep(line(8:end), ' ', '_');
            currentMat = matlab.lang.makeValidName(currentMat);
            materials.(currentMat) = struct();
            
        elseif ~isempty(currentMat)
            if startsWith(line, 'Kd ')
                % Diffuse color
                materials.(currentMat).Kd = sscanf(line(4:end), '%f %f %f')';
                
            elseif startsWith(line, 'Ka ')
                % Ambient color
                materials.(currentMat).Ka = sscanf(line(4:end), '%f %f %f')';
                
            elseif startsWith(line, 'Ks ')
                % Specular color
                materials.(currentMat).Ks = sscanf(line(4:end), '%f %f %f')';
                
            elseif startsWith(line, 'Ns ')
                % Shininess
                materials.(currentMat).Ns = sscanf(line(4:end), '%f');
                
            elseif startsWith(line, 'd ')
                % Transparency
                materials.(currentMat).d = sscanf(line(3:end), '%f');
                
            elseif startsWith(line, 'Tr ')
                % Transparency (reversed)
                materials.(currentMat).d = 1 - sscanf(line(4:end), '%f');
            end
        end
    end
end