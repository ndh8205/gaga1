function clearAllCache()
    % 모든 캐시 삭제
    cacheDirs = dir('**/.cache');
    for i = 1:length(cacheDirs)
        if cacheDirs(i).isdir
            rmdir(fullfile(cacheDirs(i).folder, cacheDirs(i).name), 's');
            fprintf('Cleared cache in %s\n', cacheDirs(i).folder);
        end
    end
end