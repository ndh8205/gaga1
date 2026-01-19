%% check_all_stl_size.m
% 위성 + 로봇팔 모든 STL 바운딩 박스 확인
clear; clc;

%% 경로
satPath = 'D:\pj2025\space_challenge\model\modeling_3d\';
armPath = 'D:\pj2025\space_challenge\model\modeling_3d\ASM_KARI_ARM\meshes\';

%% STL 파일 목록
stl_files = {
    % 위성
    [satPath 'satellite_body.stl'], '위성 본체';
    [satPath 'satellite_panel.stl'], '태양전지판';
    [satPath 'satellite_front.stl'], '전방부';
    % 로봇팔
    [armPath 'ASM_J0.STL'], 'Arm Base';
    [armPath 'ASM_J1.STL'], 'Link1';
    [armPath 'ASM_J2.STL'], 'Link2';
    [armPath 'ASM_J3.STL'], 'Link3';
    [armPath 'ASM_J4.STL'], 'Link4';
    [armPath 'ASM_J5.STL'], 'Link5';
    [armPath 'ASM_J6.STL'], 'Link6';
    [armPath 'ASM_J7.STL'], 'Link7';
};

%% 분석
fprintf('============================================================\n');
fprintf('  STL 바운딩 박스 분석\n');
fprintf('============================================================\n');
fprintf('%-15s | %8s %8s %8s | %8s %8s %8s\n', ...
    '파일', 'X_min', 'X_max', 'X_size', 'Y_min', 'Y_max', 'Y_size');
fprintf('%-15s | %8s %8s %8s | %8s %8s %8s\n', ...
    '', 'Z_min', 'Z_max', 'Z_size', '', '', '');
fprintf('------------------------------------------------------------\n');

for i = 1:size(stl_files, 1)
    fpath = stl_files{i, 1};
    fname = stl_files{i, 2};
    
    if ~isfile(fpath)
        fprintf('%-15s | 파일 없음\n', fname);
        continue;
    end
    
    tr = stlread(fpath);
    v = tr.Points;
    
    x_min = min(v(:,1)); x_max = max(v(:,1)); x_size = x_max - x_min;
    y_min = min(v(:,2)); y_max = max(v(:,2)); y_size = y_max - y_min;
    z_min = min(v(:,3)); z_max = max(v(:,3)); z_size = z_max - z_min;
    
    fprintf('%-15s | %+8.3f %+8.3f %8.3f | %+8.3f %+8.3f %8.3f\n', ...
        fname, x_min, x_max, x_size, y_min, y_max, y_size);
    fprintf('%-15s | %+8.3f %+8.3f %8.3f |\n', ...
        '', z_min, z_max, z_size);
    fprintf('------------------------------------------------------------\n');
end

fprintf('\n로봇팔 장착점: [1.9, ±0.9, 0]\n');
fprintf('위성 본체 중심: [0, 0, 0]\n');