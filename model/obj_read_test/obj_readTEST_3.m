%% OBJ 파일 시각화 테스트 (캐싱 최적화 버전)
clear; close all; clc;
addpath(genpath('C:\Users\USER\Desktop\relative2'));

%% 2. 6U CubeSat 시각화
fprintf('Loading 6U CubeSat...\n');
tic;
visualizeObjWithColor('model/obj_read_test/6U_CubeSat_v7_nopayload/6U_CubeSat_v7_nopayload.obj');
fprintf('Loading time: %.2f seconds\n', toc);

%% 3. 3U CNU CubeSat 시각화
fprintf('\nLoading 3U CNU CubeSat...\n');
tic;
visualizeObjWithColor('model/obj_read_test/1015_3U_CNU/1015_3U_CNU.obj');
fprintf('Loading time: %.2f seconds\n', toc);

%% 4. 두 모델 비교 (검은 배경)
figure('Name', 'CubeSat Comparison', 'Position', [100 100 1200 500], ...
       'Color', 'k');  % Figure 배경 검은색

% 6U CubeSat
subplot(1,2,1);
tic;
[obj6U, mat6U, matAssign6U] = readObjWithCache('model/obj_read_test/6U_CubeSat_v7_nopayload/6U_CubeSat_v7_nopayload.obj');
fprintf('6U loaded in %.2f sec\n', toc);
plotObjModel(obj6U, mat6U, matAssign6U);
title('6U CubeSat', 'Color', 'w');  % 제목 글자 흰색
view(45, 30);

% Axes 배경 검은색, 그리드와 라벨 흰색
ax1 = gca;
ax1.Color = 'k';
ax1.XColor = 'w';
ax1.YColor = 'w';
ax1.ZColor = 'w';
ax1.GridColor = [0.3 0.3 0.3];  % 그리드 어두운 회색
ax1.GridAlpha = 0.3;

% 3U CubeSat
subplot(1,2,2);
tic;
[obj3U, mat3U, matAssign3U] = readObjWithCache('model/obj_read_test/1015_3U_CNU/1015_3U_CNU.obj');
fprintf('3U loaded in %.2f sec\n', toc);
plotObjModel(obj3U, mat3U, matAssign3U);
title('3U CNU CubeSat', 'Color', 'w');  % 제목 글자 흰색
view(45, 30);

% Axes 배경 검은색, 그리드와 라벨 흰색
ax2 = gca;
ax2.Color = 'k';
ax2.XColor = 'w';
ax2.YColor = 'w';
ax2.ZColor = 'w';
ax2.GridColor = [0.3 0.3 0.3];  % 그리드 어두운 회색
ax2.GridAlpha = 0.3;

%% 5. 크기 비교
fprintf('\n=== Model Size Comparison ===\n');
fprintf('6U CubeSat:\n');
printModelDimensions(obj6U.v);
fprintf('\n3U CubeSat:\n');
printModelDimensions(obj3U.v);