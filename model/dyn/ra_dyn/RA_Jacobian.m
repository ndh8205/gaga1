function [Jb, Jm] = RA_Jacobian(FK, params)
% RA_Jacobian: Base Jacobian 및 Manipulator Jacobian 계산
%
% 논문 참조: Zong et al., "Reactionless Control of Free-floating Space Manipulators"
%           IEEE TAES, 2019 - Eq. 8-10
%
% =========================================================================
% 기구학적 관계 (Eq. 8):
%   [v_e  ]   [J_b'] [v_0    ]   [J_m']
%   [     ] = [    ] [       ] + [    ] * theta_dot
%   [w_e  ]   [    ] [w_base ]   [    ]
%
% 여기서:
%   v_e, w_e     : End-effector 선속도/각속도 (관성좌표계)
%   v_0, w_base  : Base 기준점 선속도/각속도 (관성좌표계)
%   theta_dot    : 관절 각속도
%
% =========================================================================
% 입력 (Inputs):
%   FK     - RA_FK 함수의 출력 구조체
%            FK.p_joint{i} : Joint i 위치 (3x1)
%            FK.p_ee       : End-effector 위치 (3x1)
%            FK.k{i}       : Joint i 회전축 (3x1, 관성좌표계)
%            FK.p_mount    : 로봇팔 장착점 위치 (3x1)
%   params - 시스템 파라미터 구조체
%
% 출력 (Outputs):
%   Jb - Base Jacobian (6x6)
%        Base 속도 [v_0; w_base]가 End-effector 속도에 미치는 영향
%   Jm - Manipulator Jacobian (6xn)
%        관절 속도 theta_dot이 End-effector 속도에 미치는 영향
%
% =========================================================================
% Jacobian 구조:
%
%   J_b' = [  I_3   -[p_0e]_x ]    (Eq. 9)
%          [  0_3      I_3    ]
%
%   J_m' = [ k_1 x (p_e - p_1)  ...  k_n x (p_e - p_n) ]   (Eq. 10)
%          [       k_1         ...         k_n        ]
%
%   여기서:
%     p_0e = p_ee - p_mount (기준점에서 EEF까지 벡터)
%     k_i  = Joint i 회전축 단위벡터 (관성좌표계)
%     p_i  = Joint i 위치 (관성좌표계)
%     p_e  = End-effector 위치 (관성좌표계)
%
% =========================================================================

%% ========== 파라미터 추출 ==========

n_joints = params.arm.n_joints;  % 관절 수 (7)

% FK 결과 추출
p_ee = FK.p_ee(:);       % End-effector 위치
p_mount = FK.p_mount(:); % 장착점 (Base 기준점)

%% ========== Base Jacobian (J_b') 계산 - Eq. 9 ==========
%
% J_b' = [  I_3   -[p_0e]_x ]
%        [  0_3      I_3    ]
%
% 의미: 
%   - 좌상단 I_3: Base 선속도 → EEF 선속도 (직접 전달)
%   - 우상단 -[p_0e]_x: Base 각속도 → EEF 선속도 (회전에 의한 선속도)
%   - 좌하단 0_3: Base 선속도 → EEF 각속도 (영향 없음)
%   - 우하단 I_3: Base 각속도 → EEF 각속도 (직접 전달)

% 기준점(장착점)에서 End-effector까지의 벡터
p_0e = p_ee - p_mount;

% Base Jacobian 조립
Jb = zeros(6, 6);
Jb(1:3, 1:3) = eye(3);              % I_3
Jb(1:3, 4:6) = -skew3(p_0e);        % -[p_0e]_x
Jb(4:6, 4:6) = eye(3);              % I_3

%% ========== Manipulator Jacobian (J_m') 계산 - Eq. 10 ==========
%
% J_m' = [ k_1 x (p_e - p_1)  ...  k_n x (p_e - p_n) ]  (선속도 기여)
%        [       k_1         ...         k_n        ]  (각속도 기여)
%
% 각 열 j의 의미:
%   - 상단 (1:3): Joint j 회전이 EEF 선속도에 미치는 영향
%     → k_j x (p_e - p_j) = 회전축 x (EEF까지 거리) = 선속도 방향
%   - 하단 (4:6): Joint j 회전이 EEF 각속도에 미치는 영향
%     → k_j = 회전축 방향으로 각속도 전달

Jm = zeros(6, n_joints);

for j = 1:n_joints
    % Joint j의 회전축 (관성좌표계)
    k_j = FK.k{j}(:);

    % Joint j 위치 (관성좌표계)
    p_j = FK.p_joint{j}(:);

    % Joint j에서 End-effector까지의 벡터
    r_je = p_ee - p_j;

    % 선속도 Jacobian (상단 3행)
    % v_e = k_j x r_je * theta_dot_j
    Jm(1:3, j) = cross(k_j, r_je);

    % 각속도 Jacobian (하단 3행)
    % w_e = k_j * theta_dot_j
    Jm(4:6, j) = k_j;
end

end
