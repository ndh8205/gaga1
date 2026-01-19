function [Jb_L, Jm_L, Jb_R, Jm_R] = RA_Jacobian_dual(FK_dual, params)
% RA_Jacobian_dual: 양팔 시스템 Jacobian 계산
%
% 논문 참조: Zong et al., "Reactionless Control of Free-floating Space Manipulators"
%           IEEE TAES, 2019 - Eq. 8-10
%
% =========================================================================
% 기구학적 관계:
%   [v_ee_L]   [Jb_L] [v_b  ]   [Jm_L    0  ] [theta_dot_L]
%   [w_ee_L] = [    ] [     ] + [           ] [           ]
%   [v_ee_R]   [Jb_R] [w_b  ]   [  0   Jm_R ] [theta_dot_R]
%   [w_ee_R]
%
% 또는 통합 형태:
%   x_dot_ee = J_dual * zeta
%
%   여기서:
%     x_dot_ee = [v_ee_L; w_ee_L; v_ee_R; w_ee_R] (12x1)
%     zeta = [v_b; w_b; theta_dot_L; theta_dot_R] (20x1)
%
% =========================================================================
% 입력 (Inputs):
%   FK_dual - RA_FK_dual 함수의 출력 구조체
%   params  - 시스템 파라미터 구조체
%
% 출력 (Outputs):
%   Jb_L - 왼팔 Base Jacobian (6x6)
%   Jm_L - 왼팔 Manipulator Jacobian (6x7)
%   Jb_R - 오른팔 Base Jacobian (6x6)
%   Jm_R - 오른팔 Manipulator Jacobian (6x7)
%
% =========================================================================

%% ========== 기존 RA_Jacobian 호출 ==========

% 왼팔 Jacobian
[Jb_L, Jm_L] = RA_Jacobian(FK_dual.L, params);

% 오른팔 Jacobian
[Jb_R, Jm_R] = RA_Jacobian(FK_dual.R, params);

end


function J_dual = RA_Jacobian_dual_full(FK_dual, params)
% RA_Jacobian_dual_full: 양팔 통합 Jacobian (12x20)
%
% x_dot_ee = J_dual * zeta
%
% 구조:
%   J_dual = [Jb_L  Jm_L    0  ]  6 (왼팔 EE)
%            [Jb_R    0   Jm_R ]  6 (오른팔 EE)
%              6     7      7
%
% 출력:
%   J_dual - 통합 Jacobian (12x20)

n_joints = params.arm.n_joints;  % 7

% 개별 Jacobian
[Jb_L, Jm_L, Jb_R, Jm_R] = RA_Jacobian_dual(FK_dual, params);

% 통합 Jacobian 조립
J_dual = zeros(12, 6 + 2*n_joints);

% 왼팔 EE (1:6 행)
J_dual(1:6, 1:6) = Jb_L;
J_dual(1:6, 7:13) = Jm_L;
% J_dual(1:6, 14:20) = 0 (오른팔 관절은 왼팔 EE에 영향 없음)

% 오른팔 EE (7:12 행)
J_dual(7:12, 1:6) = Jb_R;
% J_dual(7:12, 7:13) = 0 (왼팔 관절은 오른팔 EE에 영향 없음)
J_dual(7:12, 14:20) = Jm_R;

end