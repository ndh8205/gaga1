function Jg = RA_Jacobian_Gen(FK, params)
% RA_Jacobian_Gen: Free-floating Generalized Jacobian 계산
%
% 논문 참조: Zong et al., "Reactionless Control of Free-floating Space Manipulators"
%           IEEE TAES, 2019
%
% =========================================================================
% 개념:
%   일반 Jacobian:
%     ẋ_ee = Jb * ζ_b + Jm * θ̇
%
%   Free-floating 운동량 보존 (외력 없음):
%     ζ_b = -H_bb⁻¹ * H_bm * θ̇
%
%   대입하면:
%     ẋ_ee = (Jm - Jb * H_bb⁻¹ * H_bm) * θ̇
%          = J* * θ̇
%
%   여기서 J* = Generalized Jacobian
%
% =========================================================================
% 입력 (Inputs):
%   FK     - RA_FK 함수의 출력 구조체
%   params - 시스템 파라미터 구조체
%
% 출력 (Outputs):
%   Jg - Generalized Jacobian (6 x n_joints)
%        관절 속도 θ̇가 End-effector 속도 [v_ee; ω_ee]에 미치는 영향
%        (Base 반작용 포함)
%
% =========================================================================

%% ========== Mass Matrix 계산 ==========
H = RA_MM(FK, params);

% Partition: base (1:6), arm (7:13)
H_bb = H(1:6, 1:6);     % Base 관성 (6x6)
H_bm = H(1:6, 7:end);   % Base-Arm coupling (6xn)

%% ========== Jacobian 계산 ==========
[Jb, Jm] = RA_Jacobian(FK, params);

%% ========== Generalized Jacobian ==========
% J* = Jm - Jb * H_bb⁻¹ * H_bm

H_bb_inv = H_bb \ eye(6);
Jg = Jm - Jb * H_bb_inv * H_bm;

end