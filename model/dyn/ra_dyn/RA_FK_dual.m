function FK_dual = RA_FK_dual(params, p_base, q_base, theta_L, theta_R)
% RA_FK_dual: 양팔 Forward Kinematics
%
% 기존 RA_FK를 각 팔에 대해 호출하여 결과 합성
%
% 입력:
%   params  - 시스템 파라미터 (params_init 출력)
%   p_base  - 위성 CoM 위치 (3x1) [m], 관성좌표계
%   q_base  - 위성 자세 쿼터니언 (4x1) [qw; qx; qy; qz]
%   theta_L - 왼팔 관절각 (7x1) [rad]
%   theta_R - 오른팔 관절각 (7x1) [rad]
%
% 출력:
%   FK_dual.L       - 왼팔 FK 결과 (RA_FK 출력 구조체)
%   FK_dual.R       - 오른팔 FK 결과 (RA_FK 출력 구조체)
%   FK_dual.R_base  - 위성 회전행렬 (3x3)
%   FK_dual.p_base  - 위성 CoM 위치 (3x1)
%
% 장착점 정의 (params_init.m):
%   mount(1): +Y 팔 (Left)  - pos = [1.9; 0.9; 0]
%   mount(2): -Y 팔 (Right) - pos = [1.9; -0.9; 0]

%% 입력 검증
p_base = p_base(:);
q_base = q_base(:);
theta_L = theta_L(:);
theta_R = theta_R(:);

%% 위성 회전행렬 (공통)
R_base = GetDCM_QUAT(q_base);

%% 왼팔 FK (mount 1: +Y)
params_L = params;
params_L.sat.r_mount = params.sat.mount(1).pos;
params_L.sat.R_mount = params.sat.mount(1).R;
FK_L = RA_FK(params_L, p_base, q_base, theta_L);

%% 오른팔 FK (mount 2: -Y)
params_R = params;
params_R.sat.r_mount = params.sat.mount(2).pos;
params_R.sat.R_mount = params.sat.mount(2).R;
FK_R = RA_FK(params_R, p_base, q_base, theta_R);

%% 출력 구조체 조립
FK_dual.L = FK_L;
FK_dual.R = FK_R;
FK_dual.R_base = R_base;
FK_dual.p_base = p_base;

end