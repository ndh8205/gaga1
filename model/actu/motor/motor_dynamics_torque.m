function [tau_dot, tau_sat] = motor_dynamics_torque(tau_actual, tau_cmd, params)
% motor_dynamics_torque: 토크 명령에 대한 1차 지연 + Saturation 모델
%
% Inputs:
%   tau_actual - Nx1 현재 실제 토크 [Nm]
%   tau_cmd    - Nx1 명령 토크 [Nm]
%   params     - 파라미터 구조체
%
% Outputs:
%   tau_dot - Nx1 토크 변화율 [Nm/s]
%   tau_sat - Nx1 saturation 적용된 명령 토크 [Nm]
%
% Model:
%   tau_m * d(tau_actual)/dt + tau_actual = tau_sat
%   where tau_m = tau_up (가속) or tau_down (감속)
%
% 모터 관성 효과:
%   반영 관성 I_refl = I_motor * N^2
%   → 시정수에 반영: tau_up(i) ∝ sqrt(I_refl(i))

%% 파라미터 추출
tau_up   = params.motor.tau_up;      % 가속 시정수 [s] (스칼라 또는 Nx1)
tau_down = params.motor.tau_down;    % 감속 시정수 [s] (스칼라 또는 Nx1)
tau_max  = params.motor.tau_max;     % 최대 토크 [Nm] (스칼라 또는 Nx1)

%% 모터 수
n_motor = length(tau_actual);

%% 파라미터 배열 확장 (스칼라 → 벡터)
if length(tau_max) == 1
    tau_max = tau_max * ones(n_motor, 1);
end
if length(tau_up) == 1
    tau_up = tau_up * ones(n_motor, 1);
end
if length(tau_down) == 1
    tau_down = tau_down * ones(n_motor, 1);
end

%% 명령 토크 Saturation (조인트별)
tau_sat = max(min(tau_cmd(:), tau_max(:)), -tau_max(:));

%% 1차 지연 동역학 (비대칭 시정수, 조인트별)
tau_dot = zeros(n_motor, 1);

for i = 1:n_motor
    % 토크 증가/감소 방향 판단
    if abs(tau_sat(i)) >= abs(tau_actual(i))
        tau_m = tau_up(i);     % 토크 증가
    else
        tau_m = tau_down(i);   % 토크 감소
    end
    
    tau_dot(i) = (tau_sat(i) - tau_actual(i)) / tau_m;
end

end