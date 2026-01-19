clear all
close all
clc

% ===== 식 (3.18): μc × Ac × Nc × cos(αsun) × BCR × Sirr × MPPT =====

% Table 3.3 입력값 (page 33)
mu_c = 0.302;           % Cell Efficiency
A_c = 30.18e-4;        % Cell Area (m²)
% A_c = 0.003;        % Cell Area (m²)
N_c = 36;              % Cell Number (전개 셀 갯수)
S_irr = 1326;          % Solar Irradiation (W/m²)
alpha_sun = deg2rad(0);         % 태양 각도 (deg)
% other_loss = 0.90;     % 기타 손실
other_loss = 1;     % 기타 손실


% Table 3.4 MPPT 효율
function eff = calc_efficiency(omega)
    if omega >= 10
        eff = 0.60;
    elseif omega >= 2
        eff = -0.025 * abs(omega) + 0.85;
    else
        eff = 0.80;
    end
end

omega = 0;
efficiency = calc_efficiency(omega);

% 순간 최대 전력 (모든 셀이 최적 각도일 때) - 각도 0으로 둠
P_gen = mu_c * A_c * N_c * cosd(alpha_sun) * 1 * S_irr * other_loss;

fprintf('순간 최대 전력: %.2f W\n', P_gen);
fprintf('MPPT 효율: %.1f %%\n', efficiency*100);