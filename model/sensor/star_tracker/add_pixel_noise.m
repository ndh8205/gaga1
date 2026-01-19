%% add_pixel_noise.m
function pixel_coords_noisy = add_pixel_noise(pixel_coords, sigma_pixel)
% 픽셀 좌표에 가우시안 노이즈 추가
%
% Inputs:
%   pixel_coords - Nx2
%   sigma_pixel  - 픽셀 단위 표준편차 (0.1~0.5 현실적)

N = size(pixel_coords, 1);
noise = sigma_pixel * randn(N, 2);
pixel_coords_noisy = pixel_coords + noise;

end