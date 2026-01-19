% function [R, t] = mlpnp_solve_improved(A, is_planar, points3D, bearings, nullspaces, eigenRot)
% % C++ 코드와 정확히 일치 - eigenRot 사용
% 
% if nargin < 6
%     eigenRot = eye(3);
% end
% 
% % SVD로 null space 해 구하기
% AtA = A' * A;
% [~, S, V] = svd(AtA);
% solution = V(:, end);
% 
% if is_planar
%     [R, t] = recover_pose_planar(solution, points3D, bearings, nullspaces, eigenRot);
% else
%     [R, t] = recover_pose_nonplanar(solution, points3D, bearings, nullspaces);
% end
% 
% end

function [R, t] = mlpnp_solve_improved(A, is_planar, points3D, bearings, nullspaces, eigenRot, Kll)
% MLPnP 초기 해 계산 (가중 최소제곱법 선택적 지원)
%
% Inputs:
%   A          - 설계 행렬
%   is_planar  - planar 여부
%   points3D   - 3xN 3D 포인트
%   bearings   - 3xN bearing vectors
%   nullspaces - Nx1 cell array
%   eigenRot   - planar 경우 회전 행렬
%   Kll        - 2N×2N 가중 행렬 (optional)
%
% Outputs:
%   R, t - 회전 행렬과 이동 벡터

if nargin < 6
    eigenRot = eye(3);
end

% 가중치 처리
if nargin < 7 || isempty(Kll)
    % 가중치 없음: 기존 방식
    AtA = A' * A;
else
    % 가중 최소제곱법: A' * Kll * A
    AtA = A' * Kll * A;
end

% SVD로 null space 해 구하기
[~, S, V] = svd(AtA);
solution = V(:, end);

if is_planar
    [R, t] = recover_pose_planar(solution, points3D, bearings, nullspaces, eigenRot);
else
    [R, t] = recover_pose_nonplanar(solution, points3D, bearings, nullspaces);
end

end