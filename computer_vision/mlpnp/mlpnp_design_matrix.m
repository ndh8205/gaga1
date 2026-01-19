% function [A, is_planar, eigenRot, points_transformed] = mlpnp_design_matrix(points3D, nullspaces)
% % C++와 정확히 일치 - eigenRot과 points_transformed 반환
% 
% N = size(points3D, 2);
% 
% % Planar test (centering 없음!)
% planarTest = points3D * points3D';
% 
% % Rank test with threshold
% [V, D] = eig(planarTest);
% eigenvalues = diag(D);
% 
% % C++ threshold: 1e-10
% if min(eigenvalues) < 1e-10 || rank(planarTest, 1e-10) == 2
%     is_planar = true;
% 
%     % SelfAdjointEigenSolver (오름차순)
%     [V_sorted, D_sorted] = eig(planarTest);
%     [~, idx] = sort(diag(D_sorted));
%     eigenRot = V_sorted(:, idx)';  % transposeInPlace
% 
%     % Transform points
%     points_transformed = eigenRot * points3D;
% else
%     is_planar = false;
%     eigenRot = eye(3);
%     points_transformed = points3D;
% end
% 
% % Build A matrix
% if is_planar
%     A = zeros(2*N, 9);
%     for i = 1:N
%         pt = points_transformed(:,i);
%         ns = nullspaces{i};
%         row1 = 2*i - 1;
%         row2 = 2*i;
% 
%         % C++ 순서 정확히 따르기
%         A(row1, 1) = ns(1,1) * pt(2);  % r12
%         A(row2, 1) = ns(1,2) * pt(2);
%         A(row1, 2) = ns(1,1) * pt(3);  % r13
%         A(row2, 2) = ns(1,2) * pt(3);
%         A(row1, 3) = ns(2,1) * pt(2);  % r22
%         A(row2, 3) = ns(2,2) * pt(2);
%         A(row1, 4) = ns(2,1) * pt(3);  % r23
%         A(row2, 4) = ns(2,2) * pt(3);
%         A(row1, 5) = ns(3,1) * pt(2);  % r32
%         A(row2, 5) = ns(3,2) * pt(2);
%         A(row1, 6) = ns(3,1) * pt(3);  % r33
%         A(row2, 6) = ns(3,2) * pt(3);
%         A(row1, 7) = ns(1,1);          % t1
%         A(row2, 7) = ns(1,2);
%         A(row1, 8) = ns(2,1);          % t2
%         A(row2, 8) = ns(2,2);
%         A(row1, 9) = ns(3,1);          % t3
%         A(row2, 9) = ns(3,2);
%     end
% else
%     A = zeros(2*N, 12);
%     for i = 1:N
%         pt = points3D(:,i);  % 변환 안된 원본 사용
%         ns = nullspaces{i};
%         row1 = 2*i - 1;
%         row2 = 2*i;
% 
%         % C++ 순서대로
%         A(row1, 1) = ns(1,1) * pt(1);  % r11
%         A(row2, 1) = ns(1,2) * pt(1);
%         A(row1, 2) = ns(1,1) * pt(2);  % r12
%         A(row2, 2) = ns(1,2) * pt(2);
%         A(row1, 3) = ns(1,1) * pt(3);  % r13
%         A(row2, 3) = ns(1,2) * pt(3);
%         A(row1, 4) = ns(2,1) * pt(1);  % r21
%         A(row2, 4) = ns(2,2) * pt(1);
%         A(row1, 5) = ns(2,1) * pt(2);  % r22
%         A(row2, 5) = ns(2,2) * pt(2);
%         A(row1, 6) = ns(2,1) * pt(3);  % r23
%         A(row2, 6) = ns(2,2) * pt(3);
%         A(row1, 7) = ns(3,1) * pt(1);  % r31
%         A(row2, 7) = ns(3,2) * pt(1);
%         A(row1, 8) = ns(3,1) * pt(2);  % r32
%         A(row2, 8) = ns(3,2) * pt(2);
%         A(row1, 9) = ns(3,1) * pt(3);  % r33
%         A(row2, 9) = ns(3,2) * pt(3);
%         A(row1, 10) = ns(1,1);         % t1
%         A(row2, 10) = ns(1,2);
%         A(row1, 11) = ns(2,1);         % t2
%         A(row2, 11) = ns(2,2);
%         A(row1, 12) = ns(3,1);         % t3
%         A(row2, 12) = ns(3,2);
%     end
% end
% end

function [A, is_planar, eigenRot, points_transformed, Kll] = mlpnp_design_matrix(points3D, nullspaces, cov_reduced)
% MLPnP 설계 행렬 구성 (공분산 가중치 선택적 지원)
%
% Inputs:
%   points3D      - 3xN 3D 포인트
%   nullspaces    - Nx1 cell array of null space matrices
%   cov_reduced   - 2x2xN reduced 공분산 (optional)
%
% Outputs:
%   A                  - 설계 행렬 (2N×9 또는 2N×12)
%   is_planar          - planar 여부
%   eigenRot           - planar 경우 회전 행렬
%   points_transformed - 변환된 포인트
%   Kll                - 2N×2N 가중 행렬 (optional)

N = size(points3D, 2);

% 공분산 처리 여부
use_cov = nargin >= 3 && ~isempty(cov_reduced);

% 가중 행렬 초기화
if nargout > 4 && use_cov
    Kll = zeros(2*N, 2*N);
else
    Kll = eye(2*N);  % 단위 행렬 (가중치 없음)
end

% Planar test (centering 없음!)
planarTest = points3D * points3D';

% Rank test with threshold
[V, D] = eig(planarTest);
eigenvalues = diag(D);

% C++ threshold: 1e-10
if min(eigenvalues) < 1e-10 || rank(planarTest, 1e-10) == 2
    is_planar = true;
    
    % SelfAdjointEigenSolver (오름차순)
    [V_sorted, D_sorted] = eig(planarTest);
    [~, idx] = sort(diag(D_sorted));
    eigenRot = V_sorted(:, idx)';  % transposeInPlace
    
    % Transform points
    points_transformed = eigenRot * points3D;
else
    is_planar = false;
    eigenRot = eye(3);
    points_transformed = points3D;
end

% Build A matrix
if is_planar
    A = zeros(2*N, 9);
    for i = 1:N
        pt = points_transformed(:,i);
        ns = nullspaces{i};
        row1 = 2*i - 1;
        row2 = 2*i;
        
        % 가중 행렬 설정 (대각 블록)
        if use_cov
            Kll(row1:row2, row1:row2) = cov_reduced(:,:,i);
        end
        
        % C++ 순서 정확히 따르기
        A(row1, 1) = ns(1,1) * pt(2);  % r12
        A(row2, 1) = ns(1,2) * pt(2);
        A(row1, 2) = ns(1,1) * pt(3);  % r13
        A(row2, 2) = ns(1,2) * pt(3);
        A(row1, 3) = ns(2,1) * pt(2);  % r22
        A(row2, 3) = ns(2,2) * pt(2);
        A(row1, 4) = ns(2,1) * pt(3);  % r23
        A(row2, 4) = ns(2,2) * pt(3);
        A(row1, 5) = ns(3,1) * pt(2);  % r32
        A(row2, 5) = ns(3,2) * pt(2);
        A(row1, 6) = ns(3,1) * pt(3);  % r33
        A(row2, 6) = ns(3,2) * pt(3);
        A(row1, 7) = ns(1,1);          % t1
        A(row2, 7) = ns(1,2);
        A(row1, 8) = ns(2,1);          % t2
        A(row2, 8) = ns(2,2);
        A(row1, 9) = ns(3,1);          % t3
        A(row2, 9) = ns(3,2);
    end
else
    A = zeros(2*N, 12);
    for i = 1:N
        pt = points3D(:,i);  % 변환 안된 원본 사용
        ns = nullspaces{i};
        row1 = 2*i - 1;
        row2 = 2*i;
        
        % 가중 행렬 설정 (대각 블록)
        if use_cov
            Kll(row1:row2, row1:row2) = cov_reduced(:,:,i);
        end
        
        % C++ 순서대로
        A(row1, 1) = ns(1,1) * pt(1);  % r11
        A(row2, 1) = ns(1,2) * pt(1);
        A(row1, 2) = ns(1,1) * pt(2);  % r12
        A(row2, 2) = ns(1,2) * pt(2);
        A(row1, 3) = ns(1,1) * pt(3);  % r13
        A(row2, 3) = ns(1,2) * pt(3);
        A(row1, 4) = ns(2,1) * pt(1);  % r21
        A(row2, 4) = ns(2,2) * pt(1);
        A(row1, 5) = ns(2,1) * pt(2);  % r22
        A(row2, 5) = ns(2,2) * pt(2);
        A(row1, 6) = ns(2,1) * pt(3);  % r23
        A(row2, 6) = ns(2,2) * pt(3);
        A(row1, 7) = ns(3,1) * pt(1);  % r31
        A(row2, 7) = ns(3,2) * pt(1);
        A(row1, 8) = ns(3,1) * pt(2);  % r32
        A(row2, 8) = ns(3,2) * pt(2);
        A(row1, 9) = ns(3,1) * pt(3);  % r33
        A(row2, 9) = ns(3,2) * pt(3);
        A(row1, 10) = ns(1,1);         % t1
        A(row2, 10) = ns(1,2);
        A(row1, 11) = ns(2,1);         % t2
        A(row2, 11) = ns(2,2);
        A(row1, 12) = ns(3,1);         % t3
        A(row2, 12) = ns(3,2);
    end
end

end