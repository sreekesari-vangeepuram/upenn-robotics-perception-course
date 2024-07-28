function X = LinearTriangulation(K, C1, R1, C2, R2, x1, x2)
%% LinearTriangulation
% Find 3D positions of the point correspondences using the relative
% position of one camera from another
% Inputs:
%     C1 - size (3 x 1) translation of the first camera pose
%     R1 - size (3 x 3) rotation of the first camera pose
%     C2 - size (3 x 1) translation of the second camera
%     R2 - size (3 x 3) rotation of the second camera pose
%     x1 - size (N x 2) matrix of points in image 1
%     x2 - size (N x 2) matrix of points in image 2, each row corresponding
%       to x1
% Outputs: 
%     X - size (N x 3) matrix whos rows represent the 3D triangulated
%       points


% Camera projection matrices
P1 = K * [R1, -R1*C1];
P2 = K * [R2, -R2*C2];

% Ensure the input points are in homogeneous coordinates
x1 = [x1, ones(size(x1, 1), 1)];
x2 = [x2, ones(size(x2, 1), 1)];

% Number of correspondences
N = size(x1, 1);

% Triangulated 3D points
X = zeros(N, 3);

for i = 1:N
    A = [x1(i, 1)*P1(3,:) - P1(1,:); ...
         x1(i, 2)*P1(3,:) - P1(2,:); ...
         x2(i, 1)*P2(3,:) - P2(1,:); ...
         x2(i, 2)*P2(3,:) - P2(2,:)];
    
    [~, ~, V] = svd(A);
    X_hom = V(:, end);
    X(i, :) = X_hom(1:3)' / X_hom(4);
end

end