function [C, R] = LinearPnP(X, x, K)
%% LinearPnP
% Getting pose from 2D-3D correspondences
% Inputs:
%     X - size (N x 3) matrix of 3D points
%     x - size (N x 2) matrix of 2D points whose rows correspond with X
%     K - size (3 x 3) camera calibration (intrinsics) matrix
% Outputs:
%     C - size (3 x 1) pose transation
%     R - size (3 x 1) pose rotation
%
% IMPORTANT NOTE: While theoretically you can use the x directly when solving
% for the P = [R t] matrix then use the K matrix to correct the error, this is
% more numeically unstable, and thus it is better to calibrate the x values
% before the computation of P then extract R and t directly

% Number of correspondences
n = size(x, 1);

% Initialize matrix A
A = zeros(2 * n, 12);

% Constructing matrix A
for i = 1:n
    A(2*i-1, 1:4) = [0, 0, 0, 0];
    A(2*i-1, 5:8) = -1 * [X(i, :), 1];
    A(2*i-1, 9:12) = x(i, 2) * [X(i, :), 1];

    A(2*i, 1:4) = [X(i, :), 1];
    A(2*i, 5:8) = [0, 0, 0, 0];
    A(2*i, 9:12) = -1 * x(i, 1) * [X(i, :), 1];
end

% Singular Value Decomposition
[~, ~, V] = svd(A);

% Extracting the last column of V
p = V(:, end) / V(end, end);

% Reshape p into the projection matrix P
P = [p(1:4, :)'; p(5:8, :)'; p(9:12, :)'];

% Decompose the projection matrix to extract R and t
P_a = K \ P;
R_b = P_a(1:3, 1:3);

% Enforce orthogonality using SVD
[U2, ~, V2] = svd(R_b);

if det(U2 * V2.') > 0
    R = U2 * V2.';
    t = P_a(:, 4) / norm(R_b);
else
    R = -U2 * V2.';
    t = -P_a(:, 4) / norm(R_b);
end

% Compute the camera center
C = -R.' * t;

end