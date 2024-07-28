function E = EssentialMatrixFromFundamentalMatrix(F,K)
%% EssentialMatrixFromFundamentalMatrix
% Use the camera calibration matrix to esimate the Essential matrix
% Inputs:
%     K - size (3 x 3) camera calibration (intrinsics) matrix
%     F - size (3 x 3) fundamental matrix from EstimateFundamentalMatrix
% Outputs:
%     E - size (3 x 3) Essential matrix with singular values (1,1,0)


E = K' * F * K;

% Enforce singular values to (1, 1, 0)
[U, ~, V] = svd(E);
S = diag([1, 1, 0]);
E = U * S * V';

% Normalize E
E = E / norm(E);

end