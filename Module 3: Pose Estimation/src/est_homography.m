function [ H ] = est_homography(video_pts, logo_pts)
% est_homography estimates the homography to transform each of the
% video_pts into the logo_pts
% Inputs:
%     video_pts: a 4x2 matrix of corner points in the video
%     logo_pts: a 4x2 matrix of logo points that correspond to video_pts
% Outputs:
%     H: a 3x3 homography matrix such that logo_pts ~ H*video_pts
% Written for the University of Pennsylvania's Robotics:Perception course

% Number of points
num_pts = size(video_pts, 1);

% Preallocate matrix A
A = zeros(2 * num_pts, 9);

% Fill in the matrix A
for i = 1:num_pts
    x = video_pts(i, 1);
    y = video_pts(i, 2);
    xp = logo_pts(i, 1);
    yp = logo_pts(i, 2);
    A(2*i-1, :) = [-x, -y, -1, 0, 0, 0, x*xp, y*xp, xp];
    A(2*i, :) = [0, 0, 0, -x, -y, -1, x*yp, y*yp, yp];
end

% Use SVD to solve for h
[~, ~, V] = svd(A);
h = V(:, end);

% Reshape h into the homography matrix H
H = reshape(h, 3, 3)';

end

