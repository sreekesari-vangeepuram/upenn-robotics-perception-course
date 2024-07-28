function [ warped_pts ] = warp_pts( video_pts, logo_pts, sample_pts)
% warp_pts computes the homography that warps the points inside
% video_pts to those inside logo_pts. It then uses this
% homography to warp the points in sample_pts to points in the logo
% image
% Inputs:
%     video_pts: a 4x2 matrix of (x,y) coordinates of corners in the
%         video frame
%     logo_pts: a 4x2 matrix of (x,y) coordinates of corners in
%         the logo image
%     sample_pts: a nx2 matrix of (x,y) coordinates of points in the video
%         video that need to be warped to corresponding points in the
%         logo image
% Outputs:
%     warped_pts: a nx2 matrix of (x,y) coordinates of points obtained
%         after warping the sample_pts
% Written for the University of Pennsylvania's Robotics:Perception course

% Complete est_homography first!
[ H ] = est_homography(video_pts, logo_pts);

% Convert sample_pts to homogeneous coordinates
sample_pts_hom = [sample_pts, ones(size(sample_pts, 1), 1)];

% Apply the homography matrix
warped_pts_hom = (H * sample_pts_hom')';

% Normalize the points
warped_pts_hom = warped_pts_hom ./ warped_pts_hom(:, 3);

% Extract the warped points
warped_pts = warped_pts_hom(:, 1:2);

end

