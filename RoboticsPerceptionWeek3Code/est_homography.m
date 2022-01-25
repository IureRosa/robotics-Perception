function [ H ] = est_homography(video_pts, logo_pts)
% est_homography estimates the homography to transform each of the
% video_pts into the logo_pts
% Inputs:
%     video_pts: a 4x2 matrix of corner points in the video
%     logo_pts: a 4x2 matrix of logo points that correspond to video_pts
% Outputs:
%     H: a 3x3 homography matrix such that logo_pts ~ H*video_pts
% Written for the University of Pennsylvania's Robotics:Perception course

% YOUR CODE HERE
H = [];
A = [];
for i = 1:length(video_pts)
    xv = video_pts(i,1);
    yv = video_pts(i,2);
    xl = logo_pts(i,1);
    yl = logo_pts(i,2);
    ax = [ -xv, -yv, -1, 0, 0, 0, xv*xl, yv*xl, xl];
    ay = [ 0, 0, 0, -xv, -yv, -1, xv*yl, yv*yl, yl];
    A = cat(1,A,ax,ay);
end

[U,S,V] = svd(A);
h = V(:,9);
h = reshape(h,3,3);
H = h';

