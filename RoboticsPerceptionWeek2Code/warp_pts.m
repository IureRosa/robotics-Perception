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

% YOUR CODE HERE
%{
fprintf("%f ",norm(H(:,1)));
fprintf("%f ",norm(H(:,2)));
fprintf("%f\n",norm(H(:,3)));
%}
warped_pts = [];

for i = 1 : length(sample_pts)
    a = [sample_pts(i,:),1];
    %fprintf('%f',size(H));
    b = H*a';
    warped_pts(i,:)=[b(1)/b(3),b(2)/b(3)];
end

