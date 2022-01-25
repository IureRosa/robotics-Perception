function F = EstimateFundamentalMatrix(x1, x2)
%% EstimateFundamentalMatrix
% Estimate the fundamental matrix from two image point correspondences 
% Inputs:
%     x1 - size (N x 2) matrix of points in image 1
%     x2 - size (N x 2) matrix of points in image 2, each row corresponding
%       to x1
% Output:
%    F - size (3 x 3) fundamental matrix with rank 2

N = size(x1,1);
A = ones(N,9);

for i = 1 : N
    A(i,1) = x2(i,1)*x1(i,1);
    A(i,2) = x2(i,2)*x1(i,1);
    A(i,3) = x1(i,1);
    A(i,4) = x2(i,1)*x1(i,2);
    A(i,5) = x2(i,2)*x1(i,2);
    A(i,6) = x1(i,2);
    A(i,7) = x2(i,1);
    A(i,8) = x2(i,2);
end

[~,~,V] = svd(A);
F = V(:,9);
F = reshape(F,3,3);
[U,~,V] = svd(F);
F = U*[1,0,0;0,1,0;0,0,0]*V';
F = F/norm(F);
end
