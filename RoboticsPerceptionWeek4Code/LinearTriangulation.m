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

N = size(x1,1);

M1 = [R1,-R1*C1];
P1 = K*M1;
M2 = [R2,-R2*C2];
P2 = K*M2;



for i = 1 : N
    pixel1 = cross_matrix([x1(i,:),1]);
    result1 = pixel1*P1;
   
    pixel2 = cross_matrix([x2(i,:),1]);
    result2 = pixel2*P2;
    
    A = [result1;result2]; 
    
    [~,~,V] = svd(A);
    
    X(i,:) = (V(1:3,4)/V(4,4))';
end

end

