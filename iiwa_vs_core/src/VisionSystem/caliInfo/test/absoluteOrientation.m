%
%Given the coordinates of 'n>=3' points in two coordinate systems
%find the transformation between them.
%We call the coordinate systems "left" and "right".
%We seek the rigid transformation T such that T*P_left = P_right
%This implementation is based on the paper "Closed-form solution of 
%absolute orientation using unit quaternions", B.K.P. Horn,
%Journal of the Optical Society of America, Vol. 4(4), pp 629--642, 1987.
% 
% @param pointsInLeft A 3xn matrix where the columns are the points in the
%                     first (left) coordinate system. Point correspondence
%                     is assumed (i.e. column i of pointsInLeft corresponds 
%                     to column i in pointsInRight).
%
% @param pointsInRight A 3xn matrix where the columns are the points in the
%                      second (right) coordinate system. Point correspondence
%                      is assumed (i.e. column i of pointsInRight corresponds 
%                      to column i in pointsInLeft).
%
% @return T The rigid transformation such that T*pointsInLeft = pointsInRight.
%           T is a 4x4 homogenous matrix representation of a rigid transformation. 
%           Note: You cannot directly apply T to the pointsInLeft array as the points
%                 are not given in homogenous coordinates.
%
% @author Ziv Yaniv (zivy@isis.georgetown.edu)
%
function [T] = absoluteOrientation(pointsInLeft,pointsInRight);

        %point sets must have same cardinality and it should be greater
        %than 3
if size(pointsInLeft,2) ~= size(pointsInRight,2) ||... 
   size(pointsInRight,2)<3
  error('Absolute orientation cannot be estimated due to size of point sets');
end

      %should add a sanity check here that ensures the point sets are not in a 
      %degenerate configuration (line, same point...)
      
numPoints = size(pointsInLeft,2);
meanLeft = mean(pointsInLeft')';
meanRight = mean(pointsInRight')';

M = pointsInLeft*pointsInRight';

M = M - numPoints*meanLeft*meanRight';

delta = [M(2,3) - M(3,2); M(3,1) - M(1,3);M(1,2) - M(2,1)];

N = [trace(M) delta'; delta (M+M'-trace(M)*eye(3))];

[eigenVectors, eigenValues] = eig(N);  
[dummy,index] = max(diag(eigenValues));
  
rotation = quaternionToMatrix(eigenVectors(:,index));
translation = meanRight - rotation*meanLeft;
T = [rotation, translation; [0, 0, 0, 1]];

%
% q = [s,qx,qy,qz]^T
%
function R = quaternionToMatrix(q)

  R(1,1) = 1-2*q(3)^2-2*q(4)^2;        R(1,2) = 2*q(2)*q(3)-2*q(1)*q(4);    R(1,3) = 2*q(2)*q(4)+2*q(1)*q(3);    
  R(2,1) = 2*q(2)*q(3)+2*q(1)*q(4);    R(2,2) = 1-2*q(2)^2-2*q(4)^2;        R(2,3) = 2*q(3)*q(4)-2*q(1)*q(2);    
  R(3,1) = 2*q(2)*q(4)-2*q(1)*q(3);    R(3,2) = 2*q(3)*q(4)+2*q(1)*q(2);    R(3,3) = 1-2*q(2)^2-2*q(3)^2;  
