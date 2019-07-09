% Compute the error of a pose-pose constraint
% x1 3x1 vector (x,y,theta) of the first robot pose
% x2 3x1 vector (x,y,theta) of the second robot pose
% z 3x1 vector (x,y,theta) of the measurement
%
% You may use the functions v2t() and t2v() to compute
% a Homogeneous matrix out of a (x, y, theta) vector
% for computing the error.
%
% Output
% e 3x1 error of the constraint
% A 3x3 Jacobian wrt x1
% B 3x3 Jacobian wrt x2
function [e, A, B] = linearize_pose_pose_constraint(x1, x2, z)

  % TODO compute the error and the Jacobians of the error
  X1 = v2t(x1);  % the first robot pose
  X2 = v2t(x2);  % the second robot pose
  Z = v2t(z);
  R1 = X1(1:2, 1:2);
  t1 = X1(1:2, 3);
  t2 = X2(1:2, 3);
  R12 = Z(1:2, 1:2);
  t12 = Z(1:2, 3);
  theta1 = x1(3);
  theta2 = x2(3);
  theta12 = z(3);
  
  %e = [transpose(R12)*(transpose(R1)*(t2-t1)-t12); theta2-theta1-theta12];
  
  e = t2v(Z\(X1\X2));
  
  R_A = [-sin(x1(3)) -cos(x1(3));cos(x1(3)) -sin(x1(3))];
  temp_A = transpose(R12)*transpose(R_A)*(t2-t1);
  temp_B = transpose(R12)*transpose(R1);
  
  A = [-temp_B(1,:) temp_A(1);-temp_B(2,:) temp_A(2);0 0 -1];
  B = [temp_B(1,:) 0;temp_B(2,:) 0;0 0 1];
end;
