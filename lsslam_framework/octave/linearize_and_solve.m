% performs one iteration of the Gauss-Newton algorithm
% each constraint is linearized and added to the Hessian

function dx = linearize_and_solve(g)

nnz = nnz_of_graph(g);

% allocate the sparse H and the vector b
H = spalloc(length(g.x), length(g.x), nnz);
b = zeros(length(g.x), 1);

needToAddPrior = true;

% compute the addend term to H and b for each of our constraints
disp('linearize and build system');
for eid = 1:length(g.edges)
  edge = g.edges(eid);

  % pose-pose constraint
  if (strcmp(edge.type, 'P') != 0)
    % edge.fromIdx and edge.toIdx describe the location of
    % the first element of the pose in the state vector
    % You should use also this index when updating the elements
    % of the H matrix and the vector b.
    % edge.measurement is the measurement
    % edge.information is the information matrix
    x1 = g.x(edge.fromIdx:edge.fromIdx+2);  % the first robot pose
    x2 = g.x(edge.toIdx:edge.toIdx+2);      % the second robot pose

    % Computing the error and the Jacobians
    % e the error vector
    % A Jacobian wrt x1
    % B Jacobian wrt x2
    [e, A, B] = linearize_pose_pose_constraint(x1, x2, edge.measurement);


    % TODO: compute and add the term to H and b
%    J = zeros(3, length(g.x));
%    J(:,edge.fromIdx:edge.fromIdx+2) = A;
%    J(:,edge.toIdx:edge.toIdx+2) = B;
%    
%    b = b + transpose(e)*edge.information*J; %b-->b_T
%    H = H + transpose(J)*edge.information*J;
    ii = edge.fromIdx:edge.fromIdx+2;
    jj = edge.toIdx:edge.toIdx+2;
    
    b(ii) = b(ii) + (e'*edge.information*A)';
    b(jj) = b(jj) + (e'*edge.information*B)';
    
    H(ii,ii) = H(ii,ii) + A'*edge.information*A;
    H(jj,jj) = H(jj,jj) + B'*edge.information*B;
    H(ii,jj) = H(ii,jj) + A'*edge.information*B;
    H(jj,ii) = H(jj,ii) + B'*edge.information*A;

    if (needToAddPrior)
      % TODO: add the prior for one pose of this edge
      % This fixes one node to remain at its current location
%      temp_H = zeros(size(H));
%      temp_H(1,1) = 1;
%      temp_H(2,1) = 1;
%      temp_H(3,1) = 1;
%      H = H + temp_H;
      H(1,1) = H(1,1)+1;
      H(2,2) = H(2,2)+1;
      H(3,3) = H(3,3)+1;
      needToAddPrior = false;
    end

  % pose-landmark constraint
  elseif (strcmp(edge.type, 'L') != 0)
    % edge.fromIdx and edge.toIdx describe the location of
    % the first element of the pose and the landmark in the state vector
    % You should use also this index when updating the elements
    % of the H matrix and the vector b.
    % edge.measurement is the measurement
    % edge.information is the information matrix
    x1 = g.x(edge.fromIdx:edge.fromIdx+2);  % the robot pose
    x2 = g.x(edge.toIdx:edge.toIdx+1);      % the landmark

    % Computing the error and the Jacobians
    % e the error vector
    % A Jacobian wrt x1
    % B Jacobian wrt x2
    [e, A, B] = linearize_pose_landmark_constraint(x1, x2, edge.measurement);


    % TODO: compute and add the term to H and b
%    J = zeros(2, length(g.x));
%    J(:,edge.fromIdx:edge.fromIdx+2) = A;
%    J(:,edge.toIdx:edge.toIdx+1) = B;
%    
%    b = b + transpose(e)*edge.information*J; %b-->b_T
%    H = H + transpose(J)*edge.information*J;
    ii = edge.fromIdx:edge.fromIdx+2;
    jj = edge.toIdx:edge.toIdx+1;
    
    b(ii) = b(ii) + (e'*edge.information*A)';
    b(jj) = b(jj) + (e'*edge.information*B)';
    
    H(ii,ii) = H(ii,ii) + A'*edge.information*A;
    H(jj,jj) = H(jj,jj) + B'*edge.information*B;
    H(ii,jj) = H(ii,jj) + A'*edge.information*B;
    H(jj,ii) = H(jj,ii) + B'*edge.information*A;
  end
end

disp('solving system');

% TODO: solve the linear system, whereas the solution should be stored in dx
% Remember to use the backslash operator instead of inverting H
%dx = -inv(H) * b;
dx = -H\b;

end
