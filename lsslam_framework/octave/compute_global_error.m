% Computes the total error of the graph
function Fx = compute_global_error(g)

Fx = 0;

% Loop over all edges
for eid = 1:length(g.edges)
  edge = g.edges(eid);

  % pose-pose constraint
  if (strcmp(edge.type, 'P') != 0)

    x1 = v2t(g.x(edge.fromIdx:edge.fromIdx+2));  % the first robot pose
    x2 = v2t(g.x(edge.toIdx:edge.toIdx+2));      % the second robot pose

    %TODO compute the error of the constraint and add it to Fx.
    % Use edge.measurement and edge.information to access the
    % measurement and the information matrix respectively.
%    z = v2t(edge.measurement);
%    R1 = x1(1:2, 1:2);
%    t1 = x1(1:2, 3);
%    t2 = x2(1:2, 3);
%    R12 = z(1:2, 1:2);
%    t12 = z(1:2, 3);
%    theta1 = g.x(edge.fromIdx:edge.fromIdx+2)(3);
%    theta2 = g.x(edge.toIdx:edge.toIdx+2)(3);
%    theta12 = edge.measurement(3);
%    
%    err = [transpose(R12)*(transpose(R1)*(t2-t1)-t12); theta2-theta1-theta12];
%    Fx = Fx + transpose(err)*edge.information*err; 
    err_tmp = t2v(v2t(edge.measurement)\(x1\x2)); %反斜杠比inv效率更高，对于Ax=b问题，哪怕A不是方阵，都可以用反斜杠
    ei = err_tmp'*edge.information*err_tmp;
    Fx = Fx + ei;
  % pose-landmark constraint
  elseif (strcmp(edge.type, 'L') != 0)
    x = g.x(edge.fromIdx:edge.fromIdx+2);  % the robot pose
    l = g.x(edge.toIdx:edge.toIdx+1);      % the landmark

    %TODO compute the error of the constraint and add it to Fx.
    % Use edge.measurement and edge.information to access the
    % measurement and the information matrix respectively.
    x1 = v2t(x);
    R1 = x1(1:2, 1:2);
    t1 = x1(1:2, 3);
    z = edge.measurement;
    
    err = transpose(R1)*(l-t1)-z;
    Fx = Fx + transpose(err)*edge.information*err;
%    Z = v2t([edge.measurement;0]);
%    X = v2t(x);
%    L = v2t([l;0]);
%    err_tmp = t2v(Z\(X\L));
%    err_tmp = err_tmp(1:2);

%    ei = err_tmp'*edge.information*err_tmp;
%    Fx = Fx + ei;
  end

end
