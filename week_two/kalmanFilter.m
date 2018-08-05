function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
%UNTITLED Summary of this function goes here
%   Four dimensional state: position_x, position_y, velocity_x, velocity_y

    %% Place parameters like covarainces, etc. here:
    % P = eye(4)
    % R = eye(2)
    Bu = [0,0,0.5,0]';
    %Bu = [0,0,0,0]';
    % Check if the first time running this function
    if previous_t<0
        state = [x, y, 0, 0]';
        param.P = 250 * eye(4);
        param.A = [[1,0,0,0]',[0,1,0,0]',[0.033,0,1,0]',[0,0.033,0,1]'];
        param.C = [[1,0]',[0,1]',[0,0]',[0,0]'];
        param.R = [[0.05,0]',[0,0.05]'];
        param.sigma_m = 0.5*eye(4);
        predictx = x;
        predicty = y;
        return;
    else
        dt = t - previous_t;
        param.A = [[1,0,0,0]',[0,1,0,0]',[dt,0,1,0]',[0,dt,0,1]'];
        z_t = [x y]';
        
        X = param.A*state + Bu;
        P = param.A*param.P*param.A' + param.sigma_m;
        
        K = P*param.C'*inv(param.C*P*param.C'+param.R);
        state = X + K*(z_t - param.C*X);
        param.P = (eye(4)-K*param.C)*P;
        
        predictx = state(1);
        predicty = state(2);
    end

    %% TODO: Add Kalman filter updates
%     % As an example, here is a Naive estimate without a Kalman filter
%     % You should replace this code
%     vx = (x - state(1)) / (t - previous_t);
%     vy = (y - state(2)) / (t - previous_t);
%     % Predict 330ms into the future
%     predictx = x + vx * 0.330;
%     predicty = y + vy * 0.330;

    
    % State is a four dimensional element
    %state = [x, y, vx, vy];
end
