function [mu, sigma] = prediction_step(mu, sigma, u)
% Updates the belief concerning the robot pose according to the motion model,
% mu: 2N+3 x 1 vector representing the state mean
% sigma: 2N+3 x 2N+3 covariance matrix
% u: odometry reading (r1, t, r2)
% Use u.r1, u.t, and u.r2 to access the rotation and translation values

% TODO: Compute the new mu based on the noise-free (odometry-based) motion model
% Remember to normalize theta after the update (hint: use the function normalize_angle available in tools)
N = size(mu, 1);%2N+3
Fx = [eye(3) zeros(3, N-3)];
mu_t_1 = mu;
mu = mu + Fx'*[u.t*cos(mu(3)+u.r1); u.t*sin(mu(3)+u.r1); u.r1+u.r2];
mu(3) = normalize_angle(mu(3));
% TODO: Compute the 3x3 Jacobian Gx of the motion model
Gx = zeros(3);
Gx(1,3) = -u.t*sin(mu_t_1(3)+u.r1);
Gx(2,3) = u.t*cos(mu_t_1(3)+u.r1);

% TODO: Construct the full Jacobian G
I = eye(N,N);
G = I + Fx'*Gx*Fx;

% Motion noise
motionNoise = 0.1;
R3 = [motionNoise, 0, 0; 
     0, motionNoise, 0; 
     0, 0, motionNoise/10];
R = zeros(size(sigma,1));
R(1:3,1:3) = R3;

% TODO: Compute the predicted sigma after incorporating the motion
sigma = G*sigma*G'+R;

end
