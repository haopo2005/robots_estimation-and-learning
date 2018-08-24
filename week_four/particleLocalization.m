% Robotics: Estimation and Learning 
% WEEK 4
% 
% Complete this function following the instruction. 
function myPose = particleLocalization(ranges, scanAngles, map, param)

% Number of poses to calculate
N = size(ranges, 2);
% Output format is [x1 x2, ...; y1, y2, ...; z1, z2, ...]
myPose = zeros(3, N);
[h,w]=size(map);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Map Parameters 
% 
% % the number of grids for 1 meter.
myResolution = param.resol;
% % the origin of the map in pixels
myOrigin = param.origin; 
%myMap = zeros(size(map));
% The initial pose is given
myPose(:,1) = param.init_pose;
% You should put the given initial pose into myPose for j=1, ignoring the j=1 ranges. 
% The pose(:,1) should be the pose when ranges(:,j) were measured.
% thresh = graythresh(map);     %自动确定二值化阈值
% I2 = im2bw(map,thresh);       %对图像二值化
auto_thresh = mode(reshape(map, size(map,1)*size(map,2), 1));
% Decide the number of particles, M.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
M = 800; %more big num,more robust and stable, 1500 is better      % Please decide a reasonable number of M, 
                               % based on your experiment using the practice data.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create M number of particles
P = repmat(myPose(:,1), [1, M]);
weights = zeros(1,M);
acc_weights = zeros(1,M);
for k=1:1:M
    weights(k) = 1/M;
end

% num_lines = size(ranges,1);
% i_occ=zeros(size(ranges,1),2,M);
for j = 2:N % You will start estimating myPose from j=2 using ranges(:,2).

    % 1) Propagate the particles
    noise_sigma = diag([0.1 0.1 0.02]);
    rand_matrix =  noise_sigma*randn(3,M);
    P = P + rand_matrix; 

    % 2) Measurement Update 
    %   2-1) Find grid cells hit by the rays (in the grid map coordinate frame)
    for m = 1:M
         d_occ = 0;
         x_o = ranges(:,j) .* cos(scanAngles + P(3,m)) + P(1,m);
         y_o = -ranges(:,j) .* sin(scanAngles + P(3,m)) + P(2,m);
            
         occ_x = ceil(x_o*myResolution)+myOrigin(1);
         occ_y = ceil(y_o*myResolution)+myOrigin(2);
         occ_x_ = occ_x';
         occ_y_ = occ_y';
         del_occ =  occ_x_<1 | occ_y_<1 |  occ_x_ > size(map,2) |  occ_y_ > size(map,1);
         
         occ_x_(del_occ) = [];
         occ_y_(del_occ) = [];
         occupied = sub2ind(size(map),occ_y_',occ_x_');
         d_occ =  d_occ - sum(sum(map(occupied) <= auto_thresh));
         d_occ =  d_occ + sum(sum(map(occupied) > auto_thresh))*10;
         weights(m) = max(0,d_occ);
    end
%     for k = 1:num_lines
%             Loc_x = ranges(k,j).*cos(P(3,:)+scanAngles(k))+P(1,:);
%             Loc_y = -ranges(k,j).*sin(P(3,:)+scanAngles(k))+P(2,:); 
%             temp_x = ceil(Loc_x*myResolution) + myOrigin(1);
%             temp_y = ceil(Loc_y*myResolution) + myOrigin(2);
%             for mm=1:1:M
%                 if temp_x(mm) >=1 && temp_x(mm) < size(map,2)
%                     i_occ(k,1,mm) = temp_x(mm);
%                     %flag(k,mm) = flag(k,mm)+1;
%                 end
%                 if temp_y(mm) >=1 && temp_y(mm) < size(map,1)
%                     i_occ(k,2,mm) = temp_y(mm);
%                     %flag(k,mm) = flag(k,mm)+1;
%                 end
%             end
%     end
%     for q=1:1:M
%             d_occ = 0;
%             occupied = sub2ind(size(map),i_occ(:,2,q),i_occ(:,1,q));
%             d_occ =  d_occ - sum(sum(map(occupied) <= auto_thresh))*1;
%             d_occ =  d_occ + sum(sum(map(occupied) > auto_thresh))*20;  
%             weights(q) = d_occ;
%     end
    weights = weights / sum(weights);
    %[Max_,Ind_] = max(weights);
    %disp(Max_)
    %myPose(:,j) = P(:,Ind_);
    myPose(:,j) = P*weights';
%     P = repmat(myPose(:,j), [1, M]); 
    acc_weights(1) = weights(1);
    for k=2:1:M
        acc_weights(k) =  acc_weights(k-1) + weights(k);
    end
    acc_weights = acc_weights / sum(weights);
   
    qq = P;
    for k=1:1:M
        temp_index = M;
        temp = rand();%产生0-1的随机数，使得该随机数正好在某一阶段累积权重之间
        for q=1:1:M
            if temp <= acc_weights(q)
                temp_index = q;
                break;
            end
        end
        P(:,k) = qq(:,temp_index);
    end
    %   2-2) For each particle, calculate the correlation scores of the particles

    %   2-3) Update the particle weights         
 
    %   2-4) Choose the best particle to update the pose
    
    % 3) Resample if the effective number of particles is smaller than a threshold

    % 4) Visualize the pose on the map as needed
   

end

end

