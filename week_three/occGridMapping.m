% Robotics: Estimation and Learning 
% WEEK 3
% 
% Complete this function following the instruction. 
function myMap = occGridMapping(ranges, scanAngles, pose, param)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Parameters 

% the number of grids for 1 meter.
myResol = param.resol;
% the initial map size in pixels
myMap = zeros(param.size);
% the origin of the map in pixels
myorigin = param.origin; 

% 4. Log-odd parameters 
lo_occ = param.lo_occ;
lo_free = param.lo_free; 
lo_max = param.lo_max;
lo_min = param.lo_min;

N = size(pose,2);
num_lines = size(ranges,1);
i_occ=zeros(size(ranges,1),2);
for j = 1:N % for each time,
    start_x = ceil(pose(1,j)*myResol)+myorigin(1);
    start_y = ceil(pose(2,j)*myResol)+myorigin(2);
    for k = 1:num_lines
            Loc_x = ranges(k,j).*cos(pose(3,j)+scanAngles(k))+pose(1,j);
            Loc_y = -ranges(k,j).*sin(pose(3,j)+scanAngles(k))+pose(2,j); 
            i_occ(k,1) = ceil(Loc_x*myResol) + myorigin(1);
            i_occ(k,2) = ceil(Loc_y*myResol) + myorigin(2);
    end
    for k = 1:size(i_occ,1)
            [freex, freey] = bresenham(start_x,start_y,i_occ(k,1),i_occ(k,2));
            free = sub2ind(size(myMap),freey,freex);
            occupied = sub2ind(size(myMap),i_occ(k,2),i_occ(k,1));
            myMap(occupied) = min(myMap(occupied) + lo_occ,lo_max);
            myMap(free) = max(myMap(free) - lo_free,lo_min);
    end
%     figure(2),
%     imagesc(myMap); hold on;
%     plot(myorigin(1),myorigin(2),'rx','LineWidth',3); % indicate start point
%     axis equal;
end

end

