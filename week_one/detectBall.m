% Robotics: Estimation and Learning 
% WEEK 1
% 
% Complete this function following the instruction. 
function [segI, loc] = detectBall(I)
% function [segI, loc] = detectBall(I)
%
% INPUT
% I       120x160x3 numerial array 
%
% OUTPUT
% segI    120x160 numeric array
% loc     1x2 or 2x1 numeric array 



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hard code your learned model parameters here
%
% mu = 
% sig = 
% thre = 
k = 4
param = open('para4.mat');
mu = param.mu;
sig = param.sigma;
thre = 3e-06;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Find ball-color pixels using your model
% 
%test=double(I);
test = rgb2hsv(I);
[r,c,d] = size(test);
segI = zeros(r,c);
aa = [];
for i=1:1:r
    for j=1:1:c
        temp = 0;
        for m=1:1:k
            temp = temp + g_kx(mu(m,:),sig(:,:,m),test(i,j))/k;
        end
        aa = [aa;temp];
        if temp > thre
            segI(i,j) = 255;
        end
    end
end
%¸¯Ê´²Ù×÷
B = [0 1 0 1 1 1 0 1 0];
segI = imerode(segI,B);
% figure,
% imshow(segI)
CC = bwconncomp(segI);
numPixels = cellfun(@numel,CC.PixelIdxList);
[biggest,idx] = max(numPixels);
bw_biggest(CC.PixelIdxList{idx}) = true; 
% figure,
% imshow(bw_biggest); hold on;
S = regionprops(CC,'Centroid');
loc = S(idx).Centroid;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Do more processing to segment out the right cluster of pixels.
% You may use the following functions.
%   bwconncomp
%   regionprops
% Please see example_bw.m if you need an example code.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute the location of the ball center
%

% segI = 
%loc = []; 
% 
% Note: In this assigment, the center of the segmented ball area will be considered for grading. 
% (You don't need to consider the whole ball shape if the ball is occluded.)

end
