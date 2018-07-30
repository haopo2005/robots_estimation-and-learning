% Robotics: Estimation and Learning 
% WEEK 3
% 
% This is an example code for collecting ball sample colors using roipoly
close all
clear;
clc;
%{
imagepath = './train';
Samples = [];
for k=1:19
    % Load image
    I = imread(sprintf('%s/%03d.png',imagepath,k));
    
    % You may consider other color space than RGB
    R = I(:,:,1);
    G = I(:,:,2);
    B = I(:,:,3);
    
    % Collect samples 
    disp('');
    disp('INTRUCTION: Click along the boundary of the ball. Double-click when you get back to the initial point.')
    disp('INTRUCTION: You can maximize the window size of the figure for precise clicks.')
    figure(1), 
    mask = roipoly(I); 
    figure(2), imshow(mask); title('Mask');
    sample_ind = find(mask > 0);
    
    R = R(sample_ind);
    G = G(sample_ind);
    B = B(sample_ind);
    
    Samples = [Samples; [R G B]];
    
    disp('INTRUCTION: Press any key to continue. (Ctrl+c to exit)')
    pause
end
save('Samples.mat', 'Samples')
%}
tSamples = open('Samples.mat');
Samples = rgb2hsv(tSamples.Samples);

% visualize the sample distribution
% figure, 
% scatter3(Samples(:,1),Samples(:,2),Samples(:,3),'.');
% title('Pixel Color Distribubtion');
% xlabel('Red');
% ylabel('Green');
% zlabel('Blue');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [IMPORTANT]
%
% Now choose you model type and estimate the parameters (mu and Sigma) from
% the sample data.
%
k=6;
[r,d] = size(Samples);
mu = randn(k,d,'double');
sigma1 = randn(d,'double');
sigma2 = randn(d,'double');
sigma3 = randn(d,'double');
sigma4 = randn(d,'double');
% sigma5 = randn(d,'double');
% sigma6 = randn(d,'double');
%sigma必须正定，否则行列式值为负数, sigma这边目前是手动赋值的
sigma(:,:,1) = sigma1'*sigma1;
sigma(:,:,2) = sigma2'*sigma2;
sigma(:,:,3) = sigma3'*sigma3;
sigma(:,:,4) = sigma4'*sigma4;
% sigma(:,:,5) = sigma5'*sigma5;
% sigma(:,:,6) = sigma6'*sigma6;

z_ki=zeros(r,k);
zk = zeros(k,1);
%迭代10次，盼望收敛
for s=1:1:100
    %e-step
    for i = 1:1:r
        temp = 0;
        for j=1:1:k
            temp = temp + g_kx(mu(j,:),sigma(:,:,j),Samples(i,:));
        end
        for j=1:1:k
            z_ki(i,j) = g_kx(mu(j,:),sigma(:,:,j),Samples(i,:))/temp;
        end
    end

    %m-step
    for j=1:1:k
        zk(j) = sum(z_ki(:,j));
    end
    %sigma和mu要同时更新，不能mu更新了，sigma还不更新
    mu = zeros(k,d);
    sigma = zeros(d,d,k);
    for i=1:1:r   
        for j=1:1:k
            mu(j,:) = mu(j,:) + Samples(i,:).*z_ki(i,j);
        end
    end
    for i=1:1:r
        for j=1:1:k
            sigma(:,:,j) = sigma(:,:,j) + (Samples(i,:)-mu(j,:))'*(Samples(i,:)-mu(j,:)).*z_ki(i,j);
        end
    end
    for j=1:1:k
        mu(j,:) = mu(j,:)/zk(j);
        sigma(:,:,j) = sigma(:,:,j)/zk(j);
    end
end
save('para.mat','mu','sigma')

%验证threshold
aaa = [];
for i=1:1:r
    temp=0;
    for j=1:1:k
        temp = temp + g_kx(mu(j,:),sigma(:,:,j),Samples(i,:))/k;
    end
    aaa = [aaa temp];
end
median(aaa)
aaa=sort(aaa');
