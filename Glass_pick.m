close all;
clear all;
clc;

% Generating the cone of the necessary dimensions
t = 0:0.1:6;
[X,Y,Z] = cylinder(4+t);
[m,n] = size(Z);

% Increasing the height of the cone as it was of 'unit' height
for i = 2:m
    Z(i,:) = Z(i,:)*15;
end

% Getting the components of the normal at a particular point
[Nx,Ny,Nz] = surfnorm(X,Y,Z);
array_new = reshape([X,Y,Z],[m*n,3]);
array_normals = reshape([Nx,Ny,Nz],[m*n,3]);

% Converting the reshaped points forming the surface of our cylinder into a
% point cloud.
ptCloud = pointCloud(array_new);
% Finding the geometric mean of our point cloud:

C = mean(array_new);
C_new = [0 ,0 ,15 - 5.1923];
center = pointCloud(C);

% The selected points for grasping
selected_points = [8.9000         0   12.2500;
                   7.4000         0    8.5000;
                   5.9000         0    4.7500;
                  -7.4000    0.0000    8.5000];
                 
% Components of the selected normals:

selected_normals = [0.9285    0.0069   -0.3714;
                    0.9285    0.0069   -0.3714;
                    0.9285    0.0069   -0.3714;
                   -0.9285    0.0069   -0.3714];
figure;
pcshow(ptCloud);
hold on;
pcshow(C,'r');
hold on;
pcshow(C_new,'b');
hold on;
pcshow(selected_points,'[1,0,0]')
hold on;
axis square
title('Glass with Default Color Map');
xlabel('X');
ylabel('Y');
zlabel('Z');
hold on;

x = selected_points(1:end,1);
y = selected_points(1:end,2);
z = selected_points(1:end,3);
u = selected_normals(1:end,1);
v = selected_normals(1:end,2);
w = selected_normals(1:end,3);

quiver3(x,y,z,-u,-v,-w);
hold off

matrix_1 = getOrthogonal(selected_normals(1,:));
matrix_2 = getOrthogonal(selected_normals(4,:));

R_OC1 = [matrix_1,selected_normals(1,:)'];
r3 = [ -0.9285; 0.0069; -0.3714];
r1 = matrix_2(:,2);
r2 = cross(r3,r1);
R_OC4 = [r1,r2,r3];

%Calculating the position vectors:
p_OC1 = selected_points(1,:) - C_new;
p_OC2 = selected_points(2,:) - C_new;
p_OC3 = selected_points(3,:) - C_new;
p_OC4 = selected_points(4,:) - C_new;

%% 
x = 'SF';

p_OC1 = p_OC1';
p_OC2 = p_OC2';
p_OC3 = p_OC3';
p_OC4 = p_OC4';

% p_OC3 = [5.5667;   0.0000;   -2.7500];
% p_OC2 = [7.0667;   0.0000;    1.0000];
% p_OC1 = [8.5667;   0.0000;    4.7500];
% p_OC4 = [-7.7333;  0.0000;    1.0000];

% p_OC1 = [-0.2448;   -3.7279;   -0.2250];
% p_OC2 = [-0.2448;   -5.4558;    0.0500];
% p_OC3 = [-0.2448;   -7.1836;    0.3250];
% p_OC4 = [-0.2448;    5.6128;    0.0750];

% p_OC1 = [-8.7977;   -2.7503;    4.7500];
% p_OC2 = [6.2290;    2.1322;   -0.2500];
% p_OC3 = [8.1311;    2.7503;    4.7500];
% p_OC4 = [-7.4662;   -2.3176;    1.2500];


R_OC1 = [-0.0069    0.3714    0.9285
           1.0000    0.0013    0.0069
           0.0013    0.9285   -0.3714];
    
% R_OC1 = [0,-1,0;
%          0,0,-1;
%          1,0,0];
     
% R_OC1 = [-0.3714   -0.2869   -0.8830
%           -0.0566    0.9563   -0.2869
%           0.9267   -0.0566   -0.3714];
%      
% R_OC2 = [0.9285   -0.0069    0.3714;
%          0.0069    1.0000    0.0013;
%         -0.3714    0.0013    0.9285];

% R_OC2 = [0,-1,0;
%          0,0,-1;
%          1,0,0];

 R_OC2 =  [-0.0069    0.3714    0.9285
           1.0000    0.0013    0.0069
           0.0013    0.9285   -0.3714];

% R_OC3 = [0.9285   -0.0069    0.3714;
%          0.0069    1.0000    0.0013;
%         -0.3714    0.0013    0.9285];

% R_OC3 = [0,-1,0;
%          0,0,-1;
%          1,0,0];


R_OC3 =  [-0.0069    0.3714    0.9285
           1.0000    0.0013    0.0069
           0.0013    0.9285   -0.3714];

% R_OC4 = [-0.9285   -0.0069    0.3714;
%           0.0069    1.0000   -0.0013;
%          -0.3714    0.0013   -0.9285];
     
% R_OC4 = [-1,0,0;
%          0,0,1;
%          0,1,0];
 R_OC4 = [-0.3714    0.0069   -0.9285
           0.0013    1.0000    0.0069
           0.9285    0.0013   -0.3714];

p_OC1_hat = skewSymmetric(p_OC1);
p_OC2_hat = skewSymmetric(p_OC2);
p_OC3_hat = skewSymmetric(p_OC3);
p_OC4_hat = skewSymmetric(p_OC4);

G_1 = GraspMap(R_OC1, p_OC1_hat, x);
G_2 = GraspMap(R_OC2, p_OC2_hat, x);
G_3 = GraspMap(R_OC3, p_OC3_hat, x);
G_4 = GraspMap(R_OC4, p_OC4_hat, x);

G = [G_1,G_2,G_3,G_4];

F_external = [0; 0; 50; 0; 0; 0];

m = 6; n = 1;
mu = 0.1;
sigma = 0.1;

%%
cvx_begin
    cvx_precision high
    variable F(n)
    variable fc_1(m,n)
    variable fc_2(m,n)
    variable fc_3(m,n)
    variable fc_4(m,n)
    minimize F
    subject to
        G*[fc_1;fc_2;fc_3;fc_4] + F_external == 0;
    
        fc1 = fc_1(1:2);
        fc2 = fc_2(1:2);
        fc3 = fc_3(1:2);
        fc4 = fc_4(1:2);
     
        norm(fc1) <= mu*fc_1(3);
        norm(fc2) <= mu*fc_2(3);
        norm(fc3) <= mu*fc_3(3);
        norm(fc4) <= mu*fc_4(3);
      
        fc_1(3) >= 0;
        fc_2(3) >= 0;
        fc_3(3) >= 0;
        fc_4(3) >= 0;
        
        norm(fc_1(6)) <= sigma*fc_1(3);
        norm(fc_2(6)) <= sigma*fc_2(3);
        norm(fc_3(6)) <= sigma*fc_3(3);
        norm(fc_4(6)) <= sigma*fc_4(3);
        
        fc_1(4) == 0;fc_2(4) == 0;fc_3(4) == 0;fc_4(4) == 0;
        fc_1(5) == 0;fc_2(5) == 0;fc_3(5) == 0;fc_4(5) == 0;
%         fc_1(6) == 0;fc_2(6) == 0;fc_3(6) == 0;fc_4(6) == 0;
        
        
        fc_1 <= F;
        fc_2 <= F;
        fc_3 <= F;
        fc_4 <= F;
        
cvx_end
