%%
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
% surf(X,Y,Z);

% Displaying the normals on the cone using surfnorm
% surfnorm(X,Y,Z);

% Getting the components of the normal at a particular point
[Nx,Ny,Nz] = surfnorm(X,Y,Z);
array_new = reshape([X,Y,Z],[m*n,3]);
array_normals = reshape([Nx,Ny,Nz],[m*n,3]);

% Converting the reshaped points forming the surface of our cylinder into a
% point cloud.
ptCloud = pointCloud(array_new);
% Finding the geometric mean of our point cloud:

C = mean(array_new);
C_1 = [0, 0, 15-5.1923];
C_2 = [0, 0, 15-4.1538];
C_3 = [0, 0, 15-3.1154];
C_4 = [0, 0, 15-2.0769];
center = pointCloud(C);
% center_new = pointCloud(C_new);

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
pcshow(C_1,'b');
hold on;
pcshow(C_2,'b');
hold on;
pcshow(C_3,'b');
hold on;
pcshow(C_4,'b');
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
P_OC1 = selected_points(1,:) - C;
P_OC2 = selected_points(2,:) - C;
P_OC3 = selected_points(3,:) - C;
P_OC4 = selected_points(4,:) - C;


%%
g1 = frustumCentroid(6,2,15);
g2 = frustumCentroid(6,2,12);
g3 = frustumCentroid(6,2,9);
g4 = frustumCentroid(6,2,6);
%%
close all;
clear all;
clc;

[X,Y,Z] = cylinder(1,10);
% Z(2,:) = Z(2,:) * 10;
figure(1);
surf(X, Y, Z);
grid on;
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');
%%
r1 = [-0.9285; 0.0069; -0.3714];
r2 = [-0.0069;1.0000;0.0013];
r3 = [0.3714;-0.0013;-0.9285];

R = [r1,r2,r3];
