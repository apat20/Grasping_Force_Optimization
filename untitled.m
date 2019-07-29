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
center = pointCloud(C);

% The selected points for grasping
selected_points = [ 5.9000         0    4.7500;
                  7.4000         0    8.5000;
                  8.9000         0   12.2500];
                 
% Components of the selected normals:

selected_normals = [0.9285    0.0069   -0.3714;
                    0.9285    0.0069   -0.3714;
                    0.9285    0.0069   -0.3714];

figure;
pcshow(ptCloud);
hold on;
pcshow(C,'r');
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


%% 
points = [4.0000         0         0
    4.1000         0    0.2500
    4.2000         0    0.5000
    4.3000         0    0.7500
    4.4000         0    1.0000
    4.5000         0    1.2500
    4.6000         0    1.5000
    4.7000         0    1.7500
    4.8000         0    2.0000
    4.9000         0    2.2500
    5.0000         0    2.5000
    5.1000         0    2.7500
    5.2000         0    3.0000
    5.3000         0    3.2500
    5.4000         0    3.5000
    5.5000         0    3.7500
    5.6000         0    4.0000
    5.7000         0    4.2500
    5.8000         0    4.5000
    5.9000         0    4.7500
    6.0000         0    5.0000
    6.1000         0    5.2500
    6.2000         0    5.5000
    6.3000         0    5.7500
    6.4000         0    6.0000
    6.5000         0    6.2500
    6.6000         0    6.5000
    6.7000         0    6.7500
    6.8000         0    7.0000
    6.9000         0    7.2500
    7.0000         0    7.5000
    7.1000         0    7.7500
    7.2000         0    8.0000
    7.3000         0    8.2500
    7.4000         0    8.5000
    7.5000         0    8.7500
    7.6000         0    9.0000
    7.7000         0    9.2500
    7.8000         0    9.5000
    7.9000         0    9.7500
    8.0000         0   10.0000
    8.1000         0   10.2500
    8.2000         0   10.5000
    8.3000         0   10.7500
    8.4000         0   11.0000
    8.5000         0   11.2500
    8.6000         0   11.5000
    8.7000         0   11.7500
    8.8000         0   12.0000
    8.9000         0   12.2500
    9.0000         0   12.5000
    9.1000         0   12.7500
    9.2000         0   13.0000
    9.3000         0   13.2500
    9.4000         0   13.5000
    9.5000         0   13.7500
    9.6000         0   14.0000
    9.7000         0   14.2500
    9.8000         0   14.5000
    9.9000         0   14.7500
   10.0000         0   15.0000];

for i = 20:15:51
    disp(i)
    disp(array_normals(i,:));
end


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
r1 = [0.9285;0.0069;-0.3714];
r2 = [-0.0069;1.0000;0.0013];
r3 = [0.3714;0.0013;0.9285];

R = [r1,r2,r3];
