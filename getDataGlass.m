
clear all;
close all;
clc;

% Point cloud of the glass of the required dimensions generated.
[point_cloud, array_new, array_normals, g_center, m] = getGlass(0.04, 0.05, 0.12, 0.001);

[points_1, points_2, normals_1, normals_2] = selectPoints(array_new, array_normals, m);

%% Getting different geometric centroid locations.
g_1 = frustumCentroid(0.09, 0.05, 0.1);
g_center_1 = [0,0,0.1-g_1];
g_2 = frustumCentroid(0.09, 0.05, 0.08);
g_center_2 = [0,0,0.08-g_2];
g_3 = frustumCentroid(0.09, 0.05, 0.06);
g_center_3 = [0,0,0.06-g_3];

% selected_points_2 = [0.0730         0    0.0792;
%                      0.0490         0    0.0216;
%                     -0.0730    0.0000    0.0792;
%                     -0.0490    0.0000    0.0216];
% 
% p_OC1_1 = selected_points_2(1,:) - g_center_3;
% p_OC2_1 = selected_points_2(2,:) - g_center_3;
% p_OC3_1 = selected_points_2(3,:) - g_center_3;
% p_OC4_1 = selected_points_2(4,:) - g_center_3;



%% Case I: 3 contact points on one side and 1 contact points on the other side
% The exact location of these points is arbitrary and can be changed
% according to the users wishes

selected_points_1 = [0.0770         0    0.0888;
                   0.0630         0    0.0552;
                   0.0490         0    0.0216;
                  -0.0640    0.0000    0.0576];
              
selected_normals_1 = [0.9231    0.0068   -0.3846;
                    0.9231    0.0068   -0.3846;
                    0.9231    0.0068   -0.3846;
                   -0.9231         0   -0.3846];
               
               
% Processing the components for plotting the normals using 'quiver3'
x_1 = selected_points_1(1:end,1);
y_1 = selected_points_1(1:end,2);
z_1 = selected_points_1(1:end,3);
u_1 = selected_normals_1(1:end,1);
v_1 = selected_normals_1(1:end,2);
w_1 = selected_normals_1(1:end,3);

figure;
pcshow(point_cloud);
hold on;
pcshow(g_center,'b');
hold on;
pcshow(g_center_1,'b');
hold on;
pcshow(g_center_2,'b');
hold on;
pcshow(g_center_3,'b');
hold on;
pcshow(selected_points_1,'[1,0,0]')
hold on;
axis square
title('Glass with Default Color Map');
xlabel('X');
ylabel('Y');
zlabel('Z');
hold on;
quiver3(x_1,y_1,z_1,-u_1,-v_1,-w_1);
hold off

% Getting the rotational matrix for the contact points using the
% orientation of the normal. One of the tangential components is calculated
% as n_i*t_i = 0, where we assume value for one of the x,y and z components of
% t_i and solve for the other two.
% The other tangential component is calculated as o_i = cross(n_i, t_i)
matrix_11 = getOrthogonal(selected_normals_1(1,:));
matrix_21 = getOrthogonal(selected_normals_1(4,:));

% Rotation matrix for the contact points on one side. We assume that the
% contact point C_1 is on this side. S
% Since the components of the surface normals on one side are similar, the
% rotation matrices for points on that side are also similar.
R_OC1_1 = [matrix_11,selected_normals_1(1,:)'];

% Rotation matrix for the contact points on the other side. We assume that
% the contact point C_4 is on this side.
r3_1 = selected_normals_1(4,:)';
r1_1 = matrix_21(:,2);
r2_1 = cross(r3_1,r1_1);
R_OC4_1 = [r1_1,r2_1,r3_1];

%Calculating the position vectors:
p_OC1_1 = selected_points_1(1,:) - g_center;
p_OC2_1 = selected_points_1(2,:) - g_center;
p_OC3_1 = selected_points_1(3,:) - g_center;
p_OC4_1 = selected_points_1(4,:) - g_center;

%% Case I: 2 contact points on one side and 2 contact points on the other side
% The exact location of these points is arbitrary and can be changed
% according to the users wishes


selected_points_2 = [0.0730         0    0.0792;
                     0.0490         0    0.0216;
                    -0.0730    0.0000    0.0792;
                    -0.0490    0.0000    0.0216];

selected_normals_2 = [0.9231    0.0068   -0.3846;
                      0.9231    0.0068   -0.3846;
                     -0.9231         0   -0.3846;
                     -0.9231         0   -0.3846];
               
% Processing the components for plotting the normals using 'quiver3'
x_2 = selected_points_2(1:end,1);
y_2 = selected_points_2(1:end,2);
z_2 = selected_points_2(1:end,3);
u_2 = selected_normals_2(1:end,1);
v_2 = selected_normals_2(1:end,2);
w_2 = selected_normals_2(1:end,3);

figure;
pcshow(point_cloud);
hold on;
pcshow(g_center,'b');
hold on;
pcshow(g_center_1,'b');
hold on;
pcshow(g_center_2,'b');
hold on;
pcshow(g_center_3,'b');
hold on;
pcshow(selected_points_2,'[1,0,0]')
hold on;
axis square
title('Glass with Default Color Map');
xlabel('X');
ylabel('Y');
zlabel('Z');
hold on;
quiver3(x_2,y_2,z_2,-u_2,-v_2,-w_2);
hold off

% Getting the rotational matrix for the contact points using the
% orientation of the normal. One of the tangential components is calculated
% as n_i*t_i = 0, where we assume value for one of the x,y and z components of
% t_i and solve for the other two.
% The other tangential component is calculated as o_i = cross(n_i, t_i)
matrix_12 = getOrthogonal(selected_normals_2(1,:));
matrix_22 = getOrthogonal(selected_normals_2(4,:));

% Rotation matrix for the contact points on one side. We assume that the
% contact point C_1 is on this side. S
% Since the components of the surface normals on one side are similar, the
% rotation matrices for points on that side are also similar.
R_OC1_2 = [matrix_12,selected_normals_2(1,:)'];

% Rotation matrix for the contact points on the other side. We assume that
% the contact point C_4 is on this side.
r3_2 = selected_normals_2(4,:)';
r1_2 = matrix_22(:,2);
r2_2 = cross(r3_2,r1_2);
R_OC4_2 = [r1_2,r2_2,r3_2];

%Calculating the position vectors:
p_OC1_2 = selected_points_2(1,:) - g_center;
p_OC2_2 = selected_points_2(2,:) - g_center;
p_OC3_2 = selected_points_2(3,:) - g_center;
p_OC4_2 = selected_points_2(4,:) - g_center;



