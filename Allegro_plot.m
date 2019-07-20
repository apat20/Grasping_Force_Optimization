close all;
clear;
clc;

% Getting the data for allegro including position of joints and finger and
% thumb joint angle limits in radian form.
% The position of the joints will serve as the arbitary point positions
% required in forward kinematics.

data = getData('Allegro.txt');
[m,n] = size(data{1});
C = {};

for i=1:m
% Each element of the cell 'data' is split and stores in another cell
% array of size m*n.
% Each row of 'C' contains the split string.
    C{i} = strsplit(data{1}{i}, ' ');
    assignin('base', C{i}{1}, str2num(C{i}{3}));
end

%% Forward Kinematics trial for Allegro one finger:
JT11_theta =  5;
JT13_theta = 25;
JT14_theta =  40;

omega_1= [0;1;0]; omega_3= [0;1;0]; omega_4 = [0;1;0];
omega_2 = [0;0;1];

I = eye(3,3);
P_base = [0;0;109];
gst_0 = [I, P_base;
        zeros(1,3),1];
    
exp_twist_theta_11 = GetExponential(omega_1, JT11_theta, JT11);
exp_twist_theta_13 = GetExponential(omega_1, JT13_theta, JT13);
exp_twist_theta_14 = GetExponential(omega_1, JT14_theta, JT14);

% Forward Kinematics for Allegro:
g_1 = exp_twist_theta_11*gst_0;
g_3 = exp_twist_theta_13*exp_twist_theta_11*gst_0;
g_4 = exp_twist_theta_14*exp_twist_theta_13*exp_twist_theta_11*gst_0;

% Transformed position of points:
P_11 = exp_twist_theta_11(1:3,4);
P_13 = exp_twist_theta_13 (1:3,4);
P_14 = exp_twist_theta_14(1:3,4);

%% Forward kinematics trial for second finger:
JT21_theta =  8;
JT23_theta = 12;
JT24_theta =  17;

omega_1= [0;1;0]; omega_3= [0;1;0]; omega_4 = [0;1;0];
omega_2 = [0;0;1];

I = eye(3,3);
P_base = [0;0;109];
gst_0 = [I, P_base;
        zeros(1,3),1];

exp_twist_theta_21 = GetExponential(omega_1, JT21_theta, JT21);
exp_twist_theta_23 = GetExponential(omega_1, JT23_theta, JT23);
exp_twist_theta_24 = GetExponential(omega_1, JT24_theta, JT24);

% Transformed position of points:
P_21 = exp_twist_theta_21(1:3,4);
P_23 = exp_twist_theta_23 (1:3,4);
P_24 = exp_twist_theta_24(1:3,4);

%% Forward kinematics for the third finger
JT31_theta =  18;
JT33_theta = 10;
JT34_theta =  8;

omega_1= [0;1;0]; omega_3= [0;1;0]; omega_4 = [0;1;0];
omega_2 = [0;0;1];

I = eye(3,3);
P_base = [0;0;109];
gst_0 = [I, P_base;
        zeros(1,3),1];

exp_twist_theta_31 = GetExponential(omega_1, JT31_theta, JT31);
exp_twist_theta_33 = GetExponential(omega_1, JT33_theta, JT33);
exp_twist_theta_34 = GetExponential(omega_1, JT34_theta, JT34);

% Transformed position of points:
P_31 = exp_twist_theta_31(1:3,4);
P_33 = exp_twist_theta_33 (1:3,4);
P_34 = exp_twist_theta_34(1:3,4);



%% Plotting Allegro hand 
origin = [0;0;0];

X_1 = [origin(1);JT11(1);JT12(1);JT13(1);JT14(1)];
Y_1 = [origin(2);JT11(2);JT12(2);JT13(2);JT14(2)];
Z_1 = [origin(3);JT11(3);JT12(3);JT13(3);JT14(3)];

% New position of the hand with repect to the old position:
X_new_1 = [origin(1);JT11(1) + P_11(1); JT13(1) + P_13(1);JT14(1) + P_14(1)];
Y_new_1 = [origin(2);JT11(2) + P_11(2);JT13(2) + P_13(2);JT14(2) + P_14(2)];
Z_new_1 = [origin(3);JT11(3) + P_11(3);JT13(3) + P_13(2);JT14(3) + P_14(3)];

X_new_2 = [origin(1);JT21(1) + P_21(1); JT23(1) + P_23(1);JT24(1) + P_24(1)];
Y_new_2 = [origin(2);JT21(2) + P_21(2);JT23(2) + P_23(2);JT24(2) + P_24(2)];
Z_new_2 = [origin(3);JT21(3) + P_21(3);JT23(3) + P_23(2);JT24(3) + P_24(3)];

X_new_3 = [origin(1);JT31(1) + P_31(1); JT33(1) + P_33(1);JT34(1) + P_34(1)];
Y_new_3 = [origin(2);JT31(2) + P_31(2);JT33(2) + P_33(2);JT34(2) + P_34(2)];
Z_new_3 = [origin(3);JT31(3) + P_31(3);JT33(3) + P_33(2);JT34(3) + P_34(3)];


% Old position of the hand 
X_2 = [origin(1);JT21(1);JT22(1);JT23(1);JT24(1)];
Y_2 = [origin(2);JT21(2);JT22(2);JT23(2);JT24(2)];
Z_2 = [origin(3);JT21(3);JT22(3);JT23(3);JT24(3)];

X_3 = [origin(1);JT31(1);JT32(1);JT33(1);JT34(1)];
Y_3 = [origin(2);JT31(2);JT32(2);JT33(2);JT34(2)];
Z_3 = [origin(3);JT31(3);JT32(3);JT33(3);JT34(3)];

X_4 = [origin(1);JT41(1);JT42(1);JT43(1);JT44(1)];
Y_4 = [origin(2);JT41(2);JT42(2);JT43(2);JT44(2)];
Z_4 = [origin(3);JT41(3);JT42(3);JT43(3);JT44(3)];

%% Generate a sphere

numFaces = 50;
[x,y,z] = sphere(numFaces);
[m,n] = size(x);
array_new = reshape([x*30-60,y*30+30,z*30+30],[(m)*(n),3]);
ptCloud = pointCloud(array_new);
% figure;
% pcshow(ptCloud);
% hold on
% pcwrite(ptCloud, 'sphere.pcd');
% title('Sphere with Default Color Map');
% xlabel('X');
% ylabel('Y');
% zlabel('Z');


% Generating the surface normals for matlab have the vector components.
normals = pcnormals(ptCloud);

% Plotting the normals on the surface of the generated the sphere.
x = ptCloud.Location(1:end, 1);
y = ptCloud.Location(1:end, 2);
z = ptCloud.Location(1:end, 3);
u = normals(1:end,1);
v = normals(1:end,2);
w = normals(1:end,3);

% quiver3(x,y,z,u,v,w);
% hold off

Center = [-60,30,30];
% Adjusting the normals which are not pointing in the same direction.
for k = 1 : numel(x)
   p1 = Center - [x(k),y(k),z(k)];
   p2 = [u(k),v(k),w(k)];
   % Flip the normal vector if it is not pointing away and outward from the
   % center
   angle = atan2(norm(cross(p1,p2)),p1*p2');
%  We change all the normals to pointing outwards, if we want them to point
%  inwards change the conditional to 'angle > pi/2'.
   if angle <= pi/2 || angle <= -pi/2
       u(k) = -u(k);
       v(k) = -v(k);
       w(k) = -w(k);
   elseif angle == pi/2
       R=[cosd(pi/2) -sind(pi/2); sind(pi/2) cosd(pi/2)];
       u(k) = u(k)*R';
       v(k) = v(k)*R';
       w(k) = w(k)*R';
   end
end


%% Plotting the fingers and the hand.
figure(1);
grid on;
% x = -100:5:100;
% y = -100:5:100;
% z = -100:5:100;
% [X,Y,Z] = meshgrid(x,y,z);
pbaspect([1 1 1]);
daspect([1 1 1]);
hold on;
scatter3(X_1,Y_1,Z_1);
hold on;
line(X_1,Y_1,Z_1);
hold on;
scatter3(X_new_1, Y_new_1, Z_new_1, 'r');
hold on;
line(X_new_1, Y_new_1, Z_new_1, 'Color','red','LineStyle','--');
hold on;
scatter3(X_new_2, Y_new_2, Z_new_2, 'r');
hold on;
line(X_new_2, Y_new_2, Z_new_2, 'Color','red','LineStyle','--');
hold on;
scatter3(X_2,Y_2,Z_2, 'b');
hold on;
line(X_2,Y_2,Z_2, 'Color','blue');
hold on;
scatter3(X_new_3, Y_new_3, Z_new_3, 'r');
hold on;
line(X_new_3, Y_new_3, Z_new_3, 'Color','red','LineStyle','--');
hold on;
scatter3(X_3,Y_3,Z_3);
hold on;
line(X_3,Y_3,Z_3);
hold on;
scatter3(X_4,Y_4,Z_4);
hold on;
line(X_4,Y_4,Z_4);
hold on;
quiver3(x,y,z,u,v,w);
hold off