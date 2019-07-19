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
JT13_theta = 10;
JT14_theta =  15;

omega_1= [0;1;0]; omega_3= [0;1;0]; omega_4 = [0;1;0];
omega_2 = [0;0;1];

I = eye(3,3);
P_base = [0;0;109];
gst_0 = [I, P_base;
        zeros(1,3),1];
    
exp_twist_theta_1 = GetExponential(omega_1, JT11_theta, JT21);
exp_twist_theta_3 = GetExponential(omega_1, JT13_theta, JT23);
exp_twist_theta_4 = GetExponential(omega_1, JT14_theta, JT24);

% Forward Kinematics for Allegro:
g_1 = exp_twist_theta_1*gst_0;
g_3 = exp_twist_theta_3*exp_twist_theta_1*gst_0;
g_4 = exp_twist_theta_4*exp_twist_theta_3*exp_twist_theta_1*gst_0;

% Transformed position of points:
P_1 = exp_twist_theta_1(1:3,4);
P_3 = exp_twist_theta_3 (1:3,4);
P_4 = exp_twist_theta_4(1:3,4);

%% Plotting Allegro hand 
origin = [0;0;0];

X_1 = [origin(1);JT11(1);JT12(1);JT13(1);JT14(1)];
Y_1 = [origin(2);JT11(2);JT12(2);JT13(2);JT14(2)];
Z_1 = [origin(3);JT11(3);JT12(3);JT13(3);JT14(3)];

% New position of the hand with repect to the old position:
X_new = [origin(1);JT21(1) + P_1(1); JT23(1) + P_3(1);JT24(1) + P_4(1)];
Y_new = [origin(2);JT21(2) + P_1(2);JT23(2) + P_3(2);JT24(2) + P_4(2)];
Z_new = [origin(3);JT21(3) + P_1(3);JT23(3) + P_3(2);JT24(3) + P_4(3)];

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
scatter3(X_new, Y_new, Z_new, 'r');
hold on;
line(X_new, Y_new, Z_new, 'Color','red','LineStyle','--');
hold on;
scatter3(X_2,Y_2,Z_2, 'b');
hold on;
line(X_2,Y_2,Z_2, 'Color','blue');
hold on;
scatter3(X_3,Y_3,Z_3);
hold on;
line(X_3,Y_3,Z_3);
hold on;
scatter3(X_4,Y_4,Z_4);
hold on;
line(X_4,Y_4,Z_4);
hold off;






