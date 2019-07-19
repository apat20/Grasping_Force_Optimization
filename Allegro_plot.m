close all;
clear;
clc;

% Getting the data for allegro including position of joints and finger and
% thumb joint angle limits in radian form.

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


%% Plotting Allegro hand 
origin = [0;0;0];

X_1 = [origin(1);JT11(1);JT12(1);JT13(1);JT14(1)];
Y_1 = [origin(2);JT11(2);JT12(2);JT13(2);JT14(2)];
Z_1 = [origin(3);JT11(3);JT12(3);JT13(3);JT14(3)];

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
x = -100:5:100;
y = -100:5:100;
z = -100:5:100;
[X,Y,Z] = meshgrid(x,y,z);
hold on;
scatter3(X_1,Y_1,Z_1);
hold on;
line(X_1,Y_1,Z_1);
hold on;
scatter3(X_2,Y_2,Z_2);
hold on;
line(X_2,Y_2,Z_2);
hold on;
scatter3(X_3,Y_3,Z_3);
hold on;
line(X_3,Y_3,Z_3);
hold on;
scatter3(X_4,Y_4,Z_4);
hold on;
line(X_4,Y_4,Z_4);
hold off;






