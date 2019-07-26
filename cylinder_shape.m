close all;
clear all;
clc;


t = 0:pi/20:2*pi;
[X,Y,Z] = cylinder(2+t);
[m,n] = size(X);
array_new = reshape([X,Y,Z],[(m)*(n),3]);
ptCloud = pointCloud(array_new);

C = mean(array_new);
center = pointCloud(C);

points = [0   -3.7279    0.2750;
         0   -5.4558    0.5500;
          0   -7.1836    0.8250;
          0    5.6128    0.5750;
          0,0,0];
     

figure;
pcshow(ptCloud);
hold on;
pcshow(C,'r');
hold on;
pcshow(points,'[1,0,0]')
hold on;
axis square
title('Glass with Default Color Map');
xlabel('X');
ylabel('Y');
zlabel('Z');
hold off;

% Calculating the position vectors:
P1 = points(1,1:3);
P2 = points(2,1:3);
P3 = points(3,1:3);
P4 = points(4,1:3);

P_OC1 = P1 - C;
P_OC2 = P2 - C;
P_OC3 = P3 - C;
P_OC4 = P4 - C;





