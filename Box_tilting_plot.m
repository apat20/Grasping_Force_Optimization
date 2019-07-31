clear all;
close all;
clc;

% Angle of tilt of the box
theta = [0,5,10,15,20,25,30,35,40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90];

% Weight of the object to be tilted
weight = 20;

% Maximum squeezing force required to grasp the box and apply the necessary
% force to tilt it.
F_c = [48.6714, 47.5643, 46.38, 45.0789, 43.2923, 41.3084, 40.0785, 40.5084, 43.1569, 44.7767, 44.8848, 43.4779, 40.91, 38.145, 34.4218, 28.9256, 21.203, 11.3446, 0 ];

figure;
plot(theta, F_c, 'b');
xlabel('Angle of tilt (degrees)');
ylabel('Magnitude of Squeezing force required (N)');
grid on;