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
title('Angle of tilt VS Squeezing force');
xlabel('Angle of tilt (degrees)');
ylabel('Magnitude of Squeezing force required (N)');
grid on;

% The net equivalent of the max moment due to f_E1, acting along the 'y'
% direction of the reference frame {E_1}, acting in the object frame and
% expressed in the object frame '{O}'.
Max_moment_E1 = [12.1267, 11.7300, 11.2006, 10.8493,  10.4271, 9.6439,  8.7167, 7.7651,  5.9172, 3.8119, 2.1949, 1.0591,  0.4128, -0.0181, -0.2755, -0.3356, -0.2364, -0.0797, 0.0000 ];

% The net equivalent of the max moment due to fc_1, acting along the 'Z'
% direction of the reference frame {C_1}, acting in the object frame and
% expressed in the object frame '{O}'.
% Max_moment_fc_1 = [];

figure;
plot(theta, Max_moment_E1, 'b');
title('Angle of tilt VS Moment')
xlabel('Angle of tilt (degrees)');
ylabel('Net equivalent moment due to f_{E1} expressed in {O}');
grid on;