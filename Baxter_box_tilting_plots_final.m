clear all;
close all;
clc;

% Angle of tilt of the box
theta_complete = [0,5,10,15,20,25,30,35,40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90];
theta_experimental = [0,5,10,15,20,25,30,35,40, 45, 50, 55];

Fc_opt_experimental = [12.0252,11.0919,9.82204,8.25166, 6.42574,4.3968,2.22349,0.0309831,2.30062,4.51845,6.6184,8.53718];
Fc_opt_complete = [12.0252,11.0919,9.82204,8.25166, 6.42574,4.3968,2.22349,0.0309831,2.30062,4.51845,6.6184,8.53718,10.2161,11.6073,12.6546,13.331,13.6124,13.4838,12.9433];

Fc_1_tau_opt = [12.5364, 11.8350, 10.4790, 8.6500, 6.7710,4.7136,2.3549,0.0357,2.6697,5.1724,7.4648, 9.9531];
Fc_2_tau_opt = [12.7581, 12.0060, 10.4954, 8.4732,  6.2535,4.1845,2.1331,0.0266,1.9670,3.9269,5.9284, 8.2950];

F_e_experimental = [8.8334,8.9734,9.1535,9.3562,9.5616,9.7494,9.9010,10.0010,10.0390,10.0106,9.9182,9.7705];
F_e_complete = [8.8334,8.9734,9.1535,9.3562,9.5616,9.7494,9.9010,10.0010,10.0390,10.0106,9.9182,9.7705,9.5821,9.3063,9.1230,8.9752,8.8317,8.7487,8.7375];

F_e1_tau_opt = [8.8503, 8.9842, 9.1071, 9.1918, 9.0877,9.0733,9.5886,9.9904,9.1895,8.5115,8.1431,8.0078];
F_e2_tau_opt = [9.5708, 10.0206, 10.1688, 10.1357, 10.3212,10.3076,10.1149, 10.0101,10.7562,11.2596, 11.2673,10.8219];

Tau_left = [5.8621, 5.3674, 4.6976, 3.8772, 2.9470, 1.9568, 0.9870,0.0186,1.3807,2.7019,3.9251,4.9883];
Tau_right = [ 4.1734,3.5426, 3.3387,  2.8225, 2.4618, 1.7801,0.9161,0.0140,1.0300,2.0641,3.0747,3.3391];

Tau_opt = [5.62443, 5.11384, 4.42143, 3.61079, 2.70212,1.84277,0.944529,0.015876,1.17067,2.32451,3.43288,4.43672];

Tau_left_opt = [5.6244, 5.1138, 4.4214,  3.6108, 2.7021,1.8428,0.9445, 0.0159,1.1707,2.3245,3.4329, 4.4367];
Tau_right_opt = [4.3508, 3.7799, 3.5620, 2.9587, 2.5940,1.8428,0.9445, 0.0159,1.1707,2.3245,3.4329, 3.8881];

diff_1 = [0.5917,0.4773,0.4172, 0.4195,  0.4864, 0.6137, 0.7903, 0.9970,  0.7771, 0.5493, 0.3351, 0.1548, 0.0260,0,0,0.0449,0.1846, 0.3726, 0.5887];
diff_2 = [0.5917,0.4773,0.4172, 0.4195,  0.4864, 0.6137, 0.7903, 0.9970, 0.7771, 0.5493, 0.3351, 0.1548, 0.0260,0,0, 0.0449, 0.1846, 0.3726,0.5887];

%%

figure;
plot(theta_experimental, Fc_opt_experimental, 'b');
title('Angle of tilt VS Squeezing force (Experimental)');
xlabel('Angle of tilt (degrees)');
ylabel('Magnitude of Squeezing force required (N)');
grid on;

figure;
plot(theta_complete, Fc_opt_complete, 'b');
title('Angle of tilt VS Squeezing force (Complete)');
xlabel('Angle of tilt (degrees)');
ylabel('Magnitude of Squeezing force required (N)');
grid on;

%%

figure;
plot(theta_complete, F_e_complete, 'b');
title('Angle of tilt VS Reaction forces (Complete)');
xlabel('Angle of tilt (degrees)');
ylabel('Magnitude of Reaction force generated (N)');
grid on;

figure;
plot(theta_experimental, F_e_experimental, 'b');
title('Angle of tilt VS Reaction Forces (Experimental)');
xlabel('Angle of tilt (degrees)');
ylabel('Magnitude of Reaction force generated (N)');
grid on;

%%

figure;
plot(theta_experimental, Tau_left, 'r');
hold on;
plot(theta_experimental, Tau_left_opt, 'b');
hold off
title('Angle of tilt VS Torques (Experimental)');
xlabel('Angle of tilt (degrees)');
ylabel('Maximum value of Torque generated in the left arm.(N/m)');
grid on;

%%

figure;
plot(theta_complete, diff_1, 'b');
title('Satisfaction of the Friction cone constraints');
xlabel('Angle of tilt (degrees)');
ylabel('Difference between mu times normal force and magnitude of tangential forces.');
grid on;






