clear all;
close all;
clc;

% Angle of tilt of the box
theta_complete = [0,5,10,15,20,25,30,35,40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90];
theta_experimental = [0,5,10,15,20,25,30,35,40, 45, 50, 55];
name_left = ["left_s0", 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2'];
name_right = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2'];

% Fc_opt_experimental = [12.0252,11.0919,9.82204,8.25166, 6.42574,4.3968,2.22349,0.0309831,2.30062,4.51845,6.6184,8.53718];
% Fc_opt_complete = [12.0252,11.0919,9.82204,8.25166, 6.42574,4.3968,2.22349,0.0309831,2.30062,4.51845,6.6184,8.53718,10.2161,11.6073,12.6546,13.331,13.6124,13.4838,12.9433];
% 
% Fc_1_tau_opt = [12.5364, 11.8350, 10.4790, 8.6500, 6.7710,4.7136,2.3549,0.0357,2.6697,5.1724,7.4648, 9.9531];
% Fc_2_tau_opt = [12.7581, 12.0060, 10.4954, 8.4732,  6.2535,4.1845,2.1331,0.0266,1.9670,3.9269,5.9284, 8.2950];
% 
% F_e_experimental = [8.8334,8.9734,9.1535,9.3562,9.5616,9.7494,9.9010,10.0010,10.0390,10.0106,9.9182,9.7705];
% F_e_complete = [8.8334,8.9734,9.1535,9.3562,9.5616,9.7494,9.9010,10.0010,10.0390,10.0106,9.9182,9.7705,9.5821,9.3063,9.1230,8.9752,8.8317,8.7487,8.7375];
% 
% F_e1_tau_opt = [8.8503, 8.9842, 9.1071, 9.1918, 9.0877,9.0733,9.5886,9.9904,9.1895,8.5115,8.1431,8.0078];
% F_e2_tau_opt = [9.5708, 10.0206, 10.1688, 10.1357, 10.3212,10.3076,10.1149, 10.0101,10.7562,11.2596, 11.2673,10.8219];
% 
% Tau_left = [5.8621, 5.3674, 4.6976, 3.8772, 2.9470, 1.9568, 0.9870,0.0186,1.3807,2.7019,3.9251,4.9883];
% Tau_right = [ 4.1734,3.5426, 3.3387,  2.8225, 2.4618, 1.7801,0.9161,0.0140,1.0300,2.0641,3.0747,3.3391];
% 
% Tau_opt = [5.62443, 5.11384, 4.42143, 3.61079, 2.70212,1.84277,0.944529,0.015876,1.17067,2.32451,3.43288,4.43672];
% 
% Tau_left_opt = [5.6244, 5.1138, 4.4214,  3.6108, 2.7021,1.8428,0.9445, 0.0159,1.1707,2.3245,3.4329, 4.4367];
% Tau_right_opt = [4.3508, 3.7799, 3.5620, 2.9587, 2.5940,1.8428,0.9445, 0.0159,1.1707,2.3245,3.4329, 3.8881];
% 
% diff_1 = [0.5917,0.4773,0.4172, 0.4195,  0.4864, 0.6137, 0.7903, 0.9970,  0.7771, 0.5493, 0.3351, 0.1548, 0.0260,0,0,0.0449,0.1846, 0.3726, 0.5887];
% diff_2 = [0.5917,0.4773,0.4172, 0.4195,  0.4864, 0.6137, 0.7903, 0.9970, 0.7771, 0.5493, 0.3351, 0.1548, 0.0260,0,0, 0.0449, 0.1846, 0.3726,0.5887];
%%
%%%%%%%%%%%%%%% BOX TILTING ANISOTROPIC FRICTION %%%%%%%%%%%%%%%%%%%%%%%%%%
Fc_opt_experimental = [27.6831, 25.5176, 22.5777,18.9496, 14.7399, 10.0728, 5.08647, 0.0707619, 5.24489, 10.2805, 15.0255, 19.3355];
Fc_opt_complete = [23.0781, 26.1372, 28.4163, 29.8421, 30.3662, 29.9677, 28.6535];

F_e_experimental = [8.9782, 9.1162,9.2884, 9.4761, 9.6591, 9.8178, 9.9356, 10.0005, 10.0068, 9.9555, 9.8539, 9.7154];
F_e_complete = [9.5572, 9.3987, 9.2592, 9.1554, 9.0996, 9.0984, 9.1516];

Tau_left = [11.9286, 10.8776, 9.8969, 8.6043,  6.9255,4.8888, 2.4926, 0.0391, 2.8949, 5.6461, 8.1737, 10.3488];
joint_max_torque_left = [4,4, 3, 3, 3, 3, 3, 3, 3,3, 3];

Tau_right = [14.2186,  13.1890, 10.3841, 10.1341, 7.1048, 4.9078, 2.5101, 0.0317, 2.4557, 4.7566, 7.1341, 8.8899];
joint_max_torque_right = [3,3, 3, 3, 2, 2, 3, 4, 4,3, 3];

diff_1 = [0.5913,0.4896, 0.4423,  0.4553,  0.5280, 0.6530, 0.8168, 0.9975,  0.8116, 0.6326, 0.4797, 0.3691];
diff_complete = [0.3119 , 0.3138, 0.3725, 0.4801, 0.6229, 0.7834, 0.8873];

net_moment_due_reaction = [-2.6598, -2.4515, -2.1688, -1.8200, -1.415, -0.4883, 0.0068, 0.5033, 0.9862, 1.4410, 1.8539,2.2122];
net_moment_due_contact = [ 2.6598,];

Tau_opt = [13.9528, 12.9549, 10.1129, 9.86272, 6.99321, 4.88463,2.49459, 0.0349038, 2.6534, 5.15573, 7.67201, 9.89568  ];

Fc_1_tau_opt = [27.4924, 25.3251, 22.2700, 18.6243, 14.6118,10.0937 , 5.0880, 0.0787, 5.6780, 11.1644, 16.0469,  20.3641];
Fc_2_tau_opt = [28.3272,  26.1057, 23.1659, 19.5903,14.9545, 10.1069, 5.1121, 0.0634, 4.8282, 9.4292, 14.3015, 18.7794];
F_e1_tau_opt = [9.6769, 9.8665, 10.0240, 10.4463, 10.0516, 9.9066, 9.9940, 9.9865, 9.1967, 8.3384, 8.2894, 8.3996];
F_e2_tau_opt = [8.7081, 8.7463, 8.6385, 8.7832, 9.2945, 9.7540, 9.8882, 10.0149, 10.8063, 11.5607, 11.3402, 10.9444];

Tau_left_opt = [12.1984, 11.1283, 10.1129, 8.8953, 6.9932,4.8846, 2.4946,  0.0349,  2.6534, 5.1557, 7.6720, 9.8957];
joint_max_torque_left_opt = [4,4,3];
Tau_right_opt = [13.9528, 12.9549, 10.1129,  9.8627, 6.9932, 4.8846, 2.4946, 0.0349, 2.6534, 5.1557, 7.6720, 9.3597];
joint_max_torque_right_opt = [3, 3, 3];

Tau_opt_diff_e1 = [0,0,0,0,0.3708,0.6246,0.8321,0.9893,0.5034,0.0656,0,0];
Tau_opt_diff_e2 = [0,0.0383,0.0383,0,0.6000,0.6201,0.8002,0.9938,0.5740,0.0606,0,0];


%% 
for i = 1:numel(Tau_left)
    Tau_max(i) = max(Tau_left(i), Tau_right(i));
end



%%

figure('Units', 'pixels', ...
    'Position', [100 100 500 375]);
h = plot(theta_experimental, Fc_opt_experimental, 'b');
ax = h.Parent;

Title = title('Angle of tilt VS Squeezing force');
X_label = xlabel('Angle of tilt (degrees)');
Y_label = ylabel('Squeezing force (N)');
grid on;

xtl = ax.XTickLabel;
xtld = strcat(xtl,'^{\circ}');
ax.XTickLabel = xtld;

set( gca                       , ...
    'FontName'   , 'Helvetica' );
set([Title, X_label, Y_label], ...
    'FontName'   , 'AvantGarde');
set(gca             , ...
    'FontSize'   , 8           );
set([X_label, Y_label]  , ...
    'FontSize'   , 10          );
set( Title                    , ...
    'FontSize'   , 12          , ...
    'FontWeight' , 'bold'      );

set(gcf, 'PaperPositionMode', 'auto');
% print -depsc2 finalPlot1.eps
% close;


% figure;
% plot(theta_complete, Fc_opt_complete, 'b');
% title('Angle of tilt VS Squeezing force (Complete)');
% xlabel('Angle of tilt (degrees)');
% ylabel('Magnitude of Squeezing force required (N)');
% grid on;

%%

% figure;
% plot(theta_complete, F_e_complete, 'b');
% title('Angle of tilt VS Reaction forces (Complete)');
% xlabel('Angle of tilt (degrees)');
% ylabel('Magnitude of Reaction force generated (N)');
% grid on;

figure('Units', 'pixels', ...
    'Position', [100 100 500 375]);
h = plot(theta_experimental, F_e_experimental, 'b');
ax = h.Parent;

Title = title('Angle of tilt VS Reaction Forces');
X_label = xlabel('Angle of tilt (degrees)');
Y_label = ylabel('Reaction force generated (N)');
grid on;

xtl = ax.XTickLabel;
xtld = strcat(xtl,'^{\circ}');
ax.XTickLabel = xtld;

set( gca                       , ...
    'FontName'   , 'Helvetica' );
set([Title, X_label, Y_label], ...
    'FontName'   , 'AvantGarde');
set(gca             , ...
    'FontSize'   , 8           );
set([X_label, Y_label]  , ...
    'FontSize'   , 10          );
set( Title                    , ...
    'FontSize'   , 12          , ...
    'FontWeight' , 'bold'      );

 set(gcf, 'PaperPositionMode', 'auto');
% print -depsc2 finalPlot2.eps
% close;


%%

figure;
plot(theta_experimental, Tau_left, 'r');
hold on;
plot(theta_experimental, Tau_left_opt, 'b');
hold off
Title = title('Angle of tilt VS Torques (Experimental)');
X_label = xlabel('Angle of tilt (degrees)');
Y_label = ylabel('Maximum value of Torque generated in the left arm.(N/m)');
grid on;


%%

% figure;
% plot(theta_experimental, diff_1, 'b');
% title('Satisfaction of the Friction cone constraints');
% xlabel('Angle of tilt (degrees)');
% 
% grid on;

figure('Units', 'pixels', ...
    'Position', [100 100 500 375]);
hold on;
h = plot(theta_experimental, Tau_opt_diff_e1, 'b');
ax = h.Parent;
e = plot(theta_experimental, Tau_opt_diff_e2, 'r');
Title = title('Friction cone constraints satisfaction');
X_label = xlabel('Angle of tilt (degrees)');
Y_label = ylabel('Satisfaction criteria (N).');

grid on;

xtl = ax.XTickLabel;
xtld = strcat(xtl,'^{\circ}');
ax.XTickLabel = xtld;

set( gca                       , ...
    'FontName'   , 'Helvetica' );
set([Title, X_label, Y_label], ...
    'FontName'   , 'AvantGarde');
set(gca             , ...
    'FontSize'   , 8           );
set([X_label, Y_label]  , ...
    'FontSize'   , 10          );
set( Title                    , ...
    'FontSize'   , 12          , ...
    'FontWeight' , 'bold'      );

 set(gcf,'PaperPositionMode', 'auto');
% print -depsc2 finalPlot3.eps
% close;

%% 

figure('Units', 'pixels', ...
    'Position', [100 100 500 375]);
hold on;
h = plot(theta_experimental, Tau_left, 'b');
ax = h.Parent;
e = plot(theta_experimental, Tau_right, 'r');
hLegend = legend('Torque in left arm','Torque in right arm','location', 'NorthEast' );
Title = title('Joint Torque variation in both arms');
X_label = xlabel('Angle of tilt (degrees)');
Y_label = ylabel('Torque required (N).');
grid on;

xtl = ax.XTickLabel;
xtld = strcat(xtl,'^{\circ}');
ax.XTickLabel = xtld;


set(gca,'Ytick',0:2:18);
set( gca                       , ...
    'FontName'   , 'Helvetica' );
set([Title, X_label, Y_label], ...
    'FontName'   , 'AvantGarde');
set([hLegend, gca]             , ...
    'FontSize'   , 8           );
set([X_label, Y_label]  , ...
    'FontSize'   , 10          );
set( Title                    , ...
    'FontSize'   , 12          , ...
    'FontWeight' , 'bold'      );

set(gcf,'PaperPositionMode', 'auto');
% print -depsc2 TorquePlot1.eps
% close;


%% Torque optimization graphs.

figure('Units', 'pixels', ...
    'Position', [100 100 500 375]);
hold on;
h = plot(theta_experimental, F_e1_tau_opt, 'r');
ax = h.Parent;
e = plot(theta_experimental, F_e_experimental, '--b');
hLegend = legend('Torque opt.','Force opt.','location', 'NorthEast' );
Title = title('The variation in the Reaction force at E1');
X_label = xlabel('Angle of tilt (degrees)');
Y_label = ylabel('Reaction force at E1 (N).');
grid on;

xtl = ax.XTickLabel;
xtld = strcat(xtl,'^{\circ}');
ax.XTickLabel = xtld;

set( gca                       , ...
    'FontName'   , 'Helvetica' );
set([Title, X_label, Y_label], ...
    'FontName'   , 'AvantGarde');
set([hLegend, gca]             , ...
    'FontSize'   , 8           );
set([X_label, Y_label]  , ...
    'FontSize'   , 10          );
set( Title                    , ...
    'FontSize'   , 12          , ...
    'FontWeight' , 'bold'      );

set(gcf,'PaperPositionMode', 'auto');
% print -depsc2 fe1_T_opt.eps
% close;

%%

figure('Units', 'pixels', ...
    'Position', [100 100 500 375]);
hold on;
h = plot(theta_experimental, F_e2_tau_opt, 'r');
ax = h.Parent;
e = plot(theta_experimental, F_e_experimental, '--b');
hLegend = legend('Torque opt.','Force opt.','location', 'NorthEast' );
Title = title('The variation in the Reaction force at E2');
X_label = xlabel('Angle of tilt (degrees)');
Y_label = ylabel('Reaction force at E2 (N).');
grid on;

xtl = ax.XTickLabel;
xtld = strcat(xtl,'^{\circ}');
ax.XTickLabel = xtld;

set( gca                       , ...
    'FontName'   , 'Helvetica' );
set([Title, X_label, Y_label], ...
    'FontName'   , 'AvantGarde');
set([hLegend, gca]             , ...
    'FontSize'   , 8           );
set([X_label, Y_label]  , ...
    'FontSize'   , 10          );
set( Title                    , ...
    'FontSize'   , 12          , ...
    'FontWeight' , 'bold'      );

set(gcf,'PaperPositionMode', 'auto');
% print -depsc2 fe2_T_opt.eps
% close;

%%
figure('Units', 'pixels', ...
    'Position', [100 100 500 375]);
hold on;
h = plot(theta_experimental, Tau_left_opt, 'r');
ax = h.Parent;
e = plot(theta_experimental, Tau_left, '--b');
hLegend = legend('Torque opt.','Force opt.','location', 'NorthEast' );
Title = title('The variation in the max. Joint Torque in left arm');
X_label = xlabel('Angle of tilt (degrees)');
Y_label = ylabel('Joint Torque magnitude (N/m)');
grid on;

xtl = ax.XTickLabel;
xtld = strcat(xtl,'^{\circ}');
ax.XTickLabel = xtld;

set( gca                       , ...
    'FontName'   , 'Helvetica' );
set([Title, X_label, Y_label], ...
    'FontName'   , 'AvantGarde');
set([hLegend, gca]             , ...
    'FontSize'   , 8           );
set([X_label, Y_label]  , ...
    'FontSize'   , 10          );
set( Title                    , ...
    'FontSize'   , 12          , ...
    'FontWeight' , 'bold'      );

set(gcf,'PaperPositionMode', 'auto');
% print -depsc2 left_T_opt.eps
% close;

figure('Units', 'pixels', ...
    'Position', [100 100 500 375]);
hold on;
h = plot(theta_experimental, Tau_right_opt, 'r');
ax = h.Parent;
e = plot(theta_experimental, Tau_right, '--b');
hLegend = legend('Torque opt.','Force opt.','location', 'NorthEast' );
Title = title('The variation in the max. Joint Torque in right arm');
X_label = xlabel('Angle of tilt (degrees)');
Y_label = ylabel('Joint Torque magnitude (N/m)');
grid on;

xtl = ax.XTickLabel;
xtld = strcat(xtl,'^{\circ}');
ax.XTickLabel = xtld;

set( gca                       , ...
    'FontName'   , 'Helvetica' );
set([Title, X_label, Y_label], ...
    'FontName'   , 'AvantGarde');
set([hLegend, gca]             , ...
    'FontSize'   , 8           );
set([X_label, Y_label]  , ...
    'FontSize'   , 10          );
set( Title                    , ...
    'FontSize'   , 12          , ...
    'FontWeight' , 'bold'      );

set(gcf,'PaperPositionMode', 'auto');
% print -depsc2 right_T_opt.eps

%%
figure('Units', 'pixels', ...
    'Position', [100 100 500 375]);
hold on;
h = plot(theta_experimental, Tau_left_opt, 'b');
ax = h.Parent;
e = plot(theta_experimental, Tau_right_opt, 'r');
hLegend = legend('Torque in left arm','Torque in right arm','location', 'NorthEast' );
Title = title('Joint Torque variation in both arms');
X_label = xlabel('Angle of tilt (degrees)');
Y_label = ylabel('Torque required (N/m).');
grid on;
h = plot(theta_experimental, Fc_opt_experimental, 'b');
xtl = ax.XTickLabel;
xtld = strcat(xtl,'^{\circ}');
ax.XTickLabel = xtld;


set(gca,'Ytick',0:2:18);
set( gca                       , ...
    'FontName'   , 'Helvetica' );
set([Title, X_label, Y_label], ...
    'FontName'   , 'AvantGarde');
set([hLegend, gca]             , ...
    'FontSize'   , 8           );
set([X_label, Y_label]  , ...
    'FontSize'   , 10          );
set( Title                    , ...
    'FontSize'   , 12          , ...
    'FontWeight' , 'bold'      );

set(gcf,'PaperPositionMode', 'auto');
print -depsc2 Torque_opt_Plot.eps
% close;

%% 

Tau_left = [-1.1572;0.3838;-9.8969;9.4764;-4.4674;6.5023;-0.0406];
Tau_left_opt = [-1.1989;0.4262;-10.1129;9.7529;-4.5728;6.6829;-0.0422];

Tau_right = [1.3539;1.1774;-10.3841;8.6097;-6.2281;-3.1073;-0.0369];
Tau_right_opt = [1.2394;1.2822;-10.1129;8.5849;-6.1252;-3.1046;-0.0378];

% plot(y)
% set(gca,'xticklabel',x.')

figure('Units', 'pixels', ...
    'Position', [100 100 500 375]);
hold on;
h = plot(Tau_right_opt, 'b');
set(gca, 'xticklabel',name_left.')
ax = h.Parent;
e = plot(Tau_right, 'r');
hLegend = legend('Torque in left arm','Torque in right arm','location', 'NorthEast' );
Title = title('Joint Torque variation in both arms');
X_label = xlabel('Angle of tilt (degrees)');
Y_label = ylabel('Torque required (N/m).');
grid on;

% xtl = ax.XTickLabel;
% xtld = strcat(xtl,'^{\circ}');
% ax.XTickLabel = xtld;


% set(gca,'Ytick',0:2:18);
set( gca                       , ...
    'FontName'   , 'Helvetica' );
set([Title, X_label, Y_label], ...
    'FontName'   , 'AvantGarde');
set([hLegend, gca]             , ...
    'FontSize'   , 8           );
set([X_label, Y_label]  , ...
    'FontSize'   , 10          );
set( Title                    , ...
    'FontSize'   , 12          , ...
    'FontWeight' , 'bold'      );

set(gcf,'PaperPositionMode', 'auto');
% print -depsc2 Torque_opt_Plot.eps
% close;
%%
figure('Units', 'pixels', ...
    'Position', [100 100 500 375]);
hold on;
h = plot(theta_experimental, Tau_opt, 'b');
ax = h.Parent;
e = plot(theta_experimental, Tau_max, '--r');

Title = title('Torque Minimization');
X_label = xlabel('Angle of tilt (degrees)');
Y_label = ylabel('Maximum Torque (Nm)');
hLegend = legend('Torque opt.','Force opt.','location', 'NorthEast' );
grid on;

xtl = ax.XTickLabel;
xtld = strcat(xtl,'^{\circ}');
ax.XTickLabel = xtld;

set( gca                       , ...
    'FontName'   , 'Helvetica' );
set([Title, X_label, Y_label], ...
    'FontName'   , 'AvantGarde');
set(gca             , ...
    'FontSize'   , 8           );
set([X_label, Y_label]  , ...
    'FontSize'   , 10          );
set( Title                    , ...
    'FontSize'   , 12          , ...
    'FontWeight' , 'bold'      );

set(gcf, 'PaperPositionMode', 'auto');
% print -depsc2 finalPlot1.eps
% close;
























