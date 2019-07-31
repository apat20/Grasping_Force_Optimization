clear all;
close all;
clc;

centroid = [0.12,0.1,0.08,0.06];
weights = [3.5,2.7,2,1.2];
F_Grasp_1 = [3.66985, 2.83103, 2.09706, 1.25823];
F_Grasp_2 = [1.83495, 1.4155, 1.04854, 0.629126];

% Plotting the centroid against the optimal force exerted by both the
% grasps.
figure;
plot(centroid, F_Grasp_1, 'b');
hold on;
plot(centroid, F_Grasp_2, 'r');
hold on;
xlabel('Height of COM (m)');
ylabel('Optimal squeezing force (N)');
hold off;
legend('Grasp 1', 'Grasp 2', 'location','northwest');
grid on;

% Plotting the weight of glass against the optimal force exerted by both
% the grasps.
figure;
plot(weights, F_Grasp_1, 'b');
hold on;
plot(weights, F_Grasp_2, 'r');
hold on;
xlabel('Weight of the glass (N)');
ylabel('Optimal squeezing force (N)');
hold off;
legend('Grasp 1', 'Grasp 2', 'location','northwest');
grid on;