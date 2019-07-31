%% Glass picking Force Optimization Formulation.
% This file is used to read the data and process it into the necessary
% format for computation purposes.
 
clear all;
close all;
clc;

% Opening the file which contains the data for glass picking using two
% contact points on each side.
data = getData('Glass_1_6.txt');
[m,~] = size(data{1});
C = {};

% In the for loop we assign a value to a varaible which is then used for
% our processing purposes.
% The variable names are stored in the first column of the cell array 'C'
% whereas the values to be assigned are stored in the third column.

for i=1:m

% Each element of the cell 'data' is split and stores in another cell
% array of size m*n.
% Each row of 'C' contains the split string.
    C{i} = strsplit(data{1}{i}, ' ');
    assignin('base', C{i}{1}, str2num(C{i}{3}));
end

% Specifying the type pf contact to be used.
x = 'SF';

% Obtaining the skew symmetric forms of the poistion vectors.
p_OC1_hat = skewSymmetric(p_OC1);
p_OC2_hat = skewSymmetric(p_OC2);
p_OC3_hat = skewSymmetric(p_OC3);
p_OC4_hat = skewSymmetric(p_OC4);

% Calculating the grasp map between the contact frames and the object
% frame.
G_1 = GraspMap(R_OC1, p_OC1_hat, x);
G_2 = GraspMap(R_OC2, p_OC2_hat, x);
G_3 = GraspMap(R_OC3, p_OC3_hat, x);
G_4 = GraspMap(R_OC4, p_OC4_hat, x);

% Concatenating the grasp maps into a single array 'G'.
G = [G_1,G_2,G_3,G_4];

% Specifying certain paramters which we will be using in our computation.
% The dimensions of the contact forces are m*n. 
m = 6; n = 1;

% Coefficient of friction.
mu = 0.1;

% Coefficient of torsional friction.
sigma = 0.1;

%%
% Optimization formulation using 'cvx' for solving the optimization problem
% in order to compute the optimal contact forces.

cvx_begin
    cvx_precision high
    variable F(n)
    variable fc_1(m,n)
    variable fc_2(m,n)
    variable fc_3(m,n)
    variable fc_4(m,n)
    minimize F
    subject to
        G*[fc_1;fc_2;fc_3;fc_4] + F_external == 0;
    
        fc1 = fc_1(1:2);
        fc2 = fc_2(1:2);
        fc3 = fc_3(1:2);
        fc4 = fc_4(1:2);
     
%       Friction cone condition using the coefficient of friction at the
%       contact locations
        norm(fc1) <= mu*fc_1(3);
        norm(fc2) <= mu*fc_2(3);
        norm(fc3) <= mu*fc_3(3);
        norm(fc4) <= mu*fc_4(3);
      
        fc_1(3) >= 0;
        fc_2(3) >= 0;
        fc_3(3) >= 0;
        fc_4(3) >= 0;
        
%       Condition for the torsional coefficient implemented again as a
%       second order cone constraint.
        norm(fc_1(6)) <= sigma*fc_1(3);
        norm(fc_2(6)) <= sigma*fc_2(3);
        norm(fc_3(6)) <= sigma*fc_3(3);
        norm(fc_4(6)) <= sigma*fc_4(3);
        
%       Setting all the moment components about the 't_i' and 'o_i' axis
%       equal to zero.
        fc_1(4) == 0;fc_2(4) == 0;fc_3(4) == 0;fc_4(4) == 0;
        fc_1(5) == 0;fc_2(5) == 0;fc_3(5) == 0;fc_4(5) == 0;
%         fc_1(6) == 0;fc_2(6) == 0;fc_3(6) == 0;fc_4(6) == 0;
        
        
        fc_1 <= F;
        fc_2 <= F;
        fc_3 <= F;
        fc_4 <= F;
        
cvx_end
