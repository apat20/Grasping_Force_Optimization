% Box tilting formulation done.

close all;
clear;
clc;

x = 'SF';

%  Getting the necessary rotation matrices:
% R_OC1 = EulerRotation(0,90,0);
% R_OC2 = EulerRotation(90,0,90);
R_AO = EulerRotation(0,0,0);

R1 = [0,1,0;0,0,1;1,0,0];
R2 = [1,0,0;0,0,-1;0,1,0];

% R1 = [0,0,1;
%       0,1,0;
%      -1,0,0];
% 
% R2 = [0,0,1;
%       1,0,0;
%       0,1,0];

p_OC1 = [4;2;0];
p_OC2 = [-4;2;0];
p_AO = [0;2.5;-1.5];

% Getting the skew symmetric forms of the position vectors:
p_OC1_hat = skewSymmetric(p_OC1);
p_OC2_hat = skewSymmetric(p_OC2);
p_AO_hat = skewSymmetric(p_AO);

% Get the necessary Grasp Map:
G_1 = GraspMap(R1, p_OC1_hat, x);
G_2 = GraspMap(R2, p_OC2_hat, x);

% Concatenating the individual grasp maps:
G = [G_1,G_2];

F_external = [10; 0; 20; 0; 0; 20];
% beta = pinv(G)*-F_external;
% fc1 = beta(1:6,1);
% fc2 = beta(7:12,1);

m = 6; n = 1;
mu = 0.1;
sigma = 0.1;

g_AO = [R_AO, p_AO;
        zeros(1,3), 0];
    
Ad_AO = GetAdjoint(g_AO);


%% 
cvx_begin
    cvx_precision high
    variable F(n)
    variable fc_1(m,n)
    variable fc_2(m,n)
%   variable Tau(3,1)
    minimize F
    subject to
        G*[fc_1;fc_2] - F_external == 0;
    
        fc1 = fc_1(1:2);
        fc2 = fc_2(1:2);
     
        norm(fc1) <= mu*fc_1(3);
        norm(fc2) <= mu*fc_2(3);
     
        fc_1(3) >= 0;
        fc_2(3) >= 0;
     
        norm(fc_1(6)) <= sigma*fc_1(3);
        norm(fc_2(6)) <= sigma*fc_2(3);
   
        norm(fc_1) <= F;
        norm(fc_2) <= F;
cvx_end

%% 
cvx_begin
    cvx_precision high
    variable F_O(n)
    variable f_O(m,n)
    variable f_A(m,n)
    minimize F_O
    subject to
        G*[fc_1;fc_2] - f_O == 0;
        Ad_AO*f_O == f_A;
        
        f_a1 = f_A(1:2);
        norm(f_a1) < mu*f_A(3)
        f_A(3) > 0;
        
        f_A(4) == 0;
        f_A(6) == 0;
        f_A(5) > 0;
        
        norm(f_O) <= F_O;
cvx_end