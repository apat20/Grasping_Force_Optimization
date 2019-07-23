% Box tilting formulation done.

close all;
clear;
clc;

x = 'SF';

%  Getting the necessary rotation matrices:
R_OC1 = [0,1,0;
         0,0,1;
         1,0,0];
R_OC2 = [1,0,0;
         0,0,-1;
         0,1,0];

% Getting the necessary rotation matrices between the object and the
% reference frames at the edge of the object
R_OE1 = [1,0,0;
        0,1,0;
         0,0,1];
R_OE2 = [1,0,0;
        0,-1,0;
         0,0,1];

p_OC1 = [1;-2;0];
p_OC2 = [1;2;0];
p_OE1 = [-2.5; -2;-1.5];
p_OE2 = [-2.5; 2;-1.5];

% Getting the skew symmetric forms of the position vectors:
p_OC1_hat = skewSymmetric(p_OC1);
p_OC2_hat = skewSymmetric(p_OC2);
% p_OE1_hat = skewSymmetric(p_OE1);
% p_OE2_hat = skewSymmetric(p_OE2);

% Get the necessary Grasp Map:
G_1 = GraspMap(R_OC1, p_OC1_hat, x);
G_2 = GraspMap(R_OC2, p_OC2_hat, x);

% Concatenating the individual grasp maps:
G = [G_1,G_2];

F_external = [0; 0; -20; 0; 0; 0];
% beta = pinv(G)*-F_external;
% fc1 = beta(1:6,1);
% fc2 = beta(7:12,1);

m = 6; n = 1;
mu = 0.1;
sigma = 0.1;

g_OE1 = [R_OE1, p_OE1;
         zeros(1,3), 0];
Ad_OE1 = GetAdjoint(g_OE1);

% Ad_OE1 = [R_OE1, zeros()]

g_OE2 = [R_OE2, p_OE2;
         zeros(1,3), 0];
Ad_OE2 = GetAdjoint(g_OE2);


%% 
cvx_begin
    cvx_precision high
    variable F(n)
    variable fc_1(m,n)
    variable fc_2(m,n)
    variable f_E1(m,n)
    variable f_E2(m,n)
    minimize F
    subject to
        G*[fc_1;fc_2] + F_external + Ad_OE1*f_E1 + Ad_OE2*f_E2  >= 0;
    
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
        
%       Implementing the static friction constraints on the edge wrenches
%       generated at the edge
        f_e1 = f_E1(1:2);
        f_e2 = f_E2(1:2);
        
        norm(f_e1) < mu*f_E1(3)
        f_E1(3) > 0;
        
        norm(f_e2) < mu*f_E2(3)
        f_E2(3) > 0;
        
%       Implementing the moment constraints for the edge wrenches generated
%       at the edge.
        
        f_E1(4) == 0;f_E2(4) == 0;
        f_E1(6) == 0;f_E2(6) == 0;
        f_E1(5) > 0;f_E2(5) > 0;
        
        
cvx_end

% %% 
% cvx_begin
%     cvx_precision high
%     variable F_O(n)
%     variable f_O(m,n)
%     variable f_A(m,n)
%     minimize F_O
%     subject to
%         G*[fc_1;fc_2] - f_O == 0;
%         Ad_AO*f_O == f_A;
%         
%         f_a1 = f_A(1:2);
%         norm(f_a1) < mu*f_A(3)
%         f_A(3) > 0;
%         
%         f_A(4) == 0;
%         f_A(6) == 0;
%         f_A(5) > 0;
%         
%         norm(f_O) <= F_O;
% cvx_end