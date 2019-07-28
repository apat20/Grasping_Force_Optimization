close all;
clear all;
clc;

x = 'SF';

p_OC1 = [-0.2448;   -3.7279;   -0.2250];
p_OC2 = [-0.2448;   -5.4558;    0.0500];
p_OC3 = [-0.2448;   -7.1836;    0.3250];
p_OC4 = [-0.2448;    5.6128;    0.0750];

R_OC1 = [0,-1,0;
         0,0,-1;
         1,0,0];
     
R_OC2 = [0,-1,0;
         0,0,-1;
         1,0,0];

R_OC3 = [0,-1,0;
         0,0,-1;
         1,0,0];
     
R_OC4 = [-1,0,0;
         0,0,1;
         0,1,0];

p_OC1_hat = skewSymmetric(p_OC1);
p_OC2_hat = skewSymmetric(p_OC2);
p_OC3_hat = skewSymmetric(p_OC3);
p_OC4_hat = skewSymmetric(p_OC4);

G_1 = GraspMap(R_OC1, p_OC1_hat, x);
G_2 = GraspMap(R_OC2, p_OC2_hat, x);
G_3 = GraspMap(R_OC3, p_OC3_hat, x);
G_4 = GraspMap(R_OC4, p_OC4_hat, x);

G = [G_1,G_2,G_3,G_4];

F_external = [0; 0; -20; 0; 0; 0];

m = 6; n = 1;
mu = 0.1;
sigma = 0.1;

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
     
        norm(fc1) <= mu*fc_1(3);
        norm(fc2) <= mu*fc_2(3);
        norm(fc3) <= mu*fc_3(3);
        norm(fc4) <= mu*fc_4(3);
     
        fc_1(3) >= 0;
        fc_2(3) >= 0;
        fc_3(3) >= 0;
        fc_4(3) >= 0;
        
        norm(fc_1(6)) <= sigma*fc_1(3);
        norm(fc_2(6)) <= sigma*fc_2(3);
        norm(fc_3(6)) <= sigma*fc_3(3);
        norm(fc_4(6)) <= sigma*fc_4(4);
        
        fc_1(4) == 0;fc_2(4) == 0;fc_3(4) == 0;fc_4(4) == 0
        fc_1(5) == 0;fc_2(5) == 0;fc_3(5) == 0;fc_4(5) == 0
   
        fc_1 <= F;
        fc_2 <= F;
        fc_3 <= F;
        fc_4 <= F;
        
cvx_end
