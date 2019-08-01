%% Grasping Force Optimization formulation for tilting an object.
% We assume the dimensions of the box to be constant throughout the
% process. 

close all;
clear all;
clc;

% Specifying the contact model:
x = 'SF';

% The angle at which the object has been tilted:
theta = 30;

%  Getting the necessary rotation matrices of the contact frames with 
%  respect to the object frame.

R_OC1 = [0,1,0;
         0,0,1;
         1,0,0];

R_OC2 = [1,0,0;
         0,0,-1;
         0,1,0];

% Getting the necessary position vectors of the contact frames with respect
% to the object reference frame.

p_OC1 = [1.5;-2;0];

p_OC2 = [1.5;2;0];

% Rotation matrices for the edge frames with respect to the 

R_OE1 = EulerRotation(0,theta,0);
R_OE2 = EulerRotation(0,theta,0);

% The position vectors for the edge frames with respect to the object
% frames:

p_OE1 = [-2.5*cos(deg2rad(theta)); -2;-1.5*cos(deg2rad(theta))];
p_OE2 = [-2.5*cos(deg2rad(theta)); 2;-1.5*cos(deg2rad(theta))];

% Getting the skew symmetric forms of the position vectors:
p_OC1_hat = skewSymmetric(p_OC1);
p_OC2_hat = skewSymmetric(p_OC2);

G_1 = GraspMap(R_OC1, p_OC1_hat, x);
G_2 = GraspMap(R_OC2, p_OC2_hat, x);

G = [G_1,G_2];

F_external = [0; 0; -20*cos(deg2rad(theta)); 0; 0; 0];

m = 6; n = 1;
% Parameters for friction model
mu = 0.1;
sigma = 0.1;
% e11 = 1; e12 = 1; e1r = 0.2;
% e21 = 1; e22 = 1; e2r = 0.2;


g_OE1 = [R_OE1, p_OE1;
         zeros(1,3), 1];
Ad_OE1 = GetAdjointWrench(g_OE1);

g_OE2 = [R_OE2, p_OE2;
         zeros(1,3), 1];
Ad_OE2 = GetAdjointWrench(g_OE2);

%% Grasping force optimization formulation for the box tilting application
cvx_begin
    cvx_precision high
    variable F(n)
    variable fc_1(m,n)
    variable fc_2(m,n)
    variable f_E1(m,n) 
    variable f_E2(m,n)
    minimize F
    subject to
    
%   The primary wrench balance condition:
        G*[fc_1;fc_2] + F_external + Ad_OE1*f_E1 + Ad_OE2*f_E2  == 0;
  
%%%%%%%%%%% Friction constraints at robot-object contacts
% Fingers can only push not pull
        fc_1(3) >= 0;
        fc_2(3) >= 0; 
 % No moments about the tangential axis at the cotacts              
        fc_1(4) == 0;fc_2(4) == 0;
        fc_1(5) == 0;fc_2(5) == 0;
%       Extracting the first two components from the contact wrench for
%       ease in computation.
        fc1 = fc_1(1:2);
        fc2 = fc_2(1:2);
 
% Isotropic Soft Finger Contact Friction model
        norm(fc1) <= mu*fc_1(3);
        norm(fc2) <= mu*fc_2(3);
        norm(fc_1(6)) <= sigma*fc_1(3);
        norm(fc_2(6)) <= sigma*fc_2(3);
 
%       fc_1(6) == 0;fc_2(6) == 0; % Point Contact

% Anisotropic Soft Finger Contact Friction model
%         f_temp1 = [fc1(1)/e11;fc1(2)/e12; fc_1(6)/e1r];
%         norm(f_temp1) <= mu*fc_1(3);
%         f_temp2 = [fc2(1)/e21;fc2(2)/e22; fc_2(6)/e2r];
%         norm(f_temp2) <= mu*fc_2(3);
          


         fc_1(3) <= F;
         fc_2(3) <= F;
        
%       Implementing the static friction constraints on the edge wrenches
%       generated at the edge
        f_e1 = f_E1(1:2);
        f_e2 = f_E2(1:2);
        
        norm(f_e1) <= mu*f_E1(3)
        f_E1(3) > 0;
        
        norm(f_e2) <= mu*f_E2(3)
        f_E2(3) > 0;
        
%       Implementing the moment constraints for the edge wrenches generated
%       at the edge.
        
         f_E1(4) == 0;f_E2(4) == 0;
         f_E1(6) == 0;f_E2(6) == 0;
         f_E1(5) == 0;f_E2(5) == 0;
        
cvx_end

fprintf("The value of the angle of tilt:");
fprintf('\n');
disp(theta);

fprintf("The value of f_E1:");
fprintf('\n');
disp(f_E1);

fprintf("The value of Adjoint times f_E1:");
fprintf('\n');
disp(Ad_OE1*f_E1);

fprintf("The value of f_E2:");
fprintf('\n');
disp(f_E2);

fprintf("The value of Adjoint times f_E2:");
fprintf('\n');
disp(Ad_OE1*f_E2);