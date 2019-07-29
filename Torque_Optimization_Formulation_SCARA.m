close all;
clear;
clc;

data = getData('SCARA_2finger_box.txt');
[z,~] = size(data{1});
C = {};

for i=1:z

% Each element of the cell 'data' is split and stores in another cell
% array of size m*n.
% Each row of 'C' contains the split string.
    C{i} = strsplit(data{1}{i}, ' ');
    assignin('base', C{i}{1}, str2num(C{i}{3}));
end

% Data stored in the form of multidimensional arrays for the ease of
% processing.
theta = [theta_1;theta_2;theta_3];

omega(:,:,1) = omega_1;
omega(:,:,2) = omega_2;
omega(:,:,3) = omega_3;

q(:,:,1) = q_1;
q(:,:,2) = q_2;
q(:,:,3) = q_3;

%Creating a SCARA object to get the properties of the manipulator like the
%spatial Jacobian and the Analytical Jacobian.

S = SCARA(theta, omega, q, P_base);
J_spatial = spatialJacobian(S);
g_st = getTransform(S);
J_analytical = S.analyticalJacobian(J_spatial, g_st);


% The transformation matrix for the rigid body transformation from the
% contact frame to the end effector frame.
% Assuming that T is the frame at the end effector and C is the contact
% frame.
G_t1c1 = [R_t1c1, zeros(3);
        zeros(3), R_t1c1];

%The position vectors in the skew symmetric form for our selected problem

I = eye(3,5);
        Z = zeros(3,5);
        z_small = zeros(3,1);
        e_z = [0;0;1];
        
%       This is the wrench basis for the soft finger contact wherein all
%       the dimension of the wrench is preserved to 6 * 1.
        B_c = [I, z_small;
               Z, e_z];

p1_hat = [0,0,-2;
          0,0,0;
         2,0,0];
p2_hat = [0,0,2;
          0,0,0;
          -2,0,0];
      
F_external = [0;0;-5;0;0;0];
mu = 0.1;
sigma = 0.1;

T_max = [50;50;50];
T_min = [-50;-50;-50];

x = 'SF';

fprintf('The type of contact entered is soft fingered contact!');
fprintf('\n');

[Adjoint_1,G_1] = GraspMap(R_oc1, p1_hat, x);
[Adjoint_2,G_2] = GraspMap(R_oc2, p2_hat, x);

%Complete Grasp Map which preserves all the six components of the contact
%wrenches
G = [G_1,G_2];
m = 6; n = 1;

%% Force optimisation formulation without the torque constraints.

cvx_begin
    cvx_precision high
    variable F(n)
    variable fc_1(m,n)
    variable fc_2(m,n)
%   variable Tau(3,1)
    minimize F
    subject to
        G*[fc_1;fc_2] + F_external == 0;
    
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

% FC_1 = B_c*fc_1;
% FC_2 = B_c*fc_2;
%              
% FT_1 = G_t1c1*FC_1;
% FT_2 = G_t1c1*FC_2;
% 
% Tau_1 = J_analytical'*FT_1;
% Tau_2 = J_analytical'*FT_2;

%% Force Optimization formulation including the torque constraints.

cvx_begin
    cvx_precision high
    variable F(n)
    variable fc_1(m,n)
    variable fc_2(m,n)
    variable Tau_1(3,1)
    variable Tau_2(3,1)
    minimize F
    subject to
    G*[fc_1;fc_2] + F_external == 0;
    
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
%Transforming the forces from the contact frame to the end
%effector frame for further analysis.
    FC_1 = B_c*fc_1;
    FC_2 = B_c*fc_2;
             
    FT_1 = G_t1c1*FC_1;
    FT_2 = G_t1c1*FC_2;
             
%Torque constraints on the first manipulator/finger.
    Tau_1 == J_analytical'*FT_1;
    T_min <= Tau_1 <= T_max;

% Torque constraints on the second manipulator/finger.
    Tau_2 == J_analytical'*FT_2;
    T_min <= Tau_2 <= T_max;
cvx_end

%% Torque Optimisation Formulation

cvx_begin
    cvx_precision high
    variable T(n)
    variable Tau_1(3,1)
    %variable Tau_2(3,1)
    minimize T
    subject to
        Tau_1 == J_analytical'*FT_1;
        T_min <= Tau_1 <= T_max;
        max(Tau_1) <= T;
cvx_end

%% Torque Optimization alternative formulation:

cvx_begin
    cvx_precision high
    variable T(n)
    variable fc_1(m,n)
    variable fc_2(m,n)
    variable Tau_1(3,1)
    minimize T
    subject to
    G*[fc_1;fc_2] + F_external == 0;
    
    fc1 = fc_1(1:2);
    fc2 = fc_2(1:2);
     
    norm(fc1) <= mu*fc_1(3);
    norm(fc2) <= mu*fc_2(3);
     
    fc_1(3) >= 0;
    fc_2(3) >= 0;
     
    norm(fc_1(6)) <= sigma*fc_1(3);
    norm(fc_2(6)) <= sigma*fc_2(3);
   
%     norm(fc_1) <= F;
%     norm(fc_2) <= F;
    
    FC_1 = B_c*fc_1;
    FC_2 = B_c*fc_2;
             
    FT_1 = G_t1c1*FC_1;
    FT_2 = G_t1c1*FC_2;
     
    Tau_1 == J_analytical'*FT_1;
    T_min <= Tau_1 <= T_max;
    max(Tau_1) <= T;
cvx_end


