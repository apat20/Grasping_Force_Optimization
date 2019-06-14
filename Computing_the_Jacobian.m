%This code is used to calculate the Spatial Jacobian for the SCARA
%manipulator.
%%
clear;
clc;

%The angles of rotation for each of the revolute joints of the SCARA
%manipulator.
theta_1 = 30;
theta_2 = 20;
theta_3 = 20;

%Omega encodes the information regarding the axis of rotation of the SCARA
%manipulator.
omega = [0;0;1];

%Position vectors for the aribitarly chosen point on the axis of rotation.
q_1 = [0;0;0];
q_2 = [0;20;0];
q_3 = [0;50;0];

%I is the identity matrix
I = eye(3);

%P is the position vector of the end effecctor or tool frame with respect
%to the inertial frame located at the base of the manipulator or in our
%case the base of the SCARA manipulator.
%P = [0; l1+l2; l0]
P = [0;50;10];

%This is the rigid body transformation matrix for the zero configuration of
%the manipulator under consideration. In our case it is the SCARA
%manipulator.
g_zero = [I, P;
          zeros(1,3),1];
fprintf('The g_st_zero transformation matrix is:');
fprintf('\n');
disp(g_zero);

%Calculating the exponential of twists for all the revolute joints in the
%SCARA manipulator.
exp_twist_theta1 = GetExponential(omega, theta_1, q_1);
exp_twist_theta2 = GetExponential(omega, theta_2, q_2);
exp_twist_theta3 = GetExponential(omega, theta_3, q_3);

% disp(exp_twist_theta1);
%%
%Calculating the transformation matrics for our manipulator using the
%product of exponentials formula.
%These transformation matrices are calculated upto each of the revolute
%joints present in the manipulator.
g1_1 = exp_twist_theta1*g_zero;
g1_2 = exp_twist_theta1*exp_twist_theta2*g_zero;
g1_3 = exp_twist_theta1*exp_twist_theta2*exp_twist_theta3*g_zero;

fprintf('The final rigid body transformation for the SCARA manipulator is:');
fprintf('\n');
disp(g1_3);

%Computing the Adjoint matrix using the function GetAdjoint for all three
%transformations.
Adjoint_matrix_1 = GetAdjoint(g1_1);
Adjoint_matrix_2 = GetAdjoint(g1_2);
Adjoint_matrix_3 = GetAdjoint(g1_3);
fprintf('The Adjoint matrix is:');
fprintf('\n');
disp(Adjoint_matrix_3);

%%
%Now we start computing the Spatial Jacobian
%Computing the twists for each of the revolute joints in the manipulator
eta_1 =GetTwist(omega, q_1); 
eta_2 =GetTwist(omega, q_2);
eta_3 =GetTwist(omega, q_3); 

%Computing eta dash for the second and the third revolute joints as they
%are used as the columns of the Jacobian
eta_2_dash = GetTwistDash(Adjoint_matrix_2, eta_2);
eta_3_dash = GetTwistDash(Adjoint_matrix_3, eta_3);

%The Spatial Jacobian for the SCARA manipulator without the prismatic joint
J_spatial = [eta_1, eta_2_dash, eta_3_dash];
fprintf('The Spatial Jacobian is:');
fprintf('\n');
disp(J_spatial);

%Computing the Analytical Jacobian using the Spatial Jacobian
p = g1_3(1:3,4);
fprintf('The position vector used for computing the Analytical Jacobian');
fprintf('\n');
disp(p)

J_analytical = AnalyticalJacobian(J_spatial, p);
fprintf('The Ananlytical Jacobian is:');
fprintf('\n');
disp(J_analytical);
disp(J_analytical');

%Consider an arbitary force vector just to establish the relationship
%between the torque and the external forces.

F_external = [80;20;-10;10;20;20];
fprintf('The external force acting on the object is:');
fprintf('\n');
disp(F_external)

%Calculate the Torques required from the analytical Jacobian and the
%external Force vector

% tau = J_analytical'*F_external;
% fprintf('The torque vector calculated is as follows:');
% fprintf('\n');
% disp(tau);
% disp(norm(tau));


fprintf('exp_twist_theta1');
fprintf('\n');
disp(exp_twist_theta1);


fprintf('exp_twist_theta2');
fprintf('\n');
disp(exp_twist_theta2);


fprintf('exp_twist_theta3');
fprintf('\n');
disp(exp_twist_theta3);

fprintf('G1_1');
fprintf('\n');
disp(g1_1)

fprintf('G1_2');
fprintf('\n');
disp(g1_2)

fprintf('G1_3');
fprintf('\n');
disp(g1_3)

      0   49.4599   53.7051
         0  -17.6604   14.0033
         0         0         0
         0         0         0
         0         0         0
    1.0000    1.0000    1.0000

