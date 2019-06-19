%This code implements the Conic Formulation described by Stephen Boyd in
%his paper 'Fast Computation of Optimal Contact Forces'
%Here we have used just the grasp map to encode the contact information.

%In this implementation we are assuming that the configurations of both the
%manipulators is the same and the trasnformation between the contact frames
%and the tool frame for both the contact points is the same.
clear;
clc;

%The angles of rotation for each of the revolute joints of the SCARA
%manipulator.
theta_1 = 30;
theta_2 = 20;
theta_3 = 20;

theta = [theta_1;theta_2;theta_3];
% disp(theta);

%Omega encodes the information regarding the axis of rotation of the SCARA
%manipulator.
omega_1 = [0;0;1];
omega_2 = [0;0;1];
omega_3 = [0;0;1];

omega(:,:,1) = omega_1;
omega(:,:,2) = omega_2;
omega(:,:,3) = omega_3;

% disp(omega);

%Position vectors for the aribitarly chosen point on the axis of rotation.
q_1 = [0;0;0];
q_2 = [0;20;0];
q_3 = [0;50;0];

q(:,:,1) = q_1;
q(:,:,2) = q_2;
q(:,:,3) = q_3;

% disp(q);

% 'I' is the identity matrix
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

%Calculating the Spatial Jacobian using the function Spatial Jacobian. For
%more details on the nature of the input arguments refer to the script file
%of the function.
[g1, J_spatial] = spatialJacobian(theta, omega, g_zero, q);

fprintf('The Spatial Jacobian is:');
fprintf('\n');
disp(J_spatial);

%Here we extract the vector from the last transformation used for
%calculating the Analytical Jacobian.
p = g1(1:3,4,3);
fprintf('The position vector used for computing the Analytical Jacobian');
fprintf('\n');
disp(p)

%Computing the Analytical Jacobian using the Spatial Jacobian
J_analytical = AnalyticalJacobian(J_spatial, p);
fprintf('The Ananlytical Jacobian is:');
fprintf('\n');
disp(J_analytical);

%%
fprintf('Grasping Force Optimisation using CVX');
fprintf('\n');
fprintf('Type "SF" at the prompt to select the Soft Finger Contact');
fprintf('\n');
fprintf('Type "PF" at the prompt to select Point Contact with Friction');
fprintf('\n');

%The rotation matrices for our selected problem
%The rotation matrices encode the orientation of the reference frame at the
%contact point with respect to the object reference frame. i.e the
%reference frame present at the COM of the object.
R_1 = [0,1,0;
       0,0,1;
       1,0,0];
 
R_2 = [1,0,0;
       0,0,-1;
       0,1,0];
   
% The rotation matrix for the transformation between the contact frame and
% the end effector frame.

R = [1,0,0;
     0,1,0;
     0,0,1];
% The transformation matrix for the rigid body transformation from the
% contact frame to the end effector frame.
% Assuming that T is the frame at the end effector and C is the contact
% frame.
G_tc = [R, zeros(3);
        zeros(3), R];

%The position vectors in the skew symmetric form for our selected problem
p1_hat = [0,0,2;
          0,0,0;
         -2,0,0];
p2_hat = [0,0,-2;
          0,0,0;
          2,0,0];

F_external = [0;0;-10;0;0;0];
mu = 0.1;
sigma = 0.1;

Tau_min = [10;10;10];
Tau_max = [100;100;100];

%User input to specify the type of contact to be used
%We have consider only two types of contact that is the Point contact with
%friction and the Soft finger contact.
prompt = 'Enter the type of contact:';
x = input(prompt, 's');

if x == 'SF'
    fprintf('The type of contact entered is soft fingered contact!');
    fprintf('\n');

%   Wrench basis for soft finger contact:
    B_c = [1,0,0,0
           0,1,0,0;
           0,0,1,0;
           0,0,0,0;
           0,0,0,0;
           0,0,0,1];
    
    G_1 = GraspMap(R_1, p1_hat, x);
    G_2 = GraspMap(R_2, p2_hat, x);
    G = [G_1,G_2];
    
    m = 4; n = 1;
    
    fprintf('The grasp map for the soft finger contact:');
    fprintf('\n');
    disp(G)
   
    fprintf('The external force acting on the object is:');
    fprintf('\n');
    disp(F_external)
    
    fprintf('The friction coefficient is:');
    fprintf('\n')
    disp(mu);
    
    fprintf('The torsional coefficient is:');
    fprintf('\n')
    disp(sigma);
    
    cvx_begin
        cvx_precision high
        variable F(n)
        variable fc_1(m,n)
        variable fc_2(m,n)
        variable Tau(3,1)
        minimize F
        subject to
            G*[fc_1;fc_2] + F_external == 0;
    
            fc1 = fc_1(1:2);
            fc2 = fc_2(1:2);
     
            norm(fc1) <= mu*fc_1(3);
            norm(fc2) <= mu*fc_2(3);
     
            fc_1(3) >= 0;
            fc_2(3) >= 0;
     
            norm(fc_1(4)) <= sigma*fc_1(3);
            norm(fc_2(4)) <= sigma*fc_2(3);
   
            norm(fc_1) <= F;
            norm(fc_2) <= F;
            
%           Transforming the forces from the contact frame to the end
%           effector frame for further analysis.
            FC_1 = B_c*fc_1;
            FC_2 = B_c*fc_2;
            
            FT_1 = G_tc*FC_1;
            FT_2 = G_tc*FC_2;
            
%           Torque constraints on the manipulator.
            Tau == J_analytical'*FT_1;
            Tau_min <= Tau <= Tau_max;
            
      cvx_end

%Conditional for point contact with friction 
%The optimisationproblem is solved within this loop as the optimisation
%constraints are different for point contact with friction.
elseif x == 'PF'
    fprintf('The type of contact entered is point contact with fingers!');
    fprintf('\n');

%   Wrench basis for point contact with friction:
    B_c = [1,0,0;
           0,1,0;
           0,0,1;
           0,0,0;
           0,0,0;
           0,0,0];
    
    G_1 = GraspMap(R_1, p1_hat, x);
    G_2 = GraspMap(R_2, p2_hat, x);
    G = [G_1,G_2];
    
    m = 3; n = 1;
    
    fprintf('The grasp map for the point contact with friction:');
    fprintf('\n');
    disp(G)
    
    fprintf('The external force acting on the object is:');
    fprintf('\n');
    disp(F_external)
    
    fprintf('The friction coefficient is:');
    fprintf('\n')
    disp(mu);
    
    cvx_begin
        cvx_precision high
        variable F(n)
        variable fc_1(m,n)
        variable fc_2(m,n)
        variable Tau(3,1)
        minimize F
        subject to
            G*[fc_1;fc_2] + F_external == 0;
            %Extracting the tangential components in a separate vector
            fc1 = fc_1(1:2);
            fc2 = fc_2(1:2);
            
            norm(fc1) <= mu*fc_1(3);
            norm(fc2) <= mu*fc_2(3);
            
            norm(fc_1) <= F;
            norm(fc_2) <= F;
            
%           Transforming the forces from the contact frame to the end
%           effector frame for further analysis.
            FC_1 = B_c*fc_1;
            FC_2 = B_c*fc_2;
            
            FT_1 = G_tc*FC_1;
            FT_2 = G_tc*FC_2;
            
%           Torque constraints on the manipulator.
            Tau == J_analytical'*FT_1;
            Tau_min <= Tau <= Tau_max;
    cvx_end
end

