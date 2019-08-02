close all;
clear all;
clc;

% Selecting the type of contact model
c  = 'SF';

% Reading the required data from the text file.
data = getData('BAXTER_BOX.txt');
[x,~] = size(data{1});
C = {};

% In the for loop we assign a value to a varaible which is then used for
% our processing purposes.
% The variable names are stored in the first column of the cell array 'C'
% whereas the values to be assigned are stored in the third column.

for i=1:x

% Each element of the cell 'data' is split and stores in another cell
% array of size m*n.
% Each row of 'C' contains the split string.
    C{i} = strsplit(data{1}{i}, ' ');
    assignin('base', C{i}{1}, str2num(C{i}{3}));
end

% Computing the position vector of the edge frames {E1} and {E2} with
% respect to the object frame {O} while it is being tilted. Here 'alpha' is
% the angle of tilt with respect to the surface.
% p_OE1_tilt = [p_OE1(1,1)*cos(deg2rad(alpha)); p_OE1(2,1); p_OE1(1,1)*sin(deg2rad(alpha)) + p_OE1(3,1)];
% p_OE2_tilt = [p_OE2(1,1)*cos(deg2rad(alpha)); p_OE2(2,1); p_OE2(1,1)*sin(deg2rad(alpha)) + p_OE2(3,1)];

% Computing the rotation matrix for the orientation of the edge frames
% {E1}and {E2} with respect to the object frame {O}. The orientation of the
% object frame changes as the object is being tilted at an angle.
x_OE = [cos(deg2rad(alpha)); 0; sin(deg2rad(alpha))];
y_OE = [0;1;0];
z_OE = [-sin(deg2rad(alpha)); 0; cos(deg2rad(alpha))];

R_OE1 = [x_OE,y_OE,z_OE];
R_OE2 = [x_OE,y_OE,z_OE];

p_OE1_tilt = R_OE1'*p_OE1;
p_OE2_tilt = R_OE2'*p_OE2;

% Storing the obtained data in multidimensional arrays for further
% computing purposes.
R_OC(:,:,1) = R_OC1;
R_OC(:,:,2) = R_OC2;

R_OE(:,:,1) = R_OE1;
R_OE(:,:,2) = R_OE2;

p_OE(:,:,1) = p_OE1_tilt;
p_OE(:,:,2) = p_OE2_tilt;

p_OC(:,:,1) = p_OC1;
p_OC(:,:,2) = p_OC2;

% Creating the box object of the required properties and applying the
% methods of the class to the properties.
B = BOX(alpha, R_OC, R_OE, p_OC, p_OE);
p_OC_hat = B.skewSymmetric(B.p_OC);

% Calculate the grasp maps for the contact reference frames {C1} and
% {C2} with respect to the object frame {O} 
G_multi = B.graspMap(B.R_OC, p_OC_hat, c);

% Calculate the Adjoint matrix for transforming the wrenches from the edge
% frames into the object frame {O}.
Adjoint = B.getAdjoint(B.R_OE, B.p_OE);

G1 = G_multi(:,:,1);
G2 = G_multi(:,:,2);
G = [G1,G2];

Ad_OE1 = Adjoint(:,:,1);
Ad_OE2 = Adjoint(:,:,2);

% The transformation matrix for the rigid body transformation from the
% contact frame to the end effector frame.
% Assuming that T is the frame at the end effector and C is the contact
% frame. We have assumed that the orientation of both the frames is the
% same.
% Also we assume that the contact frames and the tool frames coincide.

G_TC = [eye(3), zeros(3);
        zeros(3), eye(3)];

% Here the external force acting on the object is due to its self weight.
% The weight of the object is 2kg.
F_external = [0;0;-20*cos(deg2rad(alpha));0;0;0];
%% Computing the Jacobian for Baxter left and right arms.
num_joints = numel(theta);

% Define rotation axes (wrt. arm_mount frame for both the arms)
w4 = w2; w5 = w3; w6 = w2; w7 = w3;
wr = [w1, w2, w3, w4, w5, w6, w7];

% Frame origins (wrt. arm_mount frame for both the arms)
qr = [q1, q2, q3, q4, q5, q6, q7];

% Define g_st0 (end point) (Left-Arm)
g_st0_left = [0,0,1,1.213; 0,1,0,-0.002; -1,0,0,0.190; 0,0,0,1.0000];

% Define g_st0 (end point) (Right-Arm)
g_st0_right = [0,0,1,1.213; 0,1,0,-0.002; -1,0,0,0.190; 0,0,0,1.0000];

% Define base transformation of (Right-Arm)
g_base_right = [0.7071, 0.7071, 0.0, 0.025; -0.7071, 0.7071, 0.0, -0.220;
               0.0, 0.0, 1.0, 0.109; 0, 0, 0, 1.0];
          
% Define base transformation of (Left-Arm)
g_base_left = [0.7071, -0.7071, 0.0, 0.025; 0.7071, 0.7071, 0.0, 0.220;
               0.0, 0.0, 1.0, 0.109; 0, 0, 0, 1.0];

% Transform joint axes and origins into base frame
wr_l = g_base_left(1:3, 1:3) * wr; 
qr_l = g_base_left(1:3, 1:3) * qr + g_base_left(1:3, 4);
gst0_l = g_base_left * g_st0_left;

wr_r = g_base_right(1:3, 1:3) * wr;
qr_r = g_base_right(1:3, 1:3) * qr + g_base_right(1:3, 4);
gst0_r = g_base_right * g_st0_right;

% Calculating the Jacobian for the left arm of the Baxter robot.
% [left_spatial_jac,gl] = baxter_spatial_jacobian_mat(qr_l, wr_l, theta, gst0_l);
[g_st, J_spatial] = getSpatialJacobian(theta, wr_l, qr_l, gst0_l);

g = g_st(:,:,num_joints);
 p_vector = g(1:3,4);
J_analytical = AnalyticalJacobian(J_spatial, p_vector);
%% Grasping force optimization formulation for the box tilting application with Force minimization objective
cvx_begin
    cvx_precision high
%   Declaring the optimization variables for our problem
    variable F(n)
    variable fc_1(m,n)
    variable fc_2(m,n)
    variable f_E1(m,n) 
    variable f_E2(m,n)
    minimize F
    subject to
    
%   The primary wrench balance condition:
        G*[fc_1;fc_2] + F_external + Ad_OE1*f_E1 + Ad_OE2*f_E2  == 0;
   
%       Extracting the first two components from the contact wrench for
%       ease in computation.
        fc1 = fc_1(1:2);
        fc2 = fc_2(1:2);
        
%%%%%%%%%%%% Friction constraints at robot-object contacts%%%%%%%%%%%%%%%%
% Fingers can only push not pull
        fc_1(3) >= 0;
        fc_2(3) >= 0;

% No moments about the tangential axis at the cotacts
        fc_1(4) == 0;fc_2(4) == 0;
        fc_1(5) == 0;fc_2(5) == 0;

        
% Isotropic Soft Finger Contact Friction model
        norm(fc1) < mu*fc_1(3);
        norm(fc2) < mu*fc_2(3);
        norm(fc_1(6)) <= sigma*fc_1(3);
        norm(fc_2(6)) <= sigma*fc_2(3);
        
%       Point Contact with friction model
%       fc_1(6) == 0;fc_2(6) == 0;
        
% Anisotropic Soft Finger Contact Friction model
%         f_temp1 = [fc1(1)/e11;fc1(2)/e12; fc_1(6)/e1r];
%         norm(f_temp1) <= mu*fc_1(3);
%         f_temp2 = [fc2(1)/e21;fc2(2)/e22; fc_2(6)/e2r];
%         norm(f_temp2) <= mu*fc_2(3);

% The normal force at the contact frame should less than the scalar value
% 'F' which is being minimized.
        fc_1(3) <= F;
        fc_2(3) <= F;
        
%       Implementing the static friction constraints on the edge wrenches
%       generated at the edge
        f_e1 = f_E1(1:2);
        f_e2 = f_E2(1:2);
        
        norm(f_e1) < mu*f_E1(3)
        f_E1(3) > 0;
        
        norm(f_e2) < mu*f_E2(3);
        f_E2(3) > 0;
        
%       Implementing the moment constraints for the edge wrenches generated
%       at the edge.
        f_E1(4) == 0;f_E2(4) == 0;
        f_E1(6) == 0;f_E2(6) == 0;
        f_E1(5) == 0;f_E2(5) == 0;
         
%%%%%%%%%%%%%Torque Constraints for the Baxter arm%%%%%%%%%%%%%%%%%%%%%%%%
         
% % Computing the Torques in the left arm of Baxter required to produce the 
% % necessary squeezing force at the contact points.
%         Tau_1 = J_analytical'*G_TC*fc_1;
%        
% % Implementing the torque limits as the torque constraints on the Baxter
% % joint torques.
%         Torque_limits_min <= Tau_1 <= Torque_limits_max;
% %         Tau_2 = left_spatial_jac'*fc_2;
% %         Torque_limits_min <= Tau_2 <= Torque_limits_max;
cvx_end

fprintf("The value of the angle of tilt:");
fprintf('\n');
disp(0);

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

fprintf("The value of G1 times fc_1:");
fprintf('\n');
disp(G1*fc_1);

% Tau_1_before = J_analytical'*fc_1;
% Tau_2_before = J_analytical'*fc_2;

%% Grasping Force Optimization Formulation by implementing the torque constraints and Torque Minimization objective
cvx_begin
    cvx_precision high
%   Declaring the optimization variables for our problem
    variable T(n)
    variable fc_1(m,n)
    variable fc_2(m,n)
    variable Tau_1(7,1)
    variable Tau_2(7,1)
    variable f_E1(m,n) 
    variable f_E2(m,n)
%   As our objective is to minimize the maximum torque required we will be
%   minimizing the scalar value T.
    minimize T
%   Constraints
    subject to
%   The primary wrench balance condition:
        G*[fc_1;fc_2] + F_external + Ad_OE1*f_E1 + Ad_OE2*f_E2  == 0;
   
%       Extracting the first two components from the contact wrench for
%       ease in computation.
        fc1 = fc_1(1:2);
        fc2 = fc_2(1:2);
        
%%%%%%%%%%%% Friction constraints at robot-object contacts%%%%%%%%%%%%%%%%
% Fingers can only push not pull
        fc_1(3) >= 0;
        fc_2(3) >= 0;

% No moments about the tangential axis at the cotacts
        fc_1(4) == 0;fc_2(4) == 0;
        fc_1(5) == 0;fc_2(5) == 0;
       
        
% Isotropic Soft Finger Contact Friction model
        norm(fc1) <= mu*fc_1(3);
        norm(fc2) <= mu*fc_2(3);
        norm(fc_1(6)) <= sigma*fc_1(3);
        norm(fc_2(6)) <= sigma*fc_2(3);
        
%       Point Contact with friction model
%       fc_1(6) == 0;fc_2(6) == 0;
        
% Anisotropic Soft Finger Contact Friction model
%         f_temp1 = [fc1(1)/e11;fc1(2)/e12; fc_1(6)/e1r];
%         norm(f_temp1) <= mu*fc_1(3);
%         f_temp2 = [fc2(1)/e21;fc2(2)/e22; fc_2(6)/e2r];
%         norm(f_temp2) <= mu*fc_2(3);

% The normal force at the contact frame should less than the scalar value
% 'F' which is being minimized.
%         fc_1(3) <= F;
%         fc_2(3) <= F;
        
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
         
%%%%%%%%%%%%%Torque Constraints for the Baxter arm%%%%%%%%%%%%%%%%%%%%%%%%
% Computing the Torques in the left arm of Baxter required to produce the 
% necessary squeezing force at the contact points.
        Tau_1 == J_analytical'*G_TC*fc_1;
        
% Implementing the torque limits as the torque constraints on the Baxter
% joint torques.
        Torque_limits_min <= Tau_1 <= Torque_limits_max;
        max(abs(Tau_1)) <= T;
        
%         Tau_2 == left_spatial_jac'*fc_2;
%         Torque_limits_min <= Tau_2 <= Torque_limits_max;
%         Tau_2 <= T;

cvx_end