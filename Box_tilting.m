%% Code for reading and evaluating Box tilting scenario Case 2

close all;
clear all;
clc;
% name_left = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2'];
% name_right = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2'];

% Selecting the type of contact model
c  = 'SF';

% Reading the required data from the text file.
data = getData('BOX_Case_2.txt');
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
%% Case 2: Box tilting at a particular point. Two different tilt angles, 'alpha' and 'beta', given respectively 
% The box tilts at angle 'beta' about the X axis of the object and the edge
% reference frames.
x_OE_beta = [1;0;0];
y_OE_beta = [0;cos(deg2rad(beta));sin(deg2rad(beta))];
z_OE_beta = [0;-sin(deg2rad(beta));cos(deg2rad(beta))];

R_OE_beta = [x_OE_beta,y_OE_beta,z_OE_beta];

% The box tilts at angle 'alpha' about the Y axis of the object and the edge
% reference frames.
x_OE_alpha = [cos(deg2rad(alpha)); 0; sin(deg2rad(alpha))];
y_OE_alpha = [0;1;0];
z_OE_alpha = [-sin(deg2rad(alpha)); 0; cos(deg2rad(alpha))];

R_OE_alpha = [x_OE_alpha,y_OE_alpha,z_OE_alpha];

R_OE = R_OE_beta*R_OE_alpha;

p_OE1_tilt = R_OE'*p_OE1;

p_OC1_hat = skewSymmetric(p_OC1);
p_OC2_hat = skewSymmetric(p_OC2);

% Calculating the grasp map:
G_1 = GraspMap(R_OC1, p_OC1_hat, c);
G_2 = GraspMap(R_OC2, p_OC2_hat, c);

G = [G_1,G_2];

g_OE = [R_OE, p_OE1_tilt;
         zeros(1,3), 1];
Ad_OE1 = GetAdjointWrench(g_OE);


G_TC = [eye(3), zeros(3);
        zeros(3), eye(3)];
    
F_external = [20*sin(deg2rad(alpha));0;-20*cos(deg2rad(alpha));0;0;0];

%% Computing the Jacobian for Baxter left and right arms.
num_joints = numel(theta_l);

% Define rotation axes (wrt. arm_mount frame for both the arms)
w4 = w2; w5 = w3; w6 = w2; w7 = w3;
wr = [w1, w2, w3, w4, w5, w6, w7];

% Frame origins (wrt. arm_mount frame for both the arms)
qr = [q1, q2, q3, q4, q5, q6, q7];

% Transform joint axes and origins into base frame
wr_l = g_base_left(1:3, 1:3) * wr; 
qr_l = g_base_left(1:3, 1:3) * qr + g_base_left(1:3, 4);
gst0_l = g_base_left * g_st0_left;

wr_r = g_base_right(1:3, 1:3) * wr;
qr_r = g_base_right(1:3, 1:3) * qr + g_base_right(1:3, 4);
gst0_r = g_base_right * g_st0_right;

% Calculating the Jacobian for the left arm of the Baxter robot.
% [left_spatial_jac,gl] = baxter_spatial_jacobian_mat(qr_l, wr_l, theta, gst0_l);
[g_st_left, J_spatial_left] = getSpatialJacobian(theta_l, wr_l, qr_l, gst0_l);

% Calculating the Jacobian for the right arm of the Baxter robot.
[g_st_right, J_spatial_right] = getSpatialJacobian(theta_r, wr_r, qr_r, gst0_r);

% Calculating the Analytical Jacobian for the left arm
g_left = g_st_left(:,:,num_joints);
p_left = g_left(1:3,4);
J_analytical_left = AnalyticalJacobian(J_spatial_left, p_left);

% Calculating the Analytical Jacobian for the right arm
g_right = g_st_right(:,:,num_joints);
p_right = g_right(1:3,4);
J_analytical_right = AnalyticalJacobian(J_spatial_right, p_right);


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
%         G_new*[fc_1;fc_2] + F_external == 0;
        G*[fc_1;fc_2] + F_external + Ad_OE1*f_E1  == 0;

            
%       Extracting the first two components from the contact wrench for
%       ease in computation.
        fc1 = fc_1(1:2);
        fc2 = fc_2(1:2);
        
%%%%%%%%%%%% Friction constraints at robot-object contacts%%%%%%%%%%%%%%%%
% Anisotropic Soft Finger Contact Friction model
        f_temp1 = [fc1(1)/e11,fc1(2)/e12, fc_1(6)/e1r];
        norm(f_temp1) <= mu1*fc_1(3);
        f_temp2 = [fc2(1)/e21;fc2(2)/e22; fc_2(6)/e2r];
        norm(f_temp2) <= mu1*fc_2(3);
% Isotropic Soft Finger Contact Friction model
%         norm(fc1) <= mu1*fc_1(3);
%         norm(fc2) <= mu1*fc_2(3);

% Fingers can only push not pull
        fc_1(3) >= 0
        fc_2(3) >= 0;
       
% No moments about the tangential axis at the cotacts
        fc_1(4) == 0;fc_2(4) == 0;
        fc_1(5) == 0;fc_2(5) == 0;
        
%         norm(fc_1(6)) <= sigma*fc_1(3);
%         norm(fc_2(6)) <= sigma*fc_2(3);

%       Point Contact with friction model
%       fc_1(6) == 0;fc_2(6) == 0;
        

% The normal force at the contact frame should less than the scalar value
% 'F' which is being minimized.
        fc_1(3) <= F;
        fc_2(3) <= F;
        
%       Implementing the static friction constraints on the edge wrenches
%       generated at the edge
        f_e1 = f_E1(1:2);
        
        norm(f_e1) <= mu2*f_E1(3)
        f_E1(3) > 0;
        
%       Implementing the moment constraints for the edge wrenches generated
%       at the edge.
        f_E1(4) == 0;
        f_E1(6) == 0;
        f_E1(5) == 0;
        

%%%%%%%%%%%%%Torque Constraints for the Baxter arm%%%%%%%%%%%%%%%%%%%%%%%%
         
% % Computing the Torques in the left arm of Baxter required to produce the 
% % necessary squeezing force at the contact points.
         Tau_left = J_analytical_left'*(G_TC*fc_2);
         Tau_right = J_analytical_right'*(G_TC*fc_1);
%        
% % Implementing the torque limits as the torque constraints on the Baxter
% % joint torques.
        Torque_limits_min <= Tau_right <= Torque_limits_max;
        Torque_limits_min <= Tau_left <= Torque_limits_max;

cvx_end