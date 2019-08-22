close all;
clear all;
clc;
% name_left = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2'];
% name_right = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2'];

% Selecting the type of contact model
c  = 'SF';

% Reading the required data from the text file.
data = getData('BOX_1.txt');
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

p_OC1_hat = skewSymmetric(p_OC1);
p_OC2_hat = skewSymmetric(p_OC2);

G_1 = GraspMap(R_OC1, p_OC1_hat, c);
G_2 = GraspMap(R_OC2, p_OC2_hat, c);

G = [G_1,G_2];

g_OE1 = [R_OE1, p_OE1_tilt;
         zeros(1,3), 1];
Ad_OE1 = GetAdjointWrench(g_OE1);

g_OE2 = [R_OE2, p_OE2_tilt;
         zeros(1,3), 1];
Ad_OE2 = GetAdjointWrench(g_OE2);

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
F_external = [20*sin(deg2rad(alpha));0;-20*cos(deg2rad(alpha));0;0;0];


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
        G*[fc_1;fc_2] + F_external + Ad_OE1*f_E1 + Ad_OE2*f_E2  == 0;

            
%       Extracting the first two components from the contact wrench for
%       ease in computation.
        fc1 = fc_1(1:2);
        fc2 = fc_2(1:2);
        
%%%%%%%%%%%% Friction constraints at robot-object contacts%%%%%%%%%%%%%%%%
% Anisotropic Soft Finger Contact Friction model
%         f_temp1 = [fc1(1)/e11,fc1(2)/e12, fc_1(6)/e1r];
%         norm(f_temp1) <= mu1*fc_1(3);
%         f_temp2 = [fc2(1)/e21;fc2(2)/e22; fc_2(6)/e2r];
%         norm(f_temp2) <= mu1*fc_2(3);
% Isotropic Soft Finger Contact Friction model
        norm(fc1) <= mu1*fc_1(3);
        norm(fc2) <= mu1*fc_2(3);
        norm(fc_1(6)) <= sigma*fc_1(3);
        norm(fc_2(6)) <= sigma*fc_2(3);

% Fingers can only push not pull
        fc_1(3) >= 0
        fc_2(3) >= 0;
       
% No moments about the tangential axis at the cotacts
        fc_1(4) == 0;fc_2(4) == 0;
        fc_1(5) == 0;fc_2(5) == 0;
        
%       Point Contact with friction model
%       fc_1(6) == 0;fc_2(6) == 0;
        

% The normal force at the contact frame should less than the scalar value
% 'F' which is being minimized.
        fc_1(3) <= F;
        fc_2(3) <= F;
        
%       Implementing the static friction constraints on the edge wrenches
%       generated at the edge
        f_e1 = f_E1(1:2);
        f_e2 = f_E2(1:2);
        
        norm(f_e1) <= mu2*f_E1(3)
        f_E1(3) > 0;
        
        norm(f_e2) <= mu2*f_E2(3);
        f_E2(3) > 0;
        
%       Implementing the moment constraints for the edge wrenches generated
%       at the edge.
        f_E1(4) == 0;f_E2(4) == 0;
        f_E1(6) == 0;f_E2(6) == 0;
        f_E1(5) == 0;f_E2(5) == 0;
cvx_end