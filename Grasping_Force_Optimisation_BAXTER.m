% This is the code for the grasping force optimisation applied to the arms
% of BAXTER.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% VARIABLES %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 'data' is a cell array which is used to store the all the data read from
% the .txt file in the form of strings

% 'C' is a cell array which is used for storing the strings after they have
% been split using 'strsplit'. This cell is basically used for assigning
% values to the variable names after they have been split.

% 'q' is a multidimensional array that contains all the position vectors of an 
% arbitary point on the acis of rotation of the revolute joints.

% 'o' is a multidimensional array that contains all the Quaternion 
% representations of each joint reference frames of the BAXTER arm.

% 'theta' is vector of all the joint angles.

% 'omega' is a multidimensional array which contains the axis of rotation
% of each revolute joint.

% 'P_base' is a position vector of the base frame with respect to the end
% effector frame.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% CODE %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Reading the data for BAXTER from a text file
data = getData('BAXTER_data.txt');
[m,n] = size(data{1});
C = {};

for i=1:m
% Each element of the cell 'data' is split and stores in another cell
% array of size m*n.
% Each row of 'C' contains the split string.
    C{i} = strsplit(data{1}{i}, ' ');
    assignin('base', C{i}{1}, str2num(C{i}{3}));
end

%Forming a multidimensional array 'q' to store all the 'qi's'.
q(:,:,1) = q1; q(:,:,2) = q2; q(:,:,3) = q3; q(:,:,4) = q4;
q(:,:,5) = q5; q(:,:,6) = q6; q(:,:,7) = q7;

%Forming a multidimensional array 'o' to store all the 'oi's'
o(:,:,1) = o1; o(:,:,2) = o2; o(:,:,3) = o3; o(:,:,4) = o4;
o(:,:,5) = o5; o(:,:,6) = o6; o(:,:,7) = o7; o(:,:,8) = o8;

%Forming a multidimensional array 'w' to store all the 'wi's'
% w(:,:,1) = w1; w(:,:,2) = w2; w(:,:,3) = w3; w(:,:,4) = w4;
% w(:,:,5) = w5; w(:,:,6) = w6; w(:,:,7) = w7;

% Converting the quaternion representation into equivalent exponential
% coordinates representation.
[theta,omega] = quaternionToRotation(o);

% Create an instance of the BAXTER class of the required properties
B = BAXTER(theta,omega,q,P_base);

% Calculate the Spatial, Analytical and Body Jacobian using the methods
% defined for the BAXTER class.

[Adjoint, g, J_S] = spatialJacobian(B);
J_A = BAXTER.analyticalJacobian(J_S, g);
J_B = BAXTER.bodyJacobian(Adjoint, J_S);






