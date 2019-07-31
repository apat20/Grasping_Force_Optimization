% This file is used to read the data and process it into the necessary
% format for computation purposes.
 

% % Opening the file which contains the data for two fingers each made of a 
% % SCARA manipulator grasping a box at two contact locations


data = getData('SCARA_2finger_box.txt');
[m,n] = size(data{1});
C = {};


% In the for loop we assign a value to a varaible which is then used for
% our processing purposes.
% The variable names are stored in the first column of the cell array 'C'
% whereas the values to be assigned are stored in the third column.

for i=1:m

% Each element of the cell 'data' is split and stores in another cell
% array of size m*n.
% Each row of 'C' contains the split string.
    C{i} = strsplit(data{1}{i}, ' ');
    assignin('base', C{i}{1}, str2num(C{i}{3}));
end

theta = [theta_1;theta_2;theta_3];

omega(:,:,1) = omega_1;
omega(:,:,2) = omega_2;
omega(:,:,3) = omega_3;

q(:,:,1) = q_1;
q(:,:,2) = q_2;
q(:,:,3) = q_3;


