% This file is used to read the data and process it into the necessary
% format for computation purposes.


% Opening the file which contains the data for two fingers each made of a 
% SCARA manipulator grasping a box at two contact locations.
fileID = fopen('SCARA_2finger_box.txt');
% N = 2;
formatSpec = '%s';

% Read data into a single cell array wherein one element contains the enter
% line.
data = textscan(fileID, formatSpec,'Delimiter', '\n');
fclose(fileID);

% Processing the data for computation purpose.
% Each element of the cell 'data' is split and stores in another cell
% array of size m*n.
% Each row of 'C' contains the split string.

% In the for loop we assign a value to a varaible which is then used for
% our processing purposes.
% The variable names are stored in the first column of the cell array 'C'
% whereas the values to be assigned are stored in the third column.
[m,n] = size(data{1});
C = {};
for i=1:m
    C{i} = strsplit(data{1}{i}, ' ');
    assignin('base', C{i}{1}, str2num(C{i}{3}));
end