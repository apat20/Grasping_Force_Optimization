% This is a script file to read the data for BAXTER arm configuration

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

[theta,omega] = quaternionToRotation(o);