%% Box tilting application where the angle of tilt is zero.
close all;
clear all;
clc;

c  = 'SF';

data = getData('Box_1.txt');
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

% Storing the data in multidimensional arrays for the ease of operation.
R_OC(:,:,1) = R_OC1;
R_OC(:,:,2) = R_OC2;

R_OE(:,:,1) = R_OE1;
R_OE(:,:,2) = R_OE2;

p_OE(:,:,1) = p_OE1;
p_OE(:,:,2) = p_OE2;

p_OC(:,:,1) = p_OC1;
p_OC(:,:,2) = p_OC2;

% 
F_external = [0;0;-20;0;0;0];

% Creating the box object of the required properties and applying the
% methods of the class to the properties.
B = BOX(theta, R_OC, R_OE, p_OC, p_OE);
p_OC_hat = B.skewSymmetric(B.p_OC);
G_multi = B.graspMap(B.R_OC, p_OC_hat, c);
Adjoint = B.getAdjoint(B.R_OE, B.p_OE);

G1 = G_multi(:,:,1);
G2 = G_multi(:,:,2);
G = [G1,G2];

Ad_OE1 = Adjoint(:,:,1);
Ad_OE2 = Adjoint(:,:,2);

%% Grasping force optimization formulation for the box tilting application
cvx_begin
    cvx_precision high
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
     
        norm(fc1) <= mu*fc_1(3);
        norm(fc2) <= mu*fc_2(3);
     
        fc_1(3) >= 0;
        fc_2(3) >= 0;
        fc_1(4) == 0;fc_2(4) == 0;
        fc_1(5) == 0;fc_2(5) == 0;
%         fc_1(6) == 0;fc_2(6) == 0;
     
        norm(fc_1(6)) <= sigma*fc_1(3);
        norm(fc_2(6)) <= sigma*fc_2(3);
 
        fc_1(3) <= F;
        fc_2(3) <= F;
        
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

%% Box tilting application formualtion where the angle of tilt is greater than zero.

close all;
clear all;
clc;

c  = 'SF';

data = getData('Box_2.txt');
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

% Storing the data in multidimensional arrays for the ease of operation.
R_OC(:,:,1) = R_OC1;
R_OC(:,:,2) = R_OC2;

% Getting the rotation matrices between the two reference frames at the
% edge and the object reference frame, when the box is tilted at an angle
% 'theta' from the surface.
R_OE(:,:,1) = EulerRotation(0,theta,0);
R_OE(:,:,2) = EulerRotation(0,theta,0);

% Obtaining the position vectors between the object frame and the edge
% frame when the box is tilted at an angle 'theta' from the surface.
p_OE(:,:,1) = [p_OE1(1,1)*cos(deg2rad(theta)); p_OE1(2,1);p_OE1(3,1)*cos(deg2rad(theta))];
p_OE(:,:,2) = [p_OE2(1,1)*cos(deg2rad(theta)); p_OE2(2,1);p_OE2(3,1)*cos(deg2rad(theta))];

p_OC(:,:,1) = p_OC1;
p_OC(:,:,2) = p_OC2;

F_external = [0; 0; -20*cos(deg2rad(theta)); 0; 0; 0];

% Creating the box object of the required properties and applying the
% methods of the class to the properties.
B = BOX(theta, R_OC, R_OE, p_OC, p_OE);
p_OC_hat = B.skewSymmetric(B.p_OC);
G_multi = B.graspMap(B.R_OC, p_OC_hat, c);
Adjoint = B.getAdjoint(B.R_OE, B.p_OE);

G1 = G_multi(:,:,1);
G2 = G_multi(:,:,2);
G = [G1,G2];

Ad_OE1 = Adjoint(:,:,1);
Ad_OE2 = Adjoint(:,:,2);

%% Grasping force optimization formulation for the box tilting application
cvx_begin
    cvx_precision high
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
     
        norm(fc1) <= mu*fc_1(3);
        norm(fc2) <= mu*fc_2(3);
     
        fc_1(3) >= 0;
        fc_2(3) >= 0;
        fc_1(4) == 0;fc_2(4) == 0;
        fc_1(5) == 0;fc_2(5) == 0;
%       fc_1(6) == 0;fc_2(6) == 0;
     
        norm(fc_1(6)) <= sigma*fc_1(3);
        norm(fc_2(6)) <= sigma*fc_2(3);
   
        fc_1(3) <= F;
        fc_2(3) <= F;
        
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
