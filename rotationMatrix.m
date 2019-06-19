%Function to get the rotation matrix given the axis of rotation expressed
%as a string and the angle of rotation expressed in degrees.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% INPUT ARGUMENTS %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 'theta' is angle of rotation expressed here in degrees. The angle is then
%converted into radians in the function.

% 'axis' is a string representing the axis of rotation. The rotation matirces
%depend on the specification of this axis.
% The input argument for 'axis' is a string specifying the axis of
% rotation. (e.g x,y,z)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% OUTPUT %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 'rotation_matrix' which contains the rotation information about a
% particular axis encoded in the matrix.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% CODE %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function rotation_matrix = rotationMatrix(theta,axis)
    if axis == 'x'
        rotation_matrix = [1,                   0,                    0;
                           0, cos(deg2rad(theta)), -sin(deg2rad(theta));
                           0, sin(deg2rad(theta)), cos(deg2rad(theta))];
    elseif axis == 'y'
        rotation_matrix = [cos(deg2rad(theta)), 0, sin(deg2rad(theta));
                           0,                  1,                   0;
                          -sin(deg2rad(theta)), 0, cos(deg2rad(theta))];
%         disp(rotation_matrix);
    elseif axis == 'z'
        rotation_matrix = [cos(deg2rad(theta)), -sin(deg2rad(theta)), 0;
                           sin(deg2rad(theta)), cos(deg2rad(theta)),  0;
                                             0,                   0, 1];
%         disp(rotation_matrix);
    else
        rotation_matrix = 'No axis specified';
%         disp(rotation_matrix, theta);
    end 
end
