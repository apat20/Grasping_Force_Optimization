%This function is used to calculate the rotation matrix using the fixed
%angle representation.This function can be used for both the Fixed Angle and
%Euler Angle representation. For our case however we will be implementing
%the X-Y-Z Fixed Angle representations.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% INPUT ARGUMENTS %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 'theta' is a vector of angles in degrees. Each component of the vector
% corresponds to the axis of rotation. The angles are store in the vector
% in the form [X,Y,Z].

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% OUTPUT %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 'rotation_matrix' according to the X-Y-Z fixed angle representation.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% CODE %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function rotation_matrix = fixedAngleXYZ(theta)

%   Getting the rotation around the X axis using the theta corresponding
%   to the angle of rotatio around the X axis.
    rotation_matrix_X = [1,                   0,                    0;
                           0, cos(deg2rad(theta(1))), -sin(deg2rad(theta(1)));
                           0, sin(deg2rad(theta(1))), cos(deg2rad(theta(1)))];
    
%   Rotation about the Y axis
    rotation_matrix_Y = [cos(deg2rad(theta(2))), 0, sin(deg2rad(theta(2)));
                           0,                  1,                   0;
                          -sin(deg2rad(theta(2))), 0, cos(deg2rad(theta(2)))];
                      
                      
%   Rotation about the Z axis
    rotation_matrix_Z = [cos(deg2rad(theta(3))), -sin(deg2rad(theta(3))), 0;
                           sin(deg2rad(theta(3))), cos(deg2rad(theta(3))),  0;
                                             0,                   0, 1];

%    Caculating the rotation matrix for a given set of angles using the
%    X-Y-Z fixed angle representation.
    rotation_matrix = rotation_matrix_Z*rotation_matrix_Y*rotation_matrix_X;
    
end