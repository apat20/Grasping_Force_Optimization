%Function to get the rotation matrix using the three parameter
%representation
%theta is angle of rotation
%axis is a vector representing the axis of rotation 

function rotation_matrix = RotationMatrix(theta,axis)
    if axis == 'x'
        rotation_matrix = 'The axis is X';
        disp(rotation_matrix);
    elseif axis == 'y'
        rotation_matrix = 'The axis is Y';
        disp(rotation_matrix);
    elseif axis == 'z'
        rotation_matrix = 'The axis is Z';
        disp(rotation_matrix);
    else
        rotation_matrix = 'No axis specified';
        disp(rotation_matrix, theta);
    end 
end
