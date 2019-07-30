% This function is used to select the coordinates of points and the 
% components of the normals given the  number of points on one side of the
% object to be grasped, the array of points and the array of surface
% normals,



function [points_1, points_2, normals_1, normals_2] = selectPoints(array_points, array_normals, step_size)
    points_1 = array_points(1:step_size,:);
    points_2 = array_points((step_size*10)+1:(step_size*11),:);
    
    normals_1 = array_normals(1:step_size,:);
    normals_2 = array_normals((step_size*10)+1:(step_size*11),:);
end