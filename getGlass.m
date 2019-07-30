% This function is used to generate the point cloud of the glass of the
% required dimensions and return the coordinates of the data points and
% components of the surface normals.

function [ptCloud, array_new, array_normals, g_center, m, n] = getGlass(R2, max_step, h,res)
    
%   Plotting the glass of the required radius and step size
    t = 0:res:max_step;
    [X,Y,Z] = cylinder(R2+t);
    [m,n] = size(Z);
    R1 = R2 + max_step;

    % Increasing the height of the cone as it was of 'unit' height
    for i = 2:m
        Z(i,:) = Z(i,:)*h;
    end
    
    [Nx,Ny,Nz] = surfnorm(X,Y,Z);
    array_new = reshape([X,Y,Z],[m*n,3]);
    array_normals = reshape([Nx,Ny,Nz],[m*n,3]);
    
    % Converting the reshaped points forming the surface of our cylinder into a
    % point cloud.
    ptCloud = pointCloud(array_new);
    
    % Getting the geometric center of the glass using the frustum centroid
    % function
    g1 = frustumCentroid(R1, R2, h);
    g_center = [0,0,h-g1];
end