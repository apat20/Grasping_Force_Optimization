% Generate spherical point cloud.
% In our case the center is [0,0,0]

numFaces = 20;
[x,y,z] = sphere(numFaces);
[m,n] = size(x);
array_new = reshape([x,y,z],[(m)*(n),3]);
ptCloud = pointCloud(array_new);
% figure;
pcshow(ptCloud);
hold on
% pcwrite(ptCloud, 'sphere.pcd');
% title('Sphere with Default Color Map');
% xlabel('X');
% ylabel('Y');
% zlabel('Z');


% Generating the surface normals for matlab have the vector components.
normals = pcnormals(ptCloud);

% Plotting the normals on the surface of the generated the sphere.
x = ptCloud.Location(1:end, 1);
y = ptCloud.Location(1:end, 2);
z = ptCloud.Location(1:end, 3);
u = normals(1:end,1);
v = normals(1:end,2);
w = normals(1:end,3);

quiver3(x,y,z,u,v,w);
hold off


