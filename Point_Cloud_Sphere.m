% Generate spherical point cloud.
% In our case the center is [0,0,0]

close all;
clear;
clc;

numFaces = 50;
[x,y,z] = sphere(numFaces);
[m,n] = size(x);
array_new = reshape([x+30,y+30,z+30],[(m)*(n),3]);
ptCloud = pointCloud(array_new);
% figure;
% pcshow(ptCloud);
% hold on
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

% quiver3(x,y,z,u,v,w);
% hold off

Center = [30,30,30];
% Adjusting the normals which are not pointing in the same direction.
for k = 1 : numel(x)
   p1 = Center - [x(k),y(k),z(k)];
   p2 = [u(k),v(k),w(k)];
   % Flip the normal vector if it is not pointing away and outward from the
   % center
   angle = atan2(norm(cross(p1,p2)),p1*p2');
%  We change all the normals to pointing outwards, if we want them to point
%  inwards change the conditional to 'angle > pi/2'.
   if angle <= pi/2 || angle <= -pi/2
       u(k) = -u(k);
       v(k) = -v(k);
       w(k) = -w(k);
   elseif angle == pi/2
       R=[cosd(pi/2) -sind(pi/2); sind(pi/2) cosd(pi/2)];
       u(k) = u(k)*R';
       v(k) = v(k)*R';
       w(k) = w(k)*R';
   end
end

% All normals pointing outwards from the center.
quiver3(x,y,z,u,v,w);
hold off

