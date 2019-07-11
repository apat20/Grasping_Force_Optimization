% Generating the shape of a light bulb.
a=0.255/0.511;  
r=2.8179;       
vo=1;           
m=0.511; 
F = @(z)r^2/2.*((vo./(1+a.*(1-cos(z))))./vo).^2.*...
    (vo./(vo./(1+a.*(1-cos(z))))+...
    (vo./(1+a.*(1-cos(z))))./vo-sin(z).^2);

t = linspace(0,2*pi,55); 
z = linspace(0,pi,55); 
[T,U] = meshgrid(t,z); 

X = F(U).*sin(U).*cos(T); 
Y = F(U).*sin(U).*sin(T); 
Z = F(U).*cos(U); 

[i,j] = size(X);

array_new = reshape([X,Y,Z],[i*j,3]);
ptCloud = pointCloud(array_new);
pcshow(ptCloud);
hold on

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

Center = [0,0,0];

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