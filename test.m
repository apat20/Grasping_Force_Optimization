% r=1
% teta=-pi:0.01:pi
% x=r*cos(teta);
% y=r*sin(teta)
% plot3(x,y,zeros(1,numel(x)))

% Define my 2 circles
rng default
center1 = randn(1,3);
radius1 = .25;
center2 = randn(1,3);
radius2 = 1.25;
% Create 3 orthonormal vectors. The first goes from center1 to center2.
v1 = center2 - center1;
v1 = v1 / norm(v1);
v2 = cross(v1+randn(1,3),v1);
v2 = v2 / norm(v2);
v3 = cross(v1,v2);
v3 = v3 / norm(v3);
% Create 2x32 arrays of points.
ang = linspace(0,2*pi,32);
x = [center1(1) + radius1*(v2(1)*cos(ang) + v3(1)*sin(ang)); ...
     center2(1) + radius2*(v2(1)*cos(ang) + v3(1)*sin(ang))];
y = [center1(2) + radius1*(v2(2)*cos(ang) + v3(2)*sin(ang)); ...
     center2(2) + radius2*(v2(2)*cos(ang) + v3(2)*sin(ang))];
z = [center1(3) + radius1*(v2(3)*cos(ang) + v3(3)*sin(ang)); ...
     center2(3) + radius2*(v2(3)*cos(ang) + v3(3)*sin(ang))];
surf(x,y,z,'FaceColor','yellow')