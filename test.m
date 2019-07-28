theta_1 = 30;
theta_2 = 20;
theta_3 = 20;

l_1 = 20;
l_2 = 30;



J = [0, l_1*cos(deg2rad(theta_1)), l_1*cos(deg2rad(theta_1)) + l_2*cos(deg2rad(theta_1)+deg2rad(theta_2)), 0; 
     0, l_1*sin(deg2rad(theta_1)), l_1*sin(deg2rad(theta_1)) + l_2*sin(deg2rad(theta_1)+deg2rad(theta_2)), 0
     0, 0, 0, 1;
     0,0,0,0;
     0,0,0,0;
     1,1,1,0];
 
 
 0   17.3205   36.6041         0
         0   10.0000   32.9813         0
         0         0         0    1.0000
         0         0         0         0
         0         0         0         0
    1.0000    1.0000    1.0000         0
    
    I = eye(3);
    g_zero = [I, P_base;
             zeros(1,3),1];