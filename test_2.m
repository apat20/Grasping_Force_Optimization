theta = 30;
q = [0;0;0];
omega = [0;0;1];
e_zeta1_theta1 = [cos(deg2rad(theta)),-sin(deg2rad(theta)),0,0;
                  sin(deg2rad(theta)), cos(deg2rad(theta)),0,0;
                  0,0,1,0;
                  0,0,0,1];

disp(e_zeta1_theta1);

exp_twist = GetExponential(omega,theta,q);
disp(exp_twist);

