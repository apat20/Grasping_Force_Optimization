theta_1 = 20;
theta_2 = 30;
omega = [0;0;1];

I = eye(3);
q_1 = [0;0;0];
q_2 = [-20*sin(deg2rad(20));20*cos(deg2rad(20));0];
q_3 = [-20*sin(deg2rad(theta_1)) - 20*sin(deg2rad(theta_1) + deg2rad(theta_2));
    20*cos(deg2rad(theta_1)) + 20*cos(deg2rad(theta_1) + deg2rad(theta_2));0];
g_zero = [I , q_3 ;zeros(1,3) , 1];
%disp(g_zero);

e_1 = [cos(deg2rad(theta_1)), -sin(deg2rad(theta_1)),0;
       sin(deg2rad(theta_1)), cos(deg2rad(theta_1)), 0;
       0, 0, 1];
g_1 = [e_1, (eye(3)-e_1)*q_1;
        zeros(1,3),1];
    
%disp(g_1);

g_2 = [e_1, (eye(3)-e_1)*q_2;
        zeros(1,3),1];

%disp(g_2);

g_1_1 = g_1*g_zero;
g_1_2 = g_1*g_2*g_zero;
disp(g_1_1);
disp(g_1_2);

R_1 = g_1_1(1:3,1:3);
R_2 = g_1_2(1:3,1:3);
p_1 = g_1_1(1:3,4);
p_2 = g_1_2(1:3,4);
disp(p_1);
p_1_hat = [0,-p_1(3),p_1(2);
           p_1(3),0,-p_1(1);
           -p_1(2),p_1(1),0];
fprintf("The skew symmetric form");
fprintf("\n");
disp(p_1_hat);
p_2_hat = [0,-p_2(3),p_2(2);
           p_2(3),0,-p_2(1);
           -p_2(2),p_2(1),0];
fprintf("The skew symmetric form");
fprintf("\n");
disp(p_2_hat);
      
Adjoint_g_1 = [R_1, p_1_hat*R_1;
               zeros(3,3), R_1];
disp(Adjoint_g_1);
eta_1 = [-cross(omega,q_1); omega];
eta_2 = [-cross(omega,q_2);omega];
disp(eta_2);
eta_3 = [-cross(omega,q_3); omega];
eta_2_dash = Adjoint_g_1*eta_2;
fprintf('eta_2_dash');
fprintf('\n');
disp(eta_2_dash);