m = 3; n = 1;

R_1 = [0,1,0;
       0,0,1;
       1,0,0];
R_2 = [1,0,0;
       0,0,-1;R_1 = [0,1,0;
       0,0,1;
       1,0,0];
R_2 = [1,0,0;
       0,0,-1;
       0,1,0];
p1_hat = [0,0,2;
          0,0,0;
         -2,0,0];
p2_hat = [0,0,-2;
          0,0,0;
          2,0,0];

%Calculating the grasp map.
G = [R_1, R_2;
    p1_hat*R_1, p2_hat*R_2];

disp(G);
       0,1,0];
p1_hat = [0,0,2;
          0,0,0;
         -2,0,0];
p2_hat = [0,0,-2;
          0,0,0;
          2,0,0];

%Calculating the grasp map.
G = [R_1, R_2;
    p1_hat*R_1, p2_hat*R_2];

disp(G);

%Contact forces considering point 
%contact with friction
%Get the values of the contact force vectors directly
%from the user
fx_1 = 0.1; fy_1 = 0.1; fz_1 = 1.5;
fx_2 = 0.2; fy_2 = 0.2; fz_2 = 3;

fc_1 = [fx_1;
        fy_1;
        fz_1];

fc_2 = [fx_2;
        fy_2;
        fz_2];

%Concatenated force vector
B = [fc_1; fc_2];

%Force equilibrium condition for grasping
F_external = -G*B;

%Specifying the friction coefficient
f_coeff = 0.1;

disp(B);

%Getting the force F as expressed in the conic 
%formulation of the problem in Boyd's paper.

 
%Writing the optimisation code
cvx_begin
    variable F(n)
    variable fc_1(m,n)
    variable fc_2(m,n)
    minimize F
    subject to
        Beta = pinv(G)*-F_external;
        fc_1 = Beta(1:3);
        fc_2 = Beta(4:6);
        sqrt(fc_1(1)^2 + fc_1(2)^2) <= f_coeff*fc_1(3);
        sqrt(fc_2(1)^2 + fc_2(2)^2) <= f_coeff*fc_2(3);
        norm(fc_1) <= F;
        norm(fc_2) <= F;
cvx_end

cvx_begin
    variable F(m,n)
    variable fc_1(m,n)
    variable fc_2(m,n)
    minimize(norm(F))
    subject to
        Beta = pinv(G)*-F_external;
        fc_1 = Beta(1:3);
        fc_2 = Beta(4:6);
        sqrt(fc_1(1)^2 + fc_1(2)^2) <= f_coeff*fc_1(3);
        sqrt(fc_2(1)^2 + fc_2(2)^2) <= f_coeff*fc_2(3);
        norm(fc_1)<=norm(F);
        norm(fc_2)<=norm(F);
cvx_end

fprintf('Force vector ar finger contact 1:');
fprintf('\n');
disp(fc_1);

fprintf('Force vector ar finger contact 2:');
fprintf('\n');
disp(fc_2);







