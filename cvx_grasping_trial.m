R_1 = [0,1,0;
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

%Contact forces considering point 
%contact with friction
%Get the values of the contact force vectors directly
%from the user
fc_1 = ['fx_1';
        'fy_1';
        'fz_1'];

fc_2 = ['fx_2';
        'fy_2';
        'fz_2'];

%Concatenated force vector
B = [fc_1; fc_2];

%Force equilibrium condition for grasping
F_external = -G*B;

%Specifying the friction coefficient
f_coeff = 0.1;

disp(B);

%Getting the force F as expressed in the conic 
%formulation of the problem in Boyd's paper.

F = ['F_x';
     'F_y';
     'F_z'];
 
%Writing the optimisation code
cvx_begin
    variable F
    minimize(norm(F))
cvx_end






