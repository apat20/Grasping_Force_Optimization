clear
clc
r = 2;
b = 6;
a = 4;
theta_1 = 20;
theta_2 = 30;
l_1 = 20;
l_2 = 20;


G = [0,1,0,0,1,0,0,0;
    0,0,1,0,0,0,-1,0;
    1,0,0,0,0,1,0,0;
    -r,0,0,0,0,r,0,0;
    0,0,0,1,0,0,0,-1;
    0,r,0,0,-r,0,0,0
    ];
fprintf('The grasp matrix is:');
fprintf('\n');
disp(G);

%wrench basis
B_c = [1,0,0,0;
    0,1,0,0;
    0,0,1,0;
    0,0,0,0;
    0,0,0,0;
    0,0,0,1];

%Calculating the Spatial Jacobian for the given 
%2-R manipulator.

J_spatial = [0, l_1*cos(theta_1), l_1*cos(theta_1) + l_2*cos(theta_1+theta_2),0;
            0,  l_1*sin(theta_1), l_1*sin(theta_1) + l_2*sin(theta_1+theta_2),0;
            0, 0, 0, 1;
            0, 0, 0, 0;
            0, 0, 0, 0;
            0, 0, 0, 1
    ];

R_1 = [0,1,0;
      0,0,1;
      1,0,0];
p_1 = [0;b-r;a];

R_2 = [1,0,0;
    0,0,-1;
    0,1,0];
p_2 = [0;-b+r;a];

R_p_hat_1 = [b-r,0,0;
            0,a,r-b;
            -a,0,0]; 
R_p_hat_2 = [0,a,b-r;
            r-b,0,0;
            a,0,0];

Adjoint_1 = [R_1.', R_p_hat_1;
            zeros(3), R_1.'];
Adjoint_2 = [R_2.', R_p_hat_2;
            zeros(3), R_2.'];
        
%disp(Adjoint_1);
        
J_h_1 = B_c.'*Adjoint_1*J_spatial;
J_h_2 = B_c.'*Adjoint_2*J_spatial;

%disp(J_h_1);

Hand_Jacobian = [J_h_1,zeros(4);
                 
zeros(4) , J_h_2];
fprintf('The Hand Jacobian matrix is:');
fprintf('\n');
disp(Hand_Jacobian);

%Performing SVD on the Hand Jacobian Transpose

[U,S,V] = svd(Hand_Jacobian.');
s = diag(S);
rank_jacobian = nnz(s);

%Rank of the Hand Jacobian transpose
fprintf('The rank is:');
fprintf('\n');
disp(rank_jacobian);

%Null space of the Hand Jacobian transpose
%null_space_ = null(Hand_Jacobian.');
%fprintf('The null space of the Hand Jacobian is:')
%fprintf('\n');
%disp(null_space_);

%Null space basis of the Hand Jacobian transpose
null_space_basis = V(:,~s);
fprintf('The null space basis: ');
fprintf('\n');
disp(null_space_basis);

