%This code implements the Conic Formulation described by Stephen Boyd in
%his paper 'Fast Computation of Optimal Contact Forces'
%Here we have used just the grasp map to encode the contact information.


clear;
clc;

%%
fprintf('Grasping Force Optimisation using CVX');
fprintf('\n');

%The rotation matrices for our selected problem
R_1 = [0,1,0;
       0,0,1;
       1,0,0];
R_2 = [1,0,0;
       0,0,-1;
       0,1,0];

%The position vectors in the skew symmetric form for our selected problem
p1_hat = [0,0,2;
          0,0,0;
         -2,0,0];
p2_hat = [0,0,-2;
          0,0,0;
          2,0,0];


prompt = 'Enter the type of contact:';
x = input(prompt, 's');
if x == 'SF'
    fprintf('The type of contact entered is soft fingered contact!');
    fprintf('\n');
    G_1 = GraspMap(R_1, p1_hat, x);
    G_2 = GraspMap(R_2, p2_hat, x);
    G = [G_1,G_2];
    fprintf('The grasp map for the soft finger contact:');
    fprintf('\n');
    %disp(G)
elseif x == 'PF'
    fprintf('The type of contact entered is point contact with fingers!');
    fprintf('\n');
    G_1 = GraspMap(R_1, p1_hat, x);
    G_2 = GraspMap(R_2, p2_hat, x);
    G = [G_1,G_2];
    fprintf('The grasp map for the point contact with friction:');
    fprintf('\n');
    %disp(G)
end
disp(G);



%%
F_external = [0;0;-10;0;0;0];
fprintf('The external force acting on the object is:');
fprintf('\n');
disp(F_external)

m = 3; n = 1;

fprintf('The friction coefficient is:');
fprintf('\n')
mu = 0.1;
disp(mu);

cvx_begin
    cvx_precision high
    variable F(n)
    variable fc_1(m,n)
    variable fc_2(m,n)
    minimize F
    subject to
        G*[fc_1;fc_2] + F_external == 0;
        %Extracting the tangential components in a separate vector
        fc1 = fc_1(1:2);
        fc2 = fc_2(1:2);
        norm(fc1) <= mu*fc_1(3);
        norm(fc2) <= mu*fc_2(3);
        norm(fc_1) <= F;
        norm(fc_2) <= F;
cvx_end

