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
F_external = [0;0;-10;0;0;0];
disp(F_external)

m = 3; n = 1;
f_coeff = 0.1;

cvx_begin
    cvx_precision high
    variable F(n)
    variable fc_1(m,n)
    variable fc_2(m,n)
    minimize F
    subject to
        G*[fc_1;fc_2] + F_external == 0;
        %Beta_1 = -Beta;
        fc1 = fc_1(1:2);
        fc2 = fc_2(1:2);
        norm(fc1) <= f_coeff*fc_1(3);
        norm(fc2) <= f_coeff*fc_2(3);
        norm(fc_1) <= F;
        norm(fc_2) <= F;
cvx_end

% cvx_begin
%     cvx_precision high
%   %  variable F(n)
%     variable fc_1(m,n)
%     variable fc_2(m,n)
%     minimize 0
%     subject to
%         G*[fc_1;fc_2] + F_external == 0;
%         %Beta_1 = -Beta;
%         fc1 = fc_1(1:2);
%         fc2 = fc_2(1:2);
%         norm(fc1) <= f_coeff*fc_1(3);
%         norm(fc2) <= f_coeff*fc_2(3);
%        % norm(fc_1) <= F;
%        % norm(fc_2) <= F;
% cvx_end




