%This function can be used to compute the Grasp map for a single finger
%using the wrench basis for soft point contact
%In our updated function for the grasp map we will be considering all the
%six components of a the contact wrench and therefore the dimensions of the
%wrench basis have been updated.

function G = GraspMap(R, p_hat, x)
    if x == 'SF'
%       The identity matrix of the required dimensions
        I = eye(3,5);
        Z = zeros(3,5);
        z_small = zeros(3,1);
        e_z = [0;0;1];
        
%       This is the wrench basis for the soft finger contact wherein all
%       the dimension of the wrench is preserved to 6 * 1.
        B_c = [I, z_small;
               Z, e_z];
        
%         B_c = [1,0,0,0
%               0,1,0,0;
%               0,0,1,0;
%               0,0,0,0;
%               0,0,0,0;
%               0,0,0,1];
         Adjoint = [R      , zeros(3);
                    p_hat*R, R];
           
         G = Adjoint*B_c;
    elseif x == 'PF'
        B_c = [1,0,0;
              0,1,0;
              0,0,1;
              0,0,0;
              0,0,0;
              0,0,0];
         Adjoint = [R      , zeros(3);
                    p_hat*R, R];
           
         G = Adjoint*B_c;
    end
end