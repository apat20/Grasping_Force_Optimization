%This function can be used to compute the Grasp map for a single finger
%using the wrench basis for soft point contact

function G = GraspMap(R, p_hat, x)
    if x == 'SF'
        B_c = [1,0,0,0
              0,1,0,0;
              0,0,1,0;
              0,0,0,0;
              0,0,0,0;
              0,0,0,1];
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