% This function is used to calculate the transformation matrix from the
% base frame of the manipulator to the end effector frame.

function g_st = getTransform(theta,omega,q, P_base)
    [~,~,x] = size(omega);
    I = eye(3);

    for i=1:x
        eta(:,:,i) = GetTwist(omega(:,:,i),q(:,:,i));
        exp_twist_theta(:,:,i) = GetExponential(omega(:,:,i), theta(i), q(:,:,i));
    end
 
    g_zero = [I, P_base;
    zeros(1,3),1];
    
    g_st(:,:,1) = exp_twist_theta(:,:,1);
    for i = 2:x
        g_st(:,:,i) = g_st(:,:,i-1)*exp_twist_theta(:,:,i);
    end
    
    for i = 1:x
        g_st(:,:,i) = g_st(:,:,i)*g_zero;
    end

end