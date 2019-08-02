% This function is used to calculate the transformation matrix from the
% base frame of the manipulator to the end effector frame.

function g_st = getTransform(theta,omega,q, g_zero)
    [~,~,num_joints] = size(omega);

    for i=1:num_joints
        eta(:,i) = GetTwist(omega(:,i),q(:,i));
        exp_twist_theta(:,:,i) = GetExponential(omega(:,i), theta(i), q(:,i));
    end
 
    g_st(:,:,1) = exp_twist_theta(:,:,1);
    for i = 2:num_joints
        g_st(:,:,i) = g_st(:,:,i-1)*exp_twist_theta(:,:,i);
    end
    
    for i = 1:num_joints
        g_st(:,:,i) = g_st(:,:,i)*g_zero;
    end

end