function [g_st, J_spatial] = getSpatialJacobian(theta, w, q, g_st_0)
    [~,num_joints] = size(w);
    dim = 3;
    for i = 1:num_joints
        eta(:,i) = GetTwist(w(:,i),q(:,i));
        exp_twist_theta(:,:,i) = GetExponential(w(:,i), theta(i), q(:,i));
    end
    
    g1(:,:,1) = eye(4);

    for i = 2:num_joints
        g1(:,:,i) = g1(:,:,i-1)*exp_twist_theta(:,:,i-1);
    end
    
    for i = 1:num_joints
        Adjoint(:,:,i) = GetAdjoint(g1(:,:,i));
    end
    J_spatial = zeros(dim+3, num_joints);
    J_spatial(:,1) = eta(:,1);
    for i = 2:num_joints
        J_spatial(:,i) = Adjoint(:,:,i)*eta(:,i);
    end
    
    g_st(:,:,1) = exp_twist_theta(:,:,1);
    for i = 2:num_joints
        g_st(:,:,i) = g_st(:,:,i-1)*exp_twist_theta(:,:,i);
    end
    
    for i = 1:num_joints
        g_st(:,:,i) = g_st(:,:,i)*g_st_0;
    end

end