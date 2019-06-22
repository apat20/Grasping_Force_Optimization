% Defining a class SCARA to be used for the SCARA manipulator.

classdef SCARA
    properties
        theta
        omega
        q
        P_base
    end
    methods

%   Defining a constructor method to construct the instances of class
%   SCARA:
        function S = SCARA(theta, omega, q, P_base)
            if nargin == 4
                if class(theta) == 'double' & class(omega) == 'double' & class(q) == 'double' & class(P_base) == 'double'
%                     if [3,1,3] == size(omega) & [3,1,3] == size(q) & [3,1] == size(theta) & [3,1] == size(P_base)
                        S.theta = theta;
                        S.omega = omega;
                        S.q = q;
                        S.P_base = P_base;
%                     end
                end
            end
        end
        
    
% Defining a function to calculate the Spatial Jacobian of a SCARA
%     manipulator.
        function J_spatial = spatialJacobian(S)
             [x,~,~] = size(S.omega);
             I = eye(3);
             g_zero = [I, S.P_base;
             zeros(1,3),1];
             for i=1:x
                eta(:,:,i) = GetTwist(S.omega(:,:,i),S.q(:,:,i));
                exp_twist_theta(:,:,i) = GetExponential(S.omega(:,:,i), S.theta(i), S.q(:,:,i));
             end
             g1(:,:,1) = exp_twist_theta(:,:,1);
            for i = 2:x
                g1(:,:,i) = g1(:,:,i-1)*exp_twist_theta(:,:,i);
            end
            for i = 1:x
                g1(:,:,i) = g1(:,:,i)*g_zero;
            end
            for i = 1:x
                Adjoint_Matrix(:,:,i) = GetAdjoint(g1(:,:,i));
            end
    
            %Computing the twist dash second joint onwards. These twist dash form
            %the columns of the Spatial Jacobian.
            for i = 2:x
                eta_dash(:,:,i) =  GetTwistDash(Adjoint_Matrix(:,:,i), eta(:,:,i));
            end
            J_spatial(:,1) = eta(:,:,1);
            for i = 2:x
                J_spatial(:,i) = eta_dash(:,:,i);
            end
            
        end
            
        end
end


