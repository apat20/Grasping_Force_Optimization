classdef ALLEGRO
    properties
        theta
        omega
        q
        P_base
    end
    methods
        function A = ALLEGRO(theta, omega, q, P_base)
            if nargin == 4
                if class(theta) == 'double' & class(omega) == 'double' & class(q) == 'double' & class(P_base) == 'double'
                    A.theta = theta;
                    A.omega = omega;
                    A.q = q; 
                    A.P_base = P_base;
                end
            end
        end
       function [Adjoint_Matrix, g1, J_spatial] = spatialJacobian(A)
             %[x,~,~] = size(S.omega);
             I = eye(3);
             g_zero = [I, A.P_base;
             zeros(1,3),1];
             [Adjoint_Matrix, g1, J_spatial] = spatialJacobian(A.theta, A.omega, g_zero, A.q);            
       end
    end
end