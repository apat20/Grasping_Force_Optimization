% Defining a class for the BAXTER arm

classdef BAXTER
%    The access to the object properties is public. The properties of the
%    object can be accessed after the object or the instance of the class
%    has ben created using the constructor method.
   properties
       theta
       omega
       q
       P_base
   end
   methods
%      Defining a constructor method to construct the instance of the
%      BAXTER class
       function B = BAXTER(theta,omega,q,P_base)
%      The two 'if' loops have been written for checking the errors (if
%      any) while initializing the instance of the object.
           if nargin == 4
                if class(theta) == 'double' & class(omega) == 'double' & class(q) == 'double' & class(P_base) == 'double'
                    B.theta = theta;
                    B.omega = omega;
                    B.q = q; 
                    B.P_base = P_base;
                end
            end
       end
%  Defining a function to calculate the Spatial Jacobian for the Baxter
%  robot arm.
       function [Adjoint_Matrix, g1, J_spatial] = spatialJacobian(B)
             %[x,~,~] = size(S.omega);
             I = eye(3);
             g_zero = [I, B.P_base;
             zeros(1,3),1];
             [Adjoint_Matrix, g1, J_spatial] = spatialJacobian(B.theta, B.omega, g_zero, B.q);            
         end
   end
   
   %   Defining static methods for the analytical and body Jacobian. Static
%   methods do not take the object as an input argument.
    methods(Static = true)

%       Method for computing the analytical Jacobian using the Spatial
%       Jacobian and the transformation matrix between the Spatial frame
%       and the tool frame.
        function J_analytical = analyticalJacobian(J_spatial, g1)
            p = g1(1:3,4,3);
            p_hat = [0, -p(3), p(2);
                     p(3), 0 , p(1);
                    -p(2), p(1), 0];
            I = eye(3);
            Z = zeros(3);
            J_analytical = [I, -p_hat; Z,  I]*J_spatial;
        end

%       Method for computing the Body Jacobian using the Adjoint of the
%       transformation matrix between the spatial frame(fixed frame) and
%       the tool frame and the Spatial Jacobian for our manipulator.
        function J_body = bodyJacobian(Adjoint_Matrix, J_spatial)
            [~,~,x] = size(Adjoint_Matrix);
            J_body = Adjoint_Matrix(:,:,x)*J_spatial;
        end
end
end