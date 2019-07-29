% Defining a class SCARA to be used for the SCARA manipulator.

classdef SCARA
%    The access to the object properties is public. The properties of the
%    object can be accessed after the object or the instance of the class
%    has ben created using the constructor method.
     properties %(Access = private)
        theta
        omega
        q
        P_base
    end
    methods
%   Defining a constructor method to construct the instances of class
%   SCARA:
        function S = SCARA(theta, omega, q, P_base)
%      The two 'if' loops have been written for checking the errors (if
%      any) while initializing the instance of the object.
            if nargin == 4
                if class(theta) == 'double' & class(omega) == 'double' & class(q) == 'double' & class(P_base) == 'double'
                    S.theta = theta;
                    S.omega = omega;
                    S.q = q;
                    S.P_base = P_base;
                end
            end
        end
% Defining a function to calculate the Spatial Jacobian of a SCARA
%     manipulator.
        function J_spatial = spatialJacobian(S)
             J_spatial = spatialJacobian(S.theta, S.omega, S.q);            
        end
        
%      Defining a function to calculate the transformation matrix between
%      the base frame and the end effector frame of a manipulator
       function g_st = getTransform(S)
            g_st = getTransform(S.theta, S.omega, S.q, S.P_base); 
       end
    end
    
%   Defining static methods for the analytical and body Jacobian. Static
%   methods do not take the object as an input argument.
    methods(Static = true)

%       Method for computing the analytical Jacobian using the Spatial
%       Jacobian and the transformation matrix between the Spatial frame
%       and the tool frame.
        function J_analytical = analyticalJacobian(J_spatial, g_st)
            [~,~,x] = size(g_st);
            g = g_st(:,:,x);
            p = g(1:3,4);
            p_hat = skewSymmetric(p);
            I = eye(3);
            Z = zeros(3);
            J_analytical = [I, -p_hat; Z,  I]*J_spatial;
        end
        
%       Method for computing the Body Jacobian using the Adjoint of the
%       transformation matrix between the spatial frame(fixed frame) and
%       the tool frame and the Spatial Jacobian for our manipulator.
        function J_body = bodyJacobian(J_spatial, g_st)
            [~,~,x] = size(g_st);
            g = g_st(:,:,x);
            Adjoint = GetAdjoint(g);
            J_body = Adjoint*J_spatial;
        end


    end
end


