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
        function [g1,J_spatial] = spatialJacobian(S)
             %[x,~,~] = size(S.omega);
             I = eye(3);
             g_zero = [I, S.P_base;
             zeros(1,3),1];
             [g1, J_spatial] = spatialJacobian(S.theta, S.omega, g_zero, S.q);            
         end
            
        end
end


