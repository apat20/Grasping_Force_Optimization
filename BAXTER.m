% Defining a class for the BAXTER arm

classdef BAXTER
   properties
       theta
       omega
       q
       P_base
   end
   methods
%  Defining a constructor method to construct the 
       function B = BAXTER(theta,omega,q,P_base)
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
   end
end