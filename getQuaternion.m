
% This function is used to get the quaternion representation for an
% equivalent exponential coordinate representation.

function q = getQuaternion(R)
    A = [1,1,1,1;
         1,-1,-1,1;
         -1,1,-1,1;
         -1,-1,1,1];
   
%   Find the value of the q_sqaured vector.
    q_squared = (1/4)*A*[R(1,1);R(2,2);R(3,3);1];
    
%   Find the maximum value of the q_squared vector.
    max_value = max(q_squared);
    
%   Apply the conditions in order to solve for the remaining values of the
%   quaternion representation.
    if max_value == q_squared(1)
       q(1) = sqrt(q_squared(1));
       q(2) = (R(3,2) - R(2,3))/4*q(1);
       q(3) = (R(1,3) - R(3,1))/4*q(1);
       q(4) = (R(2,1) - R(1,2))/4*q(1);
    elseif max_value == q_squared(2)
        q(2) = sqrt(q_squared(2));
        q(1) = (R(3,2) - R(2,3))/4*q(2);
        q(3) = (R(1,2) + R(2,1))/4*q(2);
        q(4) = (R(1,3) + R(3,1))/4*q(2);
    elseif max_value == q_squared(3)
         q(3) = sqrt(q_squared(3));
         q(1) = (R(1,3) - R(3,1))/4*q(3);
         q(2) = (R(1,2) + R(2,1))/4*q(3);
         q(4) = (R(2,3) + R(3,2))/4*q(3);
    elseif max_value == q_squared(4)
         q(4) = sqrt(q_squared(4));
         q(1) = (R(2,1) - R(1,2))/4*q(4);
         q(2) = (R(1,3) + R(3,1))/4*q(4);
         q(3) = (R(2,3) + R(3,2))/4*q(4);
    end
end