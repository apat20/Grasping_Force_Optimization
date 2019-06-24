% This function is used to get the values of theta and omega from the
% equivalent quaternion representation. The data used here is from the
% BAXTER 7 D-o-F arm.

function [theta, omega] = quaternionToRotation(Quaternions)
    for i = 1:7
        q_0 = Quaternions(:,1,i);
%         fprintf('The scalar part of the quaternion is:');
%         fprintf('\n');
%         disp(q_0)
    
        Q(1,1) = Quaternions(:,2,i);
        Q(1,2) = Quaternions(:,3,i);
        Q(1,3) = Quaternions(:,4,i);
%         fprintf('The vector part of the quaternion is:');
%         fprintf('\n');
%         disp(Q)
    
        theta(i) = 2*acos(q_0);
        omega(:,:,i) = (Q/sin(theta(i)/2))';
    end
    theta = (theta)';
end