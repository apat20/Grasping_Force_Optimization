%This function is used to implement the exponential of twist formula 
%The exponential of twists formula is used to calculate the rigid body
%transformation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%INPUT ARGUMENTS%%

% 'Omega' is a 3*1 vector which which encodes the information regarding the axis 
%rotation.

% 'q' is a 3*1 vector which contains the position of a point on the axis of
%rotation. The choice of this point is arbitary and depends on the user.

% 'theta' is the angle of rotation in degrees. It has been converted into
%radians for computation purposes.


%%
function exp_twist_theta = GetExponential(omega, theta, q)
    
    %The identity matrix 'I'
    I = eye(3);
    
    %Skew symmetric form of the omega vector
    omega_hat = skewSymmetric(omega);
    
   %Using the Rodriguez formula to calculate the exponential of omega
   %formula to get the exponential coordinates for rotation.
   
   % Write a function for this and include it in here.!!!!!
   exp_omega_hat_theta = I + omega_hat*sin(deg2rad(theta)) + omega_hat^2*(1-cos(deg2rad(theta)));
   
   %Calculating the transformation using the exponential of twist formula
   exp_twist_theta = [exp_omega_hat_theta, (I - exp_omega_hat_theta)*q;
                      zeros(1,3),           1];
   
end