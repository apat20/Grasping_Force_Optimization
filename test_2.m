%The angles of rotation for each of the revolute joints of the SCARA
%manipulator.
theta_1 = 30;
theta_2 = 20;
theta_3 = 20;

theta = [theta_1;theta_2;theta_3];
disp(theta);

%Omega encodes the information regarding the axis of rotation of the SCARA
%manipulator.
omega_1 = [0;0;1];
omega_2 = [0;0;1];
omega_3 = [0;0;1];

omega(:,:,1) = omega_1;
omega(:,:,2) = omega_2;
omega(:,:,3) = omega_3;

disp(omega);

%Position vectors for the aribitarly chosen point on the axis of rotation.
q_1 = [0;0;0];
q_2 = [0;20;0];
q_3 = [0;50;0];

q(:,:,1) = q_1;
q(:,:,2) = q_2;
q(:,:,3) = q_3;

disp(q);

%I is the identity matrix
I = eye(3);

%P is the position vector of the end effecctor or tool frame with respect
%to the inertial frame located at the base of the manipulator or in our
%case the base of the SCARA manipulator.
%P = [0; l1+l2; l0]
P = [0;50;10];

%This is the rigid body transformation matrix for the zero configuration of
%the manipulator under consideration. In our case it is the SCARA
%manipulator.
g_zero = [I, P;
          zeros(1,3),1];
fprintf('The g_st_zero transformation matrix is:');
fprintf('\n');
disp(g_zero);

[g1,J_spatial] = SpatialJacobian(theta, omega, g_zero, q);
disp(J_spatial);
disp(g1);


% function J_spatial = SpatialJacobian(theta, omega, g_zero, q)
%     [x,~,~] = size(omega);
% 
%     %Computing the twists and storing them in a multidimensional array 'eta'
%     %where they can be referenced to as pages of the array.
%     %Computing the Exponential of twists formula for transformations from the
%     %base frame upto the points under consideration.
%     for i=1:x
%         eta(:,:,i) = GetTwist(omega(:,:,i),q(:,:,i));
%         exp_twist_theta(:,:,i) = GetExponential(omega(:,:,i), theta(i), q(:,:,i));
%     end
%     % function J_spatial = SpatialJacobian(theta, omega, g_zero, q)
%     [x,~,~] = size(omega);
% 
%     %Computing the twists and storing them in a multidimensional array 'eta'
%     %where they can be referenced to as pages of the array.
%     %Computing the Exponential of twists formula for transformations from the
%     %base frame upto the points under consideration.
%     for i=1:x
%         eta(:,:,i) = GetTwist(omega(:,:,i),q(:,:,i));
%         exp_twist_theta(:,:,i) = GetExponential(omega(:,:,i), theta(i), q(:,:,i));
%     end
%     
%     %Calculating this transformation upto of the tool frame with respect to the
%     %frame at the base of the manipulator etc. This is stored in a
%     %multidimensional array 'g1'. Each page of the multidimensional array
%     %contains the rigid body transformation upto that point e.g. g1_1, g1_2 and
%     %g1_3
%     g1(:,:,1) = exp_twist_theta(:,:,1);
%     for i = 2:x
%         g1(:,:,i) = g1(:,:,i-1)*exp_twist_theta(:,:,i);
%     end
%     for i = 1:x
%         g1(:,:,i) = g1(:,:,i)*g_zero;
%     end
%     
%     %Computing the Adjoint Matrix using the function GetAdjoint for all the
%     %three rigid body transformations.
%     for i = 1:x
%         Adjoint_Matrix(:,:,i) = GetAdjoint(g1(:,:,i));
%     end
% 
%     for i = 2:x
%         eta_dash(:,:,i) =  GetTwistDash(Adjoint_Matrix(:,:,i), eta(:,:,i));
%     end
% 
%     J_spatial(:,1) = eta(:,:,1);
%     for i = 2:x
%         J_spatial(:,i) = eta_dash(:,:,i);
%     end
% end
%     %Calculating this transformation upto of the tool frame with respect to the
%     %frame at the base of the manipulator etc. This is stored in a
%     %multidimensional array 'g1'. Each page of the multidimensional array
%     %contains the rigid body transformation upto that point e.g. g1_1, g1_2 and
%     %g1_3
%     g1(:,:,1) = exp_twist_theta(:,:,1);
%     for i = 2:x
%         g1(:,:,i) = g1(:,:,i-1)*exp_twist_theta(:,:,i);
%     end
%     for i = 1:x
%         g1(:,:,i) = g1(:,:,i)*g_zero;
%     end
%     
%     %Computing the Adjoint Matrix using the function GetAdjoint for all the
%     %three rigid body transformations.
%     for i = 1:x
%         Adjoint_Matrix(:,:,i) = GetAdjoint(g1(:,:,i));
%     end
% 
%     for i = 2:x
%         eta_dash(:,:,i) =  GetTwistDash(Adjoint_Matrix(:,:,i), eta(:,:,i));
%     end
% 
%     J_spatial(:,1) = eta(:,:,1);
%     for i = 2:x
%         J_spatial(:,i) = eta_dash(:,:,i);
%     end
% end
% -32.9813
% 36.6041
% 10.0000
