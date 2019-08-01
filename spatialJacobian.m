%This function is used to compute the Spatial Jacobian

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% INPUT ARGUMENTS %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%The data is stored in the multidimensional arrays where in each page
%contains an individual vector or a matrix depending on the information
%encoded.

% 'theta' is a vector containing the angles of rotation of the revolute joints
%of the manipulator under consideration 

% 'omega' is a multidimensional array in which each column vector contains the information regarding the axis
%of rotation corresponding to each and every revolute joint.

% 'q' is a multidimensional array in which each vector contains the position of any
%arbitarily chosen point on the axis of rotation.

% 'P' is a position vector of the tool frame with respect to the inertial
%frame located at the base of the manipulator.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% OUTPUT %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% CODE %%J
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [J_spatial,g1] = spatialJacobian(theta, omega, q)
    [~,~,x] = size(omega);

    %Computing the twists and storing them in a multidimensional array 'eta'
    %where they can be referenced to as pages of the array.
    %Computing the Exponential of twists formula for transformations from the
    %base frame upto the points under consideration.
    for i=1:x
        eta(:,:,i) = GetTwist(omega(:,:,i),q(:,:,i));
        exp_twist_theta(:,:,i) = GetExponential(omega(:,:,i), theta(i), q(:,:,i));
    end
     
    %Calculating this transformation upto of the tool frame with respect to the
    %frame at the base of the manipulator etc. This is stored in a
    %multidimensional array 'g1'. Each page of the multidimensional array
    %contains the rigid body transformation upto that point e.g. g1_1, g1_2 and
    %g1_3
   g1(:,:,1) = exp_twist_theta(:,:,1);
   for i = 2:x
        g1(:,:,i) = g1(:,:,i-1)*exp_twist_theta(:,:,i);
   end
   
   [~,~,a] = size(g1);
 
    for i = 1:a-1
    %Computing the Adjoint Matrix using the function GetAdjoint for all the
    %three rigid body transformations.
        Adjoint_Matrix(:,:,i) = GetAdjoint(g1(:,:,i));
    %Computing the twist dash second joint onwards. These twist dash form
    %the columns of the Spatial Jacobian.
        eta_dash(:,:,i) =  GetTwistDash(Adjoint_Matrix(:,:,i), eta(:,:,i+1));
    end

     J_spatial(:,1) = eta(:,:,1);
    for i = 2:x
        J_spatial(:,i) = eta_dash(:,:,i-1);
    end
end

