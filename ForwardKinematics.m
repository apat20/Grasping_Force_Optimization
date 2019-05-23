%This function is used to compute the forward kinematics given a rotation
%matrix and a position vector for a point lying on an axis


function g = ForwardKinematics(R,q)
    I = eye(3,3);
    g = [R, (I-R)*q;
        zeros(1,3), 1];
end