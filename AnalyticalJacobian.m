%This function is used to compute the Analytical Jacobian from the Spatial
%Jacobian

function J_analytical = AnalyticalJacobian(J_spatial,p)
    p_hat = [0, -p(3), p(2);
             p(3),0, p(1);
             -p(2),p(1),0];
    I = eye(3);
    Z = zeros(3);
    J_analytical = [I, -p_hat; Z,  I]*J_spatial;
end