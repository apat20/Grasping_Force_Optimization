% This function is used to calculate the Body Jacobian from the Spatial
% Jacobian and the Adjoint matrix.

function J_body = bodyJacobian(Adjoint_Matrix, J_spatial)
    J_body = Adjoint_Matrix*J_spatial;
end

 