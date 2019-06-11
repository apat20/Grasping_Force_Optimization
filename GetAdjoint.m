%This function is used to compute the Adjoint matrix given the transformation matrix.
%The rotation matrix 'R' and the position vector 'p' have been extracted
%from the transformation matrix 'g' and used in calculating the adjoint of
%the transformation matrix.

function Adjoint = GetAdjoint(g)
    R = g(1:3,1:3);
    p = g(1:3,4);
    p_hat = [0, -p(3), p(2);
             p(3),0, p(1);
             -p(2),p(1),0];
         
    Adjoint = [R, p_hat*R
               zeros(3), R];
end