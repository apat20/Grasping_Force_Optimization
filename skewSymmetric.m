% This function is used to get the skew symmetric form of any vector with
% 3 components:

function vect_hat = skewSymmetric(vect)
    if size(vect) == [3,1]
        vect_hat = [0,-vect(3),vect(2);
                    vect(3),0, vect(1);
                    -vect(2), vect(1),0];
    else 
        fprintf('Incorrect dimensions!')
    end
end