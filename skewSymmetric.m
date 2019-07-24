% This function is used to get the skew symmetric form of any vector with
% 3 components:

function p_hat = skewSymmetric(p)
    if size(p) == [3,1]
        p_hat = [0,-p(3),p(2);
                    p(3),0, -p(1);
                    -p(2), p(1),0];
    else 
        fprintf('Incorrect dimensions!')
    end
end