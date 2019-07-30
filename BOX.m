% Defining a class for the box to be tilted. We are creating a class as the
% dimensions of the box may vary or the angle of tilt of the box that is
% the orientation of the box may vary.

classdef BOX
    properties
        theta
        R_OC
        R_OE
        p_OC
        p_OE
    end 
    methods
%       Defining a constructor method for generating the an object of the
%       BOX class.
        function B = BOX(theta, R_OC, R_OE, p_OC, p_OE)
%           One 'if' loop to check whether the correct number of arguments
%           have been passed as input parameters.
            if nargin == 5
                B.theta = theta;
                B.R_OC = R_OC;
                B.R_OE = R_OE;
                B.p_OC = p_OC;
                B.p_OE = p_OE;
            end
        end
    end
    methods(Static = true)

%       Defining a function to find the skew symmetric form of the position
%       vectors in 'p_OE' and 'p_OC'
    function p_hat = skewSymmetric(p)
        [~,~,x] = size(p);
        for i = 1:x
            p_hat(:,:,i) = skewSymmetric(p(:,:,i));
        end
    end
    
%    Defining a static method for computing the grasp map for the box with
%    the desired properties.
    function G = graspMap(R, p_hat, c)
        [~,~,x] = size(R);
        for i = 1:x
            G(:,:,i) = GraspMap(R(:,:,i), p_hat(:,:,i), c);
        end
    end
    
%    Defining a static function to get the Adjoint of the transformation
%    matrix:
    function Adjoint = getAdjoint(R,p)
        [~,~,x] = size(R);
        for i = 1:x
            g(:,:,i) = [R(:,:,i), p(:,:,i);
                        zeros(1,3), 1];
            Adjoint(:,:,i) = GetAdjointWrench(g(:,:,i));
        end
    end
end
end