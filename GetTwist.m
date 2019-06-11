%This function is to calculate the twist corresponding to a transformation 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%INPUT ARGUMENTS%%

% 'Omega' is a 3*1 vector which which encodes the information regarding the axis 
%rotation.

% 'q' is a 3*1 vector which contains the position of a point on the axis of
%rotation. The choice of this point is arbitary and depends on the user.


%%
function eta = GetTwist(omega,q)
    eta = [cross(-omega,q);
               omega];           
end