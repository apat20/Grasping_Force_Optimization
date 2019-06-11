%This function is used to calculate the eta dash which forms the columns of
%the Spatial Jacobian.

function eta_dash = GetTwistDash(Adj_g, eta)
    eta_dash = Adj_g*eta;
end