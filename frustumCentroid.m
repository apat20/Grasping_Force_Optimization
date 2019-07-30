function g = frustumCentroid(r1,r2,h)
    g = (h*(r1^2 + 2*r1*r2 + 3*r2^2)/(4*(r1^2 + r1*r2 + r2^2)));
end