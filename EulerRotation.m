%This function is used to calculate the rotation matrix using the 
%X-Y-Z fixed angle representation 

function rotation_matrix = EulerRotation(gama, beta, alpha)
    R_1 = [1,         0        ,       0           ;
           0,cos(deg2rad(gama)),-sin(deg2rad(gama));
           0,sin(deg2rad(gama)),cos(deg2rad(gama))];
    
    R_2 = [cos(deg2rad(beta)),0,sin(deg2rad(beta));
              0               ,1,     0            ;
           -sin(deg2rad(beta)),0,cos(deg2rad(beta))];
    
    R_3 = [cos(deg2rad(alpha)),-sin(deg2rad(alpha)),0;
           sin(deg2rad(alpha)),cos(deg2rad(alpha)),0;
                   0         ,           0        ,1];
    rotation_matrix = R_3*R_2*R_1;
end 

