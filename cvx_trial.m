%The rotation matrices for our selected problem
R_1 = [0,1,0;
       0,0,1;
       1,0,0];
R_2 = [1,0,0;
       0,0,-1;
       0,1,0];

%The position vectors in the skew symmetric form for our selected problem
p1_hat = [0,0,2;
          0,0,0;
         -2,0,0];
p2_hat = [0,0,-2;
          0,0,0;
          2,0,0]; 
B_c = [1,0,0;
        0,1,0;
        0,0,1;
        0,0,0;
        0,0,0;
        0,0,0];
 Adjoint = [R_1      , zeros(3);
            p1_hat*R_1, R_1];
           
 G = Adjoint*B_c;
 disp(G);
 
 
 