function t = Ktheta(n,param)
% Stiffness matrix of the single link
   L = param.links(n,1);   
   a = param.links(n,2);
   b = param.links(n,3);  
   G = param.E / (2*(1+param.nu));
   % quill cylinder
   S = pi*(a^2-b^2)/4;
   Iy = pi*(a^4-b^4)/64;
   Iz = Iy;
   J = Iy+Iz;
   % square
   %S = a*b;
   %Iy = a^3*b/12;
   %Iz = a*b^3/12;
   %J = (a^2+b^2)*a*b/12;
   % matrix   
   t = (param.E/L^2) * [S*L,       0,       0,             0,      0,      0;
                          0, 12*Iz/L,       0,             0,      0,   6*Iz;
                          0,       0, 12*Iy/L,             0,  -6*Iy,      0;
                          0,       0,       0, G*J*L/param.E,      0,      0;
                          0,       0,   -6*Iy,             0, 4*Iy*L,      0;
                          0,    6*Iz,       0,             0,      0, 4*Iz*L];                         
    
end