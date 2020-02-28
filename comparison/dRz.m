function t = dRz(a)
   t = zeros(4);
   t(1,1) = -sin(a);  t(1,2) = -cos(a);
   t(2,1) = -t(1,2); t(2,2) = t(1,1); 
end