function t = dRx(a)
   t = zeros(4);
   t(2,2) = -sin(a); t(2,3) = -cos(a);
   t(3,2) = -t(2,3); t(3,3) = t(2,2); 
end