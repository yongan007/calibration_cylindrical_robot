function t = Rx(a)
   t = eye(4);
   t(2,2) = cos(a);  t(2,3) = -sin(a);
   t(3,2) = -t(2,3); t(3,3) = t(2,2);   
end