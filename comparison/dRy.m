function t = dRy(a)
   t = zeros(4);
   t(1,1) = -sin(a);  t(1,3) = cos(a);
   t(3,1) = -t(1,3); t(3,3) = t(1,1);
end