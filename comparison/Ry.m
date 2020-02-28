function t = Ry(a)
   t = eye(4);
   t(1,1) = cos(a);  t(1,3) = sin(a);
   t(3,1) = -t(1,3); t(3,3) = t(1,1);
end