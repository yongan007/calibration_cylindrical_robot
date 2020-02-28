function [q1,q2] = solve2link(l1,l2,x,y,m)
% Find solution for 2 link manipulator
% l1, l2 - length of the links
% x, y - coordinates of the second link tip
% m - sign (+1 or -1)
  
   r2 = x^2 + y^2;
   q2 = m*acos((r2-l1^2-l2^2)/(2*l1*l2));
   q1 = atan2(y,x) - m*acos((r2+l1^2-l2^2)/(2*l1*sqrt(r2)));
end