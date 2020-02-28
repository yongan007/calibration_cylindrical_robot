function Q = check_limits(q,robot)
   Q = q;
   for i = 1:length(q)
       if isreal(q(i)) && (q(i) > robot.limits(i,2) || q(i) < robot.limits(i,1))
           %[robot.limits(i,1) robot.limits(i,2) q(i)]
           Q(i) = 1i;
       end
   end
end