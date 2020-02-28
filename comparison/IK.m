function Q = IK(P,m,robot)


   l1 = robot.links(2,1);

   q1 = atan2(P(2), P(1));
   
%    if q1 isreal(A):
       
   q2 = P(3)-l1;
   q3= sqrt(P(1)^2+P(2)^2);
           
  
   Q = check_limits([q1;q2;q3], robot);   
end