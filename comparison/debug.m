% find configuration

% parameters;
% 
% q0 = 2*rand(6,1)-1
% X = FK(q0,robot)
% q1 = IK(X, [1,1,1], robot)
% q2 = IK(X, [1,1,-1], robot)
% q3 = IK(X, [1,-1,-1], robot)
% q4 = IK(X, [1,-1,1], robot)
% 
% if isreal(q1)
% X1 = FK(q1,robot);
% end
% if isreal(q2)
% X2 = FK(q2,robot);
% end
% if isreal(q3)
% X3 = FK(q3,robot);
% end
% if isreal(q4)
% X4 = FK(q4,robot);
% end
%X = IK([10,10,10,10,10,10],robot)

for i = 1:8
    x = [2*mod(floor(i/4),2)-1,2*mod(floor(i/2),2)-1,2*mod(i,2)-1]
end

