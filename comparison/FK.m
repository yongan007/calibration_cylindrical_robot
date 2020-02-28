function out = FK(q,params)

T = Tbase*Rz(q(1))*...
    Tz(params(1))*Tz(q(2))*Tz(params(3))*Tx(q(3))*Tz(params(4))*Ttool;

T(1:3,4) = T(1:3,4)+sigma*randn(3,1);
out = T;
%[T(1:3,4)+sigma*randn(3,1)];


%    T = Tz(robot.links(1,1))*Rz(q(1))*Tx(robot.links(2,1))*Ry(q(2))*Tx(robot.links(3,1))*Ry(q(3))*...
%        Tx(robot.links(4,1))*Rx(q(4))*Tx(robot.links(5,1))*Ry(q(5))*Tx(robot.links(6,1))*Rx(q(6))*Tx(robot.links(7,1));
%    
%    X = [T(1:3,4);a;b;c];
end