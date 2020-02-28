function [t,T,o,z] = ee_trans(q, theta, param)
% Forward kinematics for end-effector position calculation
   o = zeros(3,8);
   z = zeros(3,8);
   z(3,1) = 1;
   
   T = Tz(param.links(1,1));
   o(:,2) = T(1:3,4);
   z(:,2) = T(1:3,3);
   T = T * Rz((q(1)+theta(1))) *  Tx(param.links(2,1)) * Tx(theta(2))* Ty(theta(3))* Tz(theta(4))*Rx(theta(5))*Ry(theta(6))*Rz(theta(7));
   o(:,3) = T(1:3,4);
   z(:,3) = T(1:3,2);
   T = T * Ry(q(2)+theta(8)) * Tx(param.links(3,1)) * Tx(theta(9))*Ty(theta(10))*Tz(theta(11))*Rx(theta(12))*Ry(theta(13))*Rz(theta(14));
   o(:,4) = T(1:3,4);
   z(:,4) = T(1:3,2);
   T = T * Ry(q(3)+theta(15)) * Tx(param.links(4,1)) * Tx(theta(16))*Ty(theta(17))*Tz(theta(18))*Rx(theta(19))*Ry(theta(20))*Rz(theta(21));
   o(:,5) = T(1:3,4);
   z(:,5) = T(1:3,1);
   T = T * Rx(q(4)+theta(22)) * Tx(param.links(5,1)) * Tx(theta(23))*Ty(theta(24))*Tz(theta(25))*Rx(theta(26))*Ry(theta(27))*Rz(theta(28));
   o(:,6) = T(1:3,4);
   z(:,6) = T(1:3,2); 
   T = T * Ry(q(5)+theta(29)) * Tx(param.links(6,1)) * Tx(theta(30))*Ty(theta(31))*Tz(theta(32))*Rx(theta(33))*Ry(theta(34))*Rz(theta(35));
   o(:,7) = T(1:3,4);
   z(:,7) = T(1:3,1);
   T = T * Rx(q(6)+theta(36)) * Tx(param.links(7,1)) * Tx(theta(37))*Ty(theta(38))*Tz(theta(39))*Rx(theta(40))*Ry(theta(41))*Rz(theta(42));
   o(:,8) = T(1:3,4);
   z(:,8) = T(1:3,1);
   T = T * param.tool;
   
   t = T(1:3,4);    
end