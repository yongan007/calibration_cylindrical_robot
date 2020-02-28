function t = theta_jac(q, theta, param)
   
t = zeros(6,param.theta_no);
   
L1 = param.links(1,1); L2 = param.links(2,1); L3 = param.links(3,1); L4 = param.links(4,1);
Ttool = param.tool;Tbase= Tz(L1);
 
drx = dRx(0); dry = dRy(0); drz = dRz(0);
dtx = dTx(0); dty = dTy(0); dtz = dTz(0);

T_rot = Rz(q(1))*Tz(q(2))*Tx(q(3));

tmp = Tbase * Rz(q(1)) * drz * Tz(L2) * Tz(q(2)) * Tx(L3) * Tx(q(3)) * Tx(L4) * Ttool * T_rot;

t(:,1) = Jcol(tmp);

tmp = Tbase * Rz(q(1)) * Tz(L2) *dtx* Tz(q(2)) * Tx(L3) * Tx(q(3)) * Tx(L4) * Ttool * T_rot;

t(:,2) = Jcol(tmp);

tmp = Tbase * Rz(q(1)) * Tz(L2) *dty* Tz(q(2)) * Tx(L3) * Tx(q(3)) * Tx(L4) * Ttool * T_rot;

t(:,3) = Jcol(tmp);

tmp = Tbase * Rz(q(1)) * Tz(L2) *dtz* Tz(q(2)) * Tx(L3) * Tx(q(3)) * Tx(L4) * Ttool * T_rot;

t(:,4) = Jcol(tmp);

tmp = Tbase * Rz(q(1)) * Tz(L2) *drx* Tz(q(2)) * Tx(L3) * Tx(q(3)) * Tx(L4) * Ttool * T_rot;

t(:,5) = Jcol(tmp);

tmp = Tbase * Rz(q(1)) * Tz(L2) *dry* Tz(q(2)) * Tx(L3) * Tx(q(3)) * Tx(L4) * Ttool * T_rot;

t(:,6) = Jcol(tmp);

tmp = Tbase * Rz(q(1)) * Tz(L2) *drz* Tz(q(2)) * Tx(L3) * Tx(q(3)) * Tx(L4) * Ttool * T_rot;

t(:,7) = Jcol(tmp);

tmp = Tbase * Rz(q(1)) * Tz(L2) * Tz(q(2)) *dtz* Tx(L3) * Tx(q(3)) * Tx(L4) * Ttool * T_rot;

t(:,8) = Jcol(tmp);

tmp = Tbase * Rz(q(1)) * Tz(L2) * Tz(q(2)) * Tx(L3) * dtx* Tx(q(3)) * Tx(L4) * Ttool * T_rot;

t(:,9) = Jcol(tmp);

tmp = Tbase * Rz(q(1)) * Tz(L2) * Tz(q(2)) * Tx(L3) * dty* Tx(q(3)) * Tx(L4) * Ttool * T_rot;

t(:,10) = Jcol(tmp);

tmp = Tbase * Rz(q(1)) * Tz(L2) * Tz(q(2)) * Tx(L3) * dtz* Tx(q(3)) * Tx(L4) * Ttool * T_rot;

t(:,11) = Jcol(tmp);

tmp = Tbase * Rz(q(1)) * Tz(L2) * Tz(q(2)) * Tx(L3) * drx* Tx(q(3)) * Tx(L4) * Ttool * T_rot;

t(:,12) = Jcol(tmp);

tmp = Tbase * Rz(q(1)) * Tz(L2) * Tz(q(2)) * Tx(L3) * dry* Tx(q(3)) * Tx(L4) * Ttool * T_rot;

t(:,13) = Jcol(tmp);

tmp = Tbase * Rz(q(1)) * Tz(L2) * Tz(q(2)) * Tx(L3) * drz* Tx(q(3)) * Tx(L4) * Ttool * T_rot;

t(:,14) = Jcol(tmp);

tmp = Tbase * Rz(q(1)) * Tz(L2) * Tz(q(2)) * Tx(L3) * Tx(q(3)) * dtx* Tx(L4) * Ttool * T_rot;

t(:,15) = Jcol(tmp);

tmp = Tbase * Rz(q(1)) * Tz(L2) * Tz(q(2)) * Tx(L3) * Tx(q(3)) * Tx(L4)* dtx * Ttool * T_rot;

t(:,16) = Jcol(tmp);

tmp = Tbase * Rz(q(1)) * Tz(L2) * Tz(q(2)) * Tx(L3) * Tx(q(3)) *Tx(L4)* dty * Ttool * T_rot;

t(:,17) = Jcol(tmp);

tmp = Tbase * Rz(q(1)) * Tz(L2) * Tz(q(2)) * Tx(L3) * Tx(q(3)) *Tx(L4)* dtz * Ttool * T_rot;

t(:,18) = Jcol(tmp);

tmp = Tbase * Rz(q(1)) * Tz(L2) * Tz(q(2)) * Tx(L3) * Tx(q(3)) *Tx(L4)* drx * Ttool * T_rot;

t(:,19) = Jcol(tmp);

tmp = Tbase * Rz(q(1)) * Tz(L2) * Tz(q(2)) * Tx(L3) * Tx(q(3)) *Tx(L4)* dry * Ttool * T_rot;

t(:,20) = Jcol(tmp);

tmp = Tbase * Rz(q(1)) * Tz(L2) * Tz(q(2)) * Tx(L3) * Tx(q(3)) *Tx(L4)* drz * Ttool * T_rot;

t(:,21) = Jcol(tmp);



end