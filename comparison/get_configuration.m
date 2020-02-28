function  Q = get_configuration(P,L);

d2 = L(1);   a2 =L(2);   a3 = L(3);   d4 = L(4);   a4 = L(5);   d6 =L(6);

T = Tx(P(1))*Ty(P(2))*Tz(P(3))*Rx(P(4))*Ry(P(5))*Rz(P(6));

T = T*Tx(-d6);
q1 = atan2(T(2,4),T(1,4));
T = Tx(-a2)*Tz(-d2)*Rz(-q1)*T;
q3 = acos((T(1,4)^2+T(3,4)^2-a3^2-d4^2)/2/a3/d4);
q2 = -acos((a3^2+T(1,4)^2+T(3,4)^2-d4^2)/(2*a3*sqrt(T(1,4)^2+T(3,4)^2)))+atan2(-T(3,4),T(1,4));

T23 =  Tx(-d4)*Ry(-q3)*Tx(-a3)*Ry(-q2);
T = T23*T;

q5 = acos(T(1,1));
if abs(q5)>1e-5
    q4=atan2(T(2,1),  -T(3,1));
    q6=atan2(T(1,2),  T(1,3));
else
    q4 = acos(T(2,2));
    q6 = 0;
end;

Q = [-q1, q2, q3, -q4, q5, -q6]';