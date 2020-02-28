function q = IK(T, robot)
%   Computes joints coordinates from homogeous transformation of the
%   end effector.
%   This is an implementation of Pieper's solution for
%   6 DOF KUKA AGILUS manipulator that has wrist's axies intersection 
%   in one point.
%   T - 4x4 transformation matrix.
%   robot - robot's description.
    
    % find wrist center
    lastJoint = robot.Joints(end);
    d = lastJoint.child(1).length + lastJoint.parent(1).length;
    wristCenter = T(1:3, 4) - d*T(1:3, 1:3)*lastJoint.child(1).axis';
    
    % solve inverse kinematics for 123 joints and get 2 solutions
    q123 = IK123(wristCenter, robot);
    
    % for each 123 solution solve inverse kinematics for 456
    q1456 = IK456(T, q123(1:3, 1), robot);
    q2456 = IK456(T, q123(1:3, 2), robot);
    
    % combine solutions
    q = [q123(1:3, 1) q123(1:3, 2); q1456 q2456];
end

function q123 = IK123(wristCenter, robot)
    L1 = robot.Joints(1).child(1).length;
    L2 = robot.Joints(2).child(1).length;
    L3 = robot.Joints(3).child(1).length + robot.Joints(4).child(1).length;
    
    alpha = atan2(wristCenter(2), wristCenter(1));
    
    eps = 1e-9;
    if abs(sin(alpha)) > eps
        rel = wristCenter(2)/sin(alpha);
    else
        rel = wristCenter(1)/cos(alpha);
    end
    
    gamma1 = acos((rel^2 + (L1 - wristCenter(3))^2 - L2^2 - L3^2)/(2*L2*L3));
    gamma2 = -acos((rel^2 + (L1 - wristCenter(3))^2 - L2^2 - L3^2)/(2*L2*L3));
    
    beta1 = atan2(L1 - wristCenter(3), rel) - ...
        atan2(L3*sin(gamma1), L2 + L3*cos(gamma1));
    beta2 = atan2(L1 - wristCenter(3), rel) - ...
        atan2(L3*sin(gamma2), L2 + L3*cos(gamma2));
    
    q123 = [alpha alpha; beta1 beta2; gamma1 gamma2];
end

function q456 = IK456(T, q123, robot)
    robot123.Links = robot.Links(1:4);
    robot123.Joints = robot.Joints(1:3);
    T123 = FK(q123, robot123);
    
    T456 = T123\T;
    alpha = atan2(T456(2,1), -T456(3, 1));
    gamma = atan2(T456(1, 2), T456(1, 3));
    
    eps = 1e-9;
    if abs(sin(gamma)) > eps
        beta = atan2(T456(1,2) / sin(gamma), T456(1,1));
    else
        beta = atan2(T456(1,3) / cos(gamma), T456(1,1));
    end
    
    q456 = [alpha; beta; gamma];
end