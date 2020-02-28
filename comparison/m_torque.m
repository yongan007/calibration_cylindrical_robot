function tau = m_torque(q,theta,param)
   g = 9.8;                                     % N/kg
   [~,~,o,z] = ee_trans(q, theta, param);   
   % torques
   tau = zeros(param.joint_no,1);
   for i = 1:param.joint_no
       k = i+1;
       % for each axes
       for j = k:(param.joint_no+1)
           % sum of torques
           f_pos = (o(:,k)+o(:,k+1))*0.5;       % center of mass
           f_val = [0;0;-g*param.links(k,4)];   % link force
           M = cross(f_pos - o(:,k), f_val);    % torque
           tau(i) = tau(i) + dot(z(:,k), M);    % projection to axes
       end
   end   
end