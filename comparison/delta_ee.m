function [dt,dq] = delta_ee(q,theta,force,param) 
% Find end-effector displacement  
   J = theta_jac(q, theta, param);
   
   % simplify for joints if needs
   if ~param.use_links
       tmp = zeros(6,param.joint_no);
       for i = 1:param.joint_no
           tmp(:,i) = J(:,7*i-6);
       end
       J = tmp;
   end   
   % stiffness matrix
   if param.use_links
       K = K_full(param);
   else
       K = diag(param.Kq);
   end  
      
   % joint displacement
   %J' * force
   dq = K \ (J' * force);  
   
   % gravity
   if ~param.use_links && param.use_gravity
       tau_m = m_torque(q,theta,param);
       ddq = diag(param.Kq) \ tau_m;
       dq = dq + ddq;
   end
   
   % end-effector displacement
   dt = J * dq;   
   if ~param.use_links
       tmp = zeros(param.theta_no,1);
       for i = 1:param.joint_no
           tmp(7*i-6) = dq(i);
       end
       dq = tmp;
   end
   %dt = ee_trans(q, dq + theta, param) - ee_trans(q,theta,param);
end