function [dt,vv,Kc_inv] = delta_svd2(q,param)
   J = theta_jac(q,0,param);
   
   % simplify for joints if needs
   if ~param.use_links
       tmp = zeros(6,param.joint_no);
       for i = 1:param.joint_no
           tmp(:,i) = J(:,7*i-6);
       end
       J = tmp;
   end   
   % stiffness matrix in joint space
   if param.use_links
       K = K_full(param);
   else
       K = diag(param.Kq);
   end  
   
   % stiffness matrix in task space
   Kc_inv = (J/K)*J';
   Kc_inv = Kc_inv(1:3,1:3);
   if isfinite(Kc_inv)
       [~,S,U] = svd(Kc_inv);
       dt = S(1,1);
       vv = U(:,1);
   else
       dt = NaN;
       vv = NaN;
   end   
end