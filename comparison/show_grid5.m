% load configuration
close all
parameters;

%% find stiffness

theta = zeros(robot.theta_no,1);
 
N = robot.joint_no;
S1 = zeros(N, N);
S2 = zeros(N, 1);

iter = 600;   % nomber or experiments

robot.tool(1:3,4) = [0;0.1;0.1];

% experiments
for i = 1:iter
    % random force values
    force = zeros(6,1);
    force(1:3) = 2*rand(3,1)-1;    
    force = (force / norm(force)) * 1000; 
    %force(4:6) = 10*(2*rand(3,1)-1);
    % random joint values
    q0 = (pi/2)*(2*rand(robot.joint_no,1) - 1);    
    % displacement    
    [dt,dq] = delta_ee(q0,theta,force,robot);    
    % use only cartesian displacement
    dt = dt(1:3);
    A = get_A(q0, theta, force, robot);  
    
    % accumulate sum    
    S1 = S1 + A' * A;
    S2 = S2 + A' * dt;      
end

% results
ks = S1 \ S2;          % compliance

stiffness = 1 ./ ks  % stiffness

robot.tool = eye(4);

%% Make grid

X = linspace(-3,3,200);   % m
Y = linspace(-3,3,200);   % m
Z = linspace(-3,3,200);   % m

NX = length(X); NY = length(Y); NZ = length(Z);
%NV = 8;  % number of configurations

NO_VAL = NaN;

JSh = zeros(NX,NY, robot.joint_no);
JSv = zeros(NX,NZ, robot.joint_no);
JSv2 = zeros(NX,NZ, robot.joint_no);

CONF = 3;%5
CONF2 = 3;%7
ZLVL = 0.5;
YLVL = 0;

%robot.orientation = true;

m = configuration(CONF);
m2 = configuration(CONF2);

for i = 1:NX
    % horizontal
    for j = 1:NY
        conf = IK([X(i);Y(j);ZLVL;0;0;0], m, robot);        
        JSh(i,j,:) = conf;          
    end
    % vertical
    for k = 1:NZ
        conf = IK([X(i);YLVL;Z(k);0;0;0], m, robot);        
        JSv(i,k,:) = conf;
        conf = IK([X(i);YLVL;Z(k);0;0;0], m2, robot);       
        JSv2(i,k,:) = conf;
    end
end

%% Full model
 
slice3 = zeros(NX,NY);
slice3v = zeros(NX,NZ);
slice3vv = zeros(NX,NZ);

KCf_h = zeros(NX,NY,3,3);
KCf_v = zeros(NX,NZ,3,3);
KCf_vv = zeros(NX,NZ,3,3);
for i = 1:NX
  for j = 1:NY     
    q = JSh(i,j,:);
    if isreal(q)
      [delta,u,Ki] = delta_svd2(q,robot);        
      KCf_h(i,j,:,:) = Ki;
    else
      delta = NO_VAL;                              
    end
    slice3(i,j) = delta;      
  end
  for k = 1:NZ    
    q = JSv(i,k,:);
    if isreal(q)
      [delta,u,Ki] = delta_svd2(q,robot);          
      KCf_v(i,k,:,:) = Ki;
    else
      delta = NO_VAL;                              
    end
    slice3v(i,k) = delta;      
  end
  for k = 1:NZ    
    q = JSv2(i,k,:);
    if isreal(q)
      [delta,u,Ki] = delta_svd2(q,robot);      
      KCf_vv(i,k,:,:) = Ki;
    else
      delta = NO_VAL;                              
    end
    slice3vv(i,k) = delta;      
  end
end


plot_contour(slice3*1E6,Y,X,'Y, m','X, m');

slice3u = min(slice3v,slice3vv);


plot_contour(slice3u*1E6,Z,X,'Z, m','X, m');


createhist(slice3);



createhist(slice3u);


%% Reduced model
robot.Kq = stiffness;
robot.use_links = false;

slice4 = zeros(NX,NY);
slice4v = zeros(NX,NZ);
slice4vv = zeros(NX,NZ);

KCr_h = zeros(NX,NY,3,3);
KCr_v = zeros(NX,NZ,3,3);
KCr_vv = zeros(NX,NZ,3,3);

for i = 1:NX
  for j = 1:NY     
    q = JSh(i,j,:);
    if isreal(q)
      [sig,u,Kci] = delta_svd2(q,robot);
      KCr_h(i,j,:,:) = Kci;      
    else     
      sig = NO_VAL;
    end
    slice4(i,j) = sig;    
  end
  for j = 1:NZ     
    q = JSv(i,j,:);
    if isreal(q)
      [sig,u,Kci] = delta_svd2(q,robot);
      KCr_v(i,j,:,:) = Kci;      
    else      
      sig = NO_VAL;
    end
    slice4v(i,j) = sig;   
  end
  for j = 1:NZ     
    q = JSv2(i,j,:);
    if isreal(q)
      [sig,u] = delta_svd2(q,robot);
      KCr_vv(i,j,:,:) = Kci;      
    else      
      sig = NO_VAL;
    end
    slice4vv(i,j) = sig;    
  end
end


plot_contour(slice4*1E6,Y,X,'Y, m','X, m');

slice4u = min(slice4v, slice4vv);


plot_contour(slice4u*1E6, Z,X,'Z, m','X, m');


createhist(slice4);



createhist(slice4u);


%% Difference between two last cases

% full model, v1
sf_v1_h = zeros(NX,NY);
sf_v1_v = zeros(NX,NZ);
sf_v1_vv = zeros(NX,NZ);
% full model, v2
sf_v2_h = zeros(NX,NY);
sf_v2_v = zeros(NX,NZ);
sf_v2_vv = zeros(NX,NZ);
% full model, v3
sf_v3_h = zeros(NX,NY);
sf_v3_v = zeros(NX,NZ);
sf_v3_vv = zeros(NX,NZ);
% reduced model, v1
sr_v1_h = zeros(NX,NY);
sr_v1_v = zeros(NX,NZ);
sr_v1_vv = zeros(NX,NZ);
% reduced model, v2
sr_v2_h = zeros(NX,NY);
sr_v2_v = zeros(NX,NZ);
sr_v2_vv = zeros(NX,NZ);
% reduced model, v3
sr_v3_h = zeros(NX,NY);
sr_v3_v = zeros(NX,NZ);
sr_v3_vv = zeros(NX,NZ);

for i = 1:NX    
    for j = 1:NY   
        if isnan(slice4(i,j))
            sf_v1_h(i,j) = NO_VAL;
            sf_v2_h(i,j) = NO_VAL;
            sf_v3_h(i,j) = NO_VAL;
            sr_v1_h(i,j) = NO_VAL;
            sr_v2_h(i,j) = NO_VAL;
            sr_v3_h(i,j) = NO_VAL;
            continue;
        end
        Kfi = reshape(KCf_h(i,j,:,:),3,3);
        Kri = reshape(KCr_h(i,j,:,:),3,3);
                
        [~,Sf,Vf] = svd(Kfi);
        [~,Sr,Vr] = svd(Kri);
        dK = Kfi-Kri;        
        % unit forces
        ff1 = Vf(:,1)/norm(Vf(:,1)); ff2 = Vf(:,2)/norm(Vf(:,2)); ff3 = Vf(:,3)/norm(Vf(:,3));
        fr1 = Vr(:,1)/norm(Vr(:,1)); fr2 = Vr(:,2)/norm(Vr(:,2)); fr3 = Vr(:,3)/norm(Vr(:,3));
        % difference
        sf_v1_h(i,j) = norm(dK * ff1); sf_v2_h(i,j) = norm(dK * ff2); sf_v3_h(i,j) = norm(dK * ff3);
        sr_v1_h(i,j) = norm(dK * fr1); sr_v2_h(i,j) = norm(dK * fr2); sf_v3_h(i,j) = norm(dK * fr3);        
    end
    for j = 1:NZ  
        if isnan(slice4v(i,j))
            sf_v1_v(i,j) = NO_VAL;
            sf_v2_v(i,j) = NO_VAL;
            sf_v3_v(i,j) = NO_VAL;
            sr_v1_v(i,j) = NO_VAL;
            sr_v2_v(i,j) = NO_VAL;
            sr_v3_v(i,j) = NO_VAL;
            continue;
        end
        Kfi = reshape(KCf_v(i,j,:,:),3,3);
        Kri = reshape(KCr_v(i,j,:,:),3,3);
                
        [~,Sf,Vf] = svd(Kfi);
        [~,Sr,Vr] = svd(Kri);
        dK = Kfi-Kri;        
        % unit forces
        ff1 = Vf(:,1)/norm(Vf(:,1)); ff2 = Vf(:,2)/norm(Vf(:,2)); ff3 = Vf(:,3)/norm(Vf(:,3));
        fr1 = Vr(:,1)/norm(Vr(:,1)); fr2 = Vr(:,2)/norm(Vr(:,2)); fr3 = Vr(:,3)/norm(Vr(:,3));
        % difference
        sf_v1_v(i,j) = norm(dK * ff1); sf_v2_v(i,j) = norm(dK * ff2); sf_v3_v(i,j) = norm(dK * ff3);
        sr_v1_v(i,j) = norm(dK * fr1); sr_v2_v(i,j) = norm(dK * fr2); sf_v3_v(i,j) = norm(dK * fr3);        
    end
    for j = 1:NZ  
        if isnan(slice4vv(i,j))
            sf_v1_vv(i,j) = NO_VAL;
            sf_v2_vv(i,j) = NO_VAL;
            sf_v3_vv(i,j) = NO_VAL;
            sr_v1_vv(i,j) = NO_VAL;
            sr_v2_vv(i,j) = NO_VAL;
            sr_v3_vv(i,j) = NO_VAL;
            continue;
        end
        Kfi = reshape(KCf_vv(i,j,:,:),3,3);
        Kri = reshape(KCr_vv(i,j,:,:),3,3);
                
        [~,Sf,Vf] = svd(Kfi);
        [~,Sr,Vr] = svd(Kri);
        dK = Kfi-Kri;        
        % unit forces
        ff1 = Vf(:,1)/norm(Vf(:,1)); ff2 = Vf(:,2)/norm(Vf(:,2)); ff3 = Vf(:,3)/norm(Vf(:,3));
        fr1 = Vr(:,1)/norm(Vr(:,1)); fr2 = Vr(:,2)/norm(Vr(:,2)); fr3 = Vr(:,3)/norm(Vr(:,3));
        % difference
        sf_v1_vv(i,j) = norm(dK * ff1); sf_v2_vv(i,j) = norm(dK * ff2); sf_v3_vv(i,j) = norm(dK * ff3);
        sr_v1_vv(i,j) = norm(dK * fr1); sr_v2_vv(i,j) = norm(dK * fr2); sf_v3_vv(i,j) = norm(dK * fr3);        
    end
end

% intersection
sf_v1_u = min(sf_v1_v, sf_v1_vv); sf_v2_u = min(sf_v2_v, sf_v2_vv); sf_v3_u = min(sf_v3_v, sf_v3_vv);
sr_v1_u = min(sr_v1_v, sr_v1_vv); sr_v2_u = min(sr_v2_v, sr_v2_vv); sr_v3_u = min(sr_v3_v, sr_v3_vv);

plot_contour(sf_v1_h*1E6,Y,X,'Y, m','X, m');

plot_contour(sf_v2_u*1E6,Z,X,'Z, m','X, m');