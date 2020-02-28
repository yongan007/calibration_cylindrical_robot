%% link_test
%  Stiffness identification

% include 'robot' structure
parameters;   

theta = zeros(robot.theta_no,1);

N = robot.joint_no;
S1 = zeros(N, N);
S2 = zeros(N, 1);
dsum = 0;

iter = 500;   % nomber or experiments

poses = zeros(robot.joint_no, iter);
forces = zeros(6, iter);
disp_befor = zeros(iter,1);
disp_after = zeros(iter,1);

% experiments
for i = 1:iter
    % random force values
    force = zeros(6,1);
    force(1:3) = 2*rand(3,1)-1;    
    force = (force / norm(force)) * 1000;
    %force(1:3) = [0;0;-1000];
    forces(:,i) = force;
    % random joint values
    q0 = (pi/2)*(2*rand(robot.joint_no,1) - 1);
    %q0 = [0.01 0.01 0.01 0.01 0.01 0.01];
    %q0 = zeros(6,1);
    poses(:,i) = q0;
    % displacement    
    [dt,dq] = delta_ee(q0,theta,force,robot);
    disp = norm(dt);
    disp_befor(i) = disp;
    dsum = dsum + disp;
    % use only cartesian displacement
    dt = dt(1:3);
    A = get_A(q0, theta, force, robot);  
    
    % accumulate sum    
    S1 = S1 + A' * A;
    S2 = S2 + A' * dt;      
end

% results
% compliance
ks = S1 \ S2;


%kk = robot.Kq
% stiffness
stiffness = 1 ./ ks

deflection = dsum / iter
% {
% histogram(disp_befor, 'Normalization', 'probability');
% 
% robot.Kq = stiffness;
% 
% for i = 1:iter
%     dt = delta_ee(poses(:,i), theta, forces(:,i), robot);
%     disp_after(i) = norm(dt);    
% end
% 
% figure;
% histogram(disp_after, 'Normalization', 'probability');
% }