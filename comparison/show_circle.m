% load configuration
close all
parameters;

%% find stiffness

theta = zeros(robot.theta_no,1);
 
N = robot.joint_no;
S1 = zeros(N, N);
S2 = zeros(N, 1);

iter = 600;   % nomber or experiments

robot.tool(1:3,4) = [0.5;0.5;0.5];

% 4.5
X1 = 1.21;
Y1 = 1.21;
% 5.13
X2 = 1.45;
Y2 = 1.45;
% 9.7
X3 = 2.27;
Y3 = 0.22;

% experiments
for i = 1:iter
    % random force values
    force = zeros(6,1);
    force(1:3) = 2*rand(3,1)-1;    
    force = (force / norm(force)) * 1000;    
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

kq_backup = robot.Kq;

%% Trajectory

CONF = 5;
CONF2 = 6;
ZLVL = 0.3;
YLVL = 0;


m = [2*mod(floor(CONF/4),2)-1,2*mod(floor(CONF/2),2)-1,2*mod(CONF,2)-1];
m2 = [2*mod(floor(CONF2/4),2)-1,2*mod(floor(CONF2/2),2)-1,2*mod(CONF2,2)-1];

force = [1000;1000;1000;0;0;0];

R = 0.001; 
X0 = 1.6;
Y0 = 0;

NP = 100;

alpha = linspace(0, 2*pi, NP);

JSh = zeros(6,NP);
JSv = zeros(6,NP);

XX = X0+R*cos(alpha);
YY = Y0+R*sin(alpha);

for i = 1:NP
    conf = IK([XX(i);YY(i);ZLVL;0;0;0], m, robot);
    %conf(4:6) = [0;0;0];
    JSh(:,i) = conf;
    
    conf = IK([XX(i);YLVL;YY(i);0;0;0], m2, robot);
    %conf(4:6) = [0;0;0];
    JSv(:,i) = conf;    
end

base = [XX',YY',ZLVL*ones(NP,1)];
plot3(base(:,1),base(:,2),base(:,3));
xlabel('X,m'); ylabel('Y,m'); zlabel('Z,m');
figure;

%% Joints + links 

circle1 = zeros(NP,3);
 
for i = 1:NP
    q = JSh(:,i);
    del = delta_ee(q, theta, force, robot);
    %circle1(i) = norm(del(1:3));
    circle1(i,:) = (del(1:3)+[XX(i);YY(i);ZLVL])';
end

plot3(circle1(:,1),circle1(:,2),circle1(:,3), base(:,1),base(:,2),base(:,3));
xlabel('X,m'); ylabel('Y,m'); zlabel('Z,m');

disp('Joints + Links');

[X1,Y1,ZLVL,0,0,0]

q1 = IK([X1,Y1,ZLVL,0,0,0],m,robot)
%q1(4:6) = [0;0;0];
xyz1 = FK(q1,robot)
[~,~,kci] = delta_svd(q1,0,robot);
Kc1 = inv(kci)
[u1,s1,v1] = svd(kci)

[X2,Y2,ZLVL,0,0,0]

q2 = IK([X2,Y2,ZLVL,0,0,0],m,robot)
q2(4:6) = [0;0;0];
xyz2 = FK(q2,robot);
[~,~,kci] = delta_svd(q2,0,robot);
Kc2 = inv(kci)
[u2,s2,v2] = svd(kci)

[X3,Y3,ZLVL,0,0,0]

q3 = IK([X3,Y3,ZLVL,0,0,0],m,robot)
q3(4:6) = [0;0;0];
xyz3 = FK(q3,robot);
[~,~,kci] = delta_svd(q3,0,robot);
Kc3 = inv(kci)
[u3,s3,v3] = svd(kci)


%% Estimation
robot.Kq = stiffness;
robot.use_links = false;

circle2 = zeros(NP,3);
 
for i = 1:NP
    q = JSh(:,i);
    del = delta_ee(q, theta, force, robot);
    %circle1(i) = norm(del(1:3));
    circle2(i,:) = (del(1:3)+[XX(i);YY(i);ZLVL])';
end

figure;
plot3(circle2(:,1),circle2(:,2),circle2(:,3));

disp('Estimation');

%[X1,Y1,ZLVL,0,0,0]

%q1e = IK([X1,Y1,ZLVL,0,0,0],m,robot);
%xyz1e = FK(q1e,robot);
[~,~,kci] = delta_svd(q1,0,robot);
Kc1e = inv(kci)
[u1e,s1e,v1e] = svd(kci)

%[X2,Y2,ZLVL,0,0,0]

%q2e = IK([X2,Y2,ZLVL,0,0,0],m,robot);
%xyz2e = FK(q2e,robot);
[~,~,kci] = delta_svd(q2,0,robot);
Kc2e = inv(kci)
[u2e,s2e,v2e] = svd(kci)

%[X3,Y3,ZLVL,0,0,0]

%q3e = IK([X3,Y3,ZLVL,0,0,0],m,robot);
%xyz3e = FK(q3e,robot);
[~,~,kci] = delta_svd(q3,0,robot);
Kc3e = inv(kci)
[u3e,s3e,v3e] = svd(kci)

%% Combine

%figure;
%plot3(circle1(:,1),circle1(:,2),circle1(:,3),circle2(:,1),circle2(:,2),circle2(:,3),XX,YY,ZLVL*ones(NP,1));

%% New trajectory

diff = base - circle2;
upd = base + diff;

robot.Kq = kq_backup;
robot.use_links = true;

circle4 = zeros(size(upd));
for i = 1:NP
    q = JSh(:,i);
    del = delta_ee(q,theta,force,robot);
    circle4(i,:) = del(1:3)' + upd(i,:);
end

figure;
plot3(circle4(:,1),circle4(:,2),circle4(:,3),base(:,1),base(:,2),base(:,3),upd(:,1),upd(:,2),upd(:,3));
xlabel('X,m'); ylabel('Y,m'); zlabel('Z,m');

