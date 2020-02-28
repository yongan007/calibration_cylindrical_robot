% load configuration
parameters;

%% find stiffness

theta = zeros(robot.theta_no,1);
 
N = robot.joint_no;
S1 = zeros(N, N);
S2 = zeros(N, 1);
robot.tool(1:3,4) = [0.5;0.5;0.5];

it = 0;

k0 = zeros(6,1);

% experiments
while true
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
    
    if it > 10 
        k1 = S1 \ S2; 
        dk = (k1-k0) ./ k1;
        if max(dk) < 0.0001
            break
        else
            k0 = k1;
        end
    end
    it = it+1;
        
end

% results
ks = S1 \ S2;          % compliance

stiffness = 1 ./ ks  % stiffness

it 
