% Define constants and inputs

global delta_s x_opt segmentStart collisionPosition;

maxSegmentLength = norm(segmentStart-collisionPosition);

delta_s = 20;

% Define the objective function: minimize total travel time
function total_time = objective(x)
    
    global maxVelocity slowedPath otherTraj collidingSegment 
    % Extract optimization variables
    L_s = x(1);  % Length of the slow-down segment
    alpha = x(2); % Scaling factor for slowing down

    path = slowedPath;
    path = pp_addNewSegment(path,collidingSegment,0,L_s);
    path = unique(path,'rows','stable');
    slowedTraj = pp_interpolatePath(path,maxVelocity,collidingSegment,alpha);

    total_time = max(slowedTraj.t_tot(end),otherTraj.t_tot(end));
    
    % Ensure total_time is scalar
    if numel(total_time) > 1
        error('Objective function must return a scalar value, but received an array.');
    end
end




% Define the collision avoidance constraint
function [c, ceq] = collision_constraint(x)
    
    global delta_s maxVelocity otherTraj slowedPath collidingSegment;

    L_s = x(1);
    alpha = x(2);
    
    % Time for the robot that slows down
    % time_robot1 = (L_s / (alpha * maxVelocity)) + ((collision_position - L_s) / maxVelocity);
    
    path = slowedPath;
    path = pp_addNewSegment(path,collidingSegment,0,L_s);
    path = unique(path,'rows','stable');
    slowedTraj = pp_interpolatePath(path,maxVelocity,collidingSegment,alpha);

    c = [];

    % So as to have always the same number of constraints
    numberOfConstraints = 500;

    minLength = min(length(slowedTraj.x_tot),length(otherTraj.x_tot));

    for j=1:numberOfConstraints

        if j<=minLength
            slowedPos = [slowedTraj.x_tot(j),slowedTraj.y_tot(j)];
            otherPos = [otherTraj.x_tot(j),otherTraj.y_tot(j)];
            c = [c; delta_s - norm(slowedPos-otherPos)];
        else
            c = [c; 0];
        end
    end

    % time_robot1 = slowedTraj.t_tot(collisionIndex);
    % 
    % % Time for the other robot (assumed to move at full speed)
    % time_robot2 = collidingTime;
    
    % Collision constraint
    % T1 >= DELTA + T2 così da rallentare T1 ed evitare la collisione
    % Quindi T1 - DELTA - T2 >= 0 --> c = T2 + DELTA - T1 <= 0 
    % c = delta_t - time_robot1 + time_robot2 % Time buffer to avoid collision
    ceq = [];
end

% Initial guess for the optimization variables [L_s, alpha]
x0 = [maxSegmentLength / 2, 0.5];

% Bounds: 0 < alpha <= 1, and L_s should be between 0 and the collision position
lb = [0, 0.1]; % Lower bounds for [L_s, alpha]
ub = [maxSegmentLength, 1]; % Upper bounds for [L_s, alpha]

% Set optimization options
options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp');
options = optimoptions('fmincon', 'TolFun', 1e-4, 'TolCon', 1e-4, 'TolX', 1e-6, 'Display', 'final');



% Call the fmincon solver
[x_opt, fval] = fmincon(@objective, x0, [], [], [], [], lb, ub, @collision_constraint, options);

% Call the collision constraint function with the optimal solution
[c_val, ceq_val] = collision_constraint(x_opt);

% Output results
fprintf('Ls: %f\n', x_opt(1));
fprintf('alpha: %f\n', x_opt(2));
fprintf('Minimized travel time: %f\n', fval);

% Output the value of c
% fprintf('Value of c (constraint): %f\n', c_val);
