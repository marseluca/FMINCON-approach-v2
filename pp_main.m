%% TRAJECTORY COMPUTATION

clc
clear all
close all
global nRobots samplingTime pathColors;

openfig("warehouse.fig");
paths = {};
paths = load("C:\Users\Luca\Desktop\Collaborative path\paths_registration.mat").paths_registration;

nRobots = size(paths,2);

for j=1:nRobots
    paths{j} = unique(paths{j},'rows','stable');
end

%% VARIABLES
robotSize = 20;
collisionThreshold = 20;
maxVelocity = 20;

%% OPTIONS
animation = true;
animVelocity = 8;
recordAnimation = true;
solveCollisions = true;
plotVelocities = true;
plotCollisions = true;

samplingTime = 0.1;

% Create random path colors
pathColors = distinguishable_colors(nRobots);


%% INTERPOLATION
trajectories = {};
for j=1:nRobots
    trajectories{j} = pp_interpolatePath(paths{j},maxVelocity,0,0);
end

%% COLLISION CHECKING
collisions = {};
for j=1:nRobots
    collisions{j} = pp_checkCollisionForOneRobot(paths,trajectories,collisionThreshold,j);
end

if plotCollisions
    pp_plotCollisions(collisions,trajectories);
end

finishTimes = [];
for j=1:nRobots
    finishTimes = [finishTimes, trajectories{j}.t_tot(end)];
end
finishTimes


%% COLLISIONS
if ~all(cellfun(@isempty,collisions)) && solveCollisions

    global maxVelocity collisionPosition slowedPath otherTraj collidingSegment collidingTime segmentStart;
    
    slowDownRobotIndex = 2;
    otherRobotIndex = 1;
    slowedPath = paths{slowDownRobotIndex};
    otherTraj = trajectories{otherRobotIndex};

    collidingSegment = collisions{slowDownRobotIndex}(1,3);
    collidingTime = collisions{slowDownRobotIndex}(1,4);
    segmentStart = slowedPath(collidingSegment,:);
    collisionPosition = collisions{slowDownRobotIndex}(1,5:6);
    
    pp_collisionOptimization;
    
    % Add a new segment inside the colliding segment
    paths{slowDownRobotIndex} = pp_addNewSegment(paths{slowDownRobotIndex},collidingSegment,0,x_opt(1));

    % Apply the velocity scaling to the new segment
    trajectories{slowDownRobotIndex} = pp_interpolatePath(paths{slowDownRobotIndex},maxVelocity,collidingSegment,x_opt(2));

end

pp_plotPathOnMap(paths,trajectories,'-');

if plotVelocities
    % Plot positions, velocities and accelerations
    pp_producePlots(trajectories,plotVelocities);
end

%% ANIMATION
if animation
    fprintf("\nPress enter to record animation with velocity %dx...\n",animVelocity);
    pp_animateTrajectory(trajectories,robotSize,recordAnimation,animVelocity);
end

% figure(1)
% saveas(gcf,'warehouse.png')




