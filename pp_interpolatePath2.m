function trajectory = pp_interpolatePath2(path,vmax,segmentToIncrease,scalingFactor)
    
    numberOfSamples = 1000;

    orientations = zeros(length(path),1);

    path = [path, orientations];

    x = path(:,1);
    y = path(:,2);

    lengths = sqrt(diff(x).^2 + diff(y).^2);
    segment_times = lengths / vmax;
    t = [0, cumsum(segment_times)'];
    
    
    % Create a waypoint trajectory
    traj1 = waypointTrajectory(path,t);

    sampleTimes1 = linspace(t(1), t(end), numberOfSamples);
    [position1, orientation1, velocity1] = lookupPose(traj1,sampleTimes1);
    
    figure(1)
    plot(path(:,1),path(:,2),"o",position1(:,1), position1(:,2),".");

    plot(sampleTimes1,vecnorm(velocity1,2,2));
    xlabel("Time (s)");
    ylabel("Speed (m/s)");

end

