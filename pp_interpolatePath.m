function trajectory = pp_interpolatePath(path,vmax,segmentToIncrease,scalingFactor)
    
    numberOfSamples = 10000;
    interpolatedPoints = {};
    trajectory = struct();
    hold on

    x = path(:,1);
    y = path(:,2);
    
    segment = 1;

    lengths = sqrt(diff(x).^2 + diff(y).^2);
    segment_times = lengths / vmax;

    % Increase segment time
    if segmentToIncrease~=0
        segment_times(segmentToIncrease) = lengths(segmentToIncrease) / (scalingFactor*vmax);
    end
    
    t = [0, cumsum(segment_times)'];

    spline_x = pchip(t,x);
    spline_y = pchip(t,y);

    trajectory.t_tot = linspace(t(1), t(end), numberOfSamples);

    % Evaluate the splines
    trajectory.x_tot = ppval(spline_x, trajectory.t_tot);
    trajectory.y_tot = ppval(spline_y, trajectory.t_tot);    


    % === Compute the velocity ===
    % Differentiate the pchip splines to get velocity
    velocity_x_spline = fnder(spline_x);
    velocity_y_spline = fnder(spline_y);
    
    % Evaluate the velocity components at the time points
    trajectory.xdot_tot = ppval(velocity_x_spline, trajectory.t_tot);
    trajectory.ydot_tot = ppval(velocity_y_spline, trajectory.t_tot);

    % Compute the velocity magnitude at each time step
    velocity_magnitude = sqrt(trajectory.xdot_tot.^2 + trajectory.ydot_tot.^2);
    
    % Find the maximum velocity encountered
    max_encountered_velocity = max(velocity_magnitude);
    
    % Check if the encountered velocity exceeds the max allowed velocity
    if max_encountered_velocity > vmax
        % Scale the maximum velocity down to avoid collisions
        scaling_factor = vmax / max_encountered_velocity;

        vmax = scaling_factor*vmax;

        segment = 1;

        lengths = sqrt(diff(x).^2 + diff(y).^2);
        segment_times = lengths / vmax;

        % Increase segment time
        if segmentToIncrease~=0
            segment_times(segmentToIncrease) = lengths(segmentToIncrease) / (scalingFactor*vmax);
        end

        
        t = [0, cumsum(segment_times)'];

        spline_x = pchip(t,x);
        spline_y = pchip(t,y);

        trajectory.t_tot = linspace(t(1), t(end), numberOfSamples);

        % Evaluate the splines
        trajectory.x_tot = ppval(spline_x, trajectory.t_tot);
        trajectory.y_tot = ppval(spline_y, trajectory.t_tot);

        % === Compute the velocity ===
        % Differentiate the pchip splines to get velocity
        velocity_x_spline = fnder(spline_x);
        velocity_y_spline = fnder(spline_y);

        % Evaluate the velocity components at the time points
        trajectory.xdot_tot = ppval(velocity_x_spline, trajectory.t_tot);
        trajectory.ydot_tot = ppval(velocity_y_spline, trajectory.t_tot);

    end
    

    % === Compute the acceleration ===
    % Differentiate the velocity splines to get acceleration
    acceleration_x_spline = fnder(velocity_x_spline); % Second derivative of position (x)
    acceleration_y_spline = fnder(velocity_y_spline); % Second derivative of position (y)
    
    % Evaluate the acceleration components at the time points
    trajectory.xddot_tot = ppval(acceleration_x_spline, trajectory.t_tot);
    trajectory.yddot_tot = ppval(acceleration_y_spline, trajectory.t_tot);

    trajectory = pp_commonTimeSampling(trajectory);
    
end

