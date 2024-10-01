function plots = pp_producePlots(trajectories,flag)
    
    global nRobots pathColors;
    
    if flag

        figure(2)
        maxLength = min(length(trajectories{1}.x_tot),length(trajectories{2}.x_tot));
        distances = sqrt((trajectories{1}.x_tot(1:maxLength)-trajectories{2}.x_tot(1:maxLength)).^2+(trajectories{1}.y_tot(1:maxLength)-trajectories{2}.y_tot(1:maxLength)).^2);
        plot(trajectories{1}.t_tot(1:maxLength),distances,'LineWidth',1.2);
        hold on
        plot(trajectories{1}.t_tot(1:maxLength),20*ones(1,maxLength));
        grid
        xlabel("t [s]")
        ylabel("$d(t)\:[m]$",'Interpreter','latex')
        title("Distance between R1 and R2 over time")
        legend("","Safety margin")
        hold off

        for j=1:nRobots

            figure
    
            subplot(2,2,1)
            sgtitle("Robot "+j)
    
            plot(trajectories{j}.t_tot,trajectories{j}.x_tot,'LineWidth',1.2,'Color',pathColors(j,:));
            grid
            xlabel("t [s]")
            ylabel("$x(t)\:[m]$",'Interpreter','latex')
    
    
            subplot(2,2,2)
            plot(trajectories{j}.t_tot,trajectories{j}.y_tot,'LineWidth',1.2,'Color',pathColors(j,:));
            grid
            xlabel("t [s]")
            ylabel("$y(t)\:[m]$",'Interpreter','latex')
   

            subplot(2,2,3)
            velocity_magnitude = sqrt(trajectories{j}.xdot_tot.^2 + trajectories{j}.ydot_tot.^2);
            plot(trajectories{j}.t_tot,velocity_magnitude,'LineWidth',1.2,'Color',pathColors(j,:));
            grid
            xlabel("t [s]")
            ylabel("$v(t)\:[m/s^2]$",'Interpreter','latex')

            subplot(2,2,4)
            acc_magnitude = sqrt(trajectories{j}.xddot_tot.^2 + trajectories{j}.yddot_tot.^2);
            plot(trajectories{j}.t_tot,acc_magnitude,'LineWidth',1.2,'Color',pathColors(j,:));
            grid
            xlabel("t [s]")
            ylabel("$a(t)\:[m/s^2]$",'Interpreter','latex')

        end

    end
    
end

