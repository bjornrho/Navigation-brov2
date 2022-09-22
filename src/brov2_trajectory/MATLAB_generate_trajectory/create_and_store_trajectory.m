function path_constraints  = create_and_store_trajectory(trajectory_file_name,           ...
                                                         trajectory_period,              ...
                                                         live_plot,                      ...
                                                         trajectory_constraints,         ...
                                                         turn_radius)
    % This function generates trajectory constraints for side to side trajectory 

    % Input-
    % trajectory_file_name      : The file name of under which to store the
    %                             trajectory
    % trajectory_period         : The inverted sample rate for the
    %                             trajectory (must coincide with
    %                             trajectory_period in
    %                             trajectory_publisher)
    % live_plot                 : Bool allowing live plot as trajectory is
    %                             generated
    % trajectory_constraints    : The constraints of which the trajectory
    %                             must adhere
    % turn_radius               : Radius of each turn in the trajectory
    %%

    if not(trajectory_file_name(end-3:end) == '.csv')
        error('Make sure the trajectory_file_name ends with .csv')
    end
    
    rotate3d on;
    trajectory_constraints = trajectory_constraints

    trajectory = waypointTrajectory(trajectory_constraints(:,2:4), ...
                         'TimeOfArrival',trajectory_constraints(:,1), ...
                         'Velocities',trajectory_constraints(:,5:7), ...
                         'Orientation',quaternion(trajectory_constraints(:,8:10),'eulerd','ZYX','frame'), ...
                         'SampleRate',1/trajectory_period);

    tInfo = waypointInfo(trajectory)

    % Plotting
    fig = figure(1);
    plot3(tInfo.Waypoints(1,1),tInfo.Waypoints(1,2),tInfo.Waypoints(1,3),'b*') 
    axis([min(trajectory_constraints(:,2))-3*turn_radius,max(trajectory_constraints(:,2))+3*turn_radius,...
          min(trajectory_constraints(:,3))-3*turn_radius,max(trajectory_constraints(:,3))+3*turn_radius,...
          min(trajectory_constraints(:,4))-1,max(trajectory_constraints(:,4))+1]);

    xlabel('North')
    ylabel('East')
    grid on
    daspect([1 1 1])
    set(gca, 'ZDir','reverse')
    set(gca, 'YDir','reverse')

    hold on

    % Accumulating trajectory
    pos     = zeros(int64(tInfo.TimeOfArrival(end)*trajectory.SampleRate),3);
    orient  = zeros(int64(tInfo.TimeOfArrival(end)*trajectory.SampleRate),1,'quaternion');
    vel     = zeros(int64(tInfo.TimeOfArrival(end)*trajectory.SampleRate),3);
    acc     = zeros(int64(tInfo.TimeOfArrival(end)*trajectory.SampleRate),3);
    angVel  = zeros(int64(tInfo.TimeOfArrival(end)*trajectory.SampleRate),3);
    angAcc  = zeros(int64(tInfo.TimeOfArrival(end)*trajectory.SampleRate),3);

    count = 1;
    while ~isDone(trajectory)
        [pos(count,:),orient(count),vel(count,:),acc(count,:),angVel(count,:)] = trajectory();
        plot3(pos(count,1),pos(count,2),pos(count,3),'bo')
        if live_plot
            pause(trajectory.SamplesPerFrame/trajectory.SampleRate)
        end
        count = count + 1;
    end

    % Getting angular acceleration
    roughAngAcc = diff(angVel,1,1)/(tInfo.TimeOfArrival(end)/(tInfo.TimeOfArrival(end)*trajectory.SampleRate));
    roughAngAcc = [zeros(1,3); roughAngAcc];

    % Write to csv-file (if relative path is not working use full path
    [real,i,j,k] = parts(orient);
    trajectory_matrix = [pos,real,i,j,k,vel,acc,angVel,roughAngAcc];
    trajectory_matrix(end,:) = []; % pop last row which doesn't contain trajectory data
    relative_path = split(pwd,"/");
    relative_path(length(relative_path)-2:end) = [];
    relative_path{end+1} = join(['trajectories/',trajectory_file_name]);
    file_path = join(relative_path,"/");    
    writematrix(trajectory_matrix,file_path{1},'Delimiter','comma')
    
    success_msg = " successfully created and stored in '/Navigation-brov2/trajectories' folder.";
    disp("<strong>" + trajectory_file_name + success_msg + "</strong>")
    
end