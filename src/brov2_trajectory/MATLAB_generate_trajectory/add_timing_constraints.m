function trajectory_constraints = add_timing_constraints(start_time,                ...
                                                         surge_velocity,            ...
                                                         turn_velocity_percentage,  ...
                                                         turn_radius,               ...
                                                         path_constraints,          ...
                                                         path_type)
    % This function adds timing constraints to array of path constraints 

    % Input-
    % start_time                : Amount of time the vehicle can use to get
    %                             to the starting point
    % surge_velocity            : Surge velocity used for generating path
    %                             constraints
    % turn_velocity_percentage  : Percentage of surge velocity to keep
    %                             through turns used for generating path
    %                             constraints
    % turn_radius               : Radius of each turn in the trajectory
    % path_constraints          : Already generated constraints
    % path_type                 : Valid types of paths: ["out-n-back",
    %                             "side-to-side"]
    % 
    % Output-
    % trajectory_constraints    : Constraints defining the trajectory with
    %                             timing constraints. Feed into trajectory
    %                             generator.
    %%

    
    toa_straight = @(cp,pp) max((sqrt( (cp(1)-pp(1))^2 + (cp(2)-pp(2))^2 + (cp(3)-pp(3))^2 ) / surge_velocity), 0.0001);
    toa_turn = (pi*turn_radius) / (surge_velocity*turn_velocity_percentage/100); 
    
    if path_type == "out-n-back"
        
        trajectory_constraints = [start_time, path_constraints(1,:)];
        % Constraint definition: Time of arrival, Waypoints, Velocities, Orientation
        for i=2:(size(path_constraints,1)-1)
            previous_time = trajectory_constraints(i-1,1);
            if mod(i,2) == 0
                previous_point = path_constraints(i-1,1:3); 
                current_point = path_constraints(i,1:3);
                trajectory_constraints = [trajectory_constraints; ...
                    [previous_time + toa_straight(current_point, previous_point), path_constraints(i,:)]];
            else
                trajectory_constraints = [trajectory_constraints;...
                [previous_time + toa_turn, path_constraints(i,:)]];
            end
        end
        previous_time = trajectory_constraints(i,1);
        end_point_index = size(path_constraints,1);
        previous_point = path_constraints(end_point_index-1,1:3);
        end_point = path_constraints(end_point_index,1:3);
        trajectory_constraints = [trajectory_constraints; ...
                    [previous_time + 2*toa_straight(end_point, previous_point), path_constraints(end_point_index,:)]]
                
    elseif path_type == "side-to-side"
        
        trajectory_constraints = [start_time, path_constraints(1,:)];
        trajectory_constraints = [trajectory_constraints;[start_time + toa_turn, path_constraints(2,:)]];
        trajectory_constraints = [trajectory_constraints;...
            [trajectory_constraints(end,1)+toa_straight(path_constraints(3,1:3),path_constraints(2,1:3)),path_constraints(3,:)]];
            
        % Constraint definition: Time of arrival, Waypoints, Velocities, Orientation
        for i=4:(size(path_constraints,1)-2)
            previous_time = trajectory_constraints(i-1,1);
            if mod(i,2) == 0
                trajectory_constraints = [trajectory_constraints;...
                [previous_time + toa_turn, path_constraints(i,:)]];
            else
                previous_point = path_constraints(i-1,1:3);
                current_point = path_constraints(i,1:3);
                trajectory_constraints = [trajectory_constraints; ...
                    [previous_time + toa_straight(current_point, previous_point), path_constraints(i,:)]];
            end
        end
        % Smoothing up the return path
        return_point_index = size(path_constraints,1)-1;
        previous_point = path_constraints(return_point_index-2,1:3);
        end_point = path_constraints(return_point_index,1:3);
        return_time = previous_time + 2*toa_straight(end_point, previous_point);
        trajectory_constraints = [trajectory_constraints; ...
                    [return_time, path_constraints(return_point_index,:)]];
                
        end_point_index = size(path_constraints,1);
        previous_point = path_constraints(end_point_index-1,1:3);
        end_point = path_constraints(end_point_index,1:3);
        trajectory_constraints = [trajectory_constraints; ...
                    [return_time + 2*toa_straight(end_point, previous_point), path_constraints(end_point_index,:)]];
         
    else
        error('Use a valid trajectory type.')
    end        
end