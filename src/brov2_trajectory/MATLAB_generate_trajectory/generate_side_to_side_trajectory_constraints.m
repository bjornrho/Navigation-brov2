function path_constraints  = generate_side_to_side_trajectory_constraints(start_point,           ...
                                                                          start_angle,           ...
                                                                          end_point,                ...
                                                                          surge_velocity,           ...
                                                                          turn_velocity_percentage, ...
                                                                          number_of_lines,          ...
                                                                          turn_radius,              ...
                                                                          line_distance,            ...
                                                                          turn_direction)
    % This function generates trajectory constraints for side to side trajectory 

    % Input-
    % starting_point            : Starting point of the trajectory
    % starting_angle            : Angle determining trajectory orientation
    %                             (grades)
    % end_point                 : End point of the trajectory
    % surge_velocity            : Surge velocity through 
    % turn_velocity_percentage  : Percentage of surge velocity to keep
    %                             through turns
    % number_of_lines           : Number of lines in the trajectory
    % turn_radius               : Radius of each turn in the trajectory
    % line_distance             : Distance of the straight parts
    % turn_direction            : Direction of turns. [right, left]
    % 
    % Output-
    % path_constraints          : Constraints defining the path (no timing
    %                             constraints). Feed into
    %                             add_timing_constraints()
    %%

    velocity_still          = [0,0,0];
    orientation_straight    = [0,0,0];
    
    if turn_direction == "right"
        sign = 1;
    elseif turn_direction == "left"
        sign = -1;
    else
        error('Use valid direction: [right, left]')
    end
    
    % First turn and half line to initiate path
    path_constraints = [start_point, velocity_still, orientation_straight];
    path_constraints = [path_constraints;...
        [start_point(1)+turn_radius, start_point(2)+sign*turn_radius,                    0,...
         0,                          sign*surge_velocity*(turn_velocity_percentage/100), 0,...
         sign*90,                    0,                                                  0]];
    path_constraints = [path_constraints;...
        [path_constraints(end,1),path_constraints(end,2)+sign*(line_distance/2-turn_radius),0,...
        0,                       sign*surge_velocity*(turn_velocity_percentage/100),      0,...
        sign*90,                 0,                                                       0]];
    % Constraint definition: Waypoints, Velocities, Orientation
    for line_n = 1:number_of_lines
        if mod(line_n,2) == 0
            % Add turn
            next_turn_point         = path_constraints(end,1:3) + [turn_radius, 0, 0];
            next_turn_velocity      = [0, sign*surge_velocity*(turn_velocity_percentage/100), 0];
            next_turn_orientation   = [sign*90, 0, 0];
            path_constraints = [path_constraints;...
                                [next_turn_point, next_turn_velocity, next_turn_orientation]];
            % Add line
            next_point       = path_constraints(end,1:3) + [0,   (sign*line_distance), 0];
            next_velocity    = [0, sign*surge_velocity*(turn_velocity_percentage/100), 0];
            next_orientation = [sign*90, 0, 0];
            path_constraints = [path_constraints;...
                                [next_point, next_velocity, next_orientation]];      
        else
            % Add turn
            next_turn_point         = path_constraints(end,1:3) + [turn_radius, 0, 0];
            next_turn_velocity      = [0, -sign*surge_velocity*(turn_velocity_percentage/100), 0];
            next_turn_orientation   = [-sign*90, 0, 0];
            path_constraints = [path_constraints;...
                                [next_turn_point, next_turn_velocity, next_turn_orientation]];
            
            % Add line
            next_point       = path_constraints(end,1:3) + [0, (-sign*line_distance), 0];
            next_velocity    = [0, -sign*surge_velocity*(turn_velocity_percentage/100), 0];
            next_orientation = [-sign*90, 0, 0];
            path_constraints = [path_constraints;...
                                [next_point, next_velocity, next_orientation]];
            
        end
    end
    % Smoothing up the return path
    path_constraints = [path_constraints;...
        [path_constraints(end,1:3)/2,[-surge_velocity,0,0],[180,0,0]]];
    path_constraints = [path_constraints;[end_point,velocity_still,orientation_straight]];
end