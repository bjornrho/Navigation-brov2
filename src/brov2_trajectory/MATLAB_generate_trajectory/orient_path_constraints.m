function oriented_path_constraints = orient_path_constraints(start_angle,           ...
                                                             turn_radius,           ...
                                                             line_distance,          ...
                                                             path_constraints,      ...
                                                             path_type)
    % This function reorients a path given by path constraints with the angle (degrees) given 

    % Input-
    % start_angle                 : Angle (degrees) with which to reorient
    %                               path constraints
    % turn_radius                 : Radius of each turn in the trajectory
    % line_distance               : Distance of the straight parts
    % path_constraints            : Path constraints to be oriented
    % path_type                   : Valid types of paths: ["out-n-back",
    %                               "side-to-side"]
    
    % 
    % Output-
    % oriented_path_constraints   : Resulting path constraints skewed with
    %                               start angle in NED frame along z-axis
    %                               (positive - clockwise)
    %%
    start_wp_index  = 1:3;
    wp_index        = 1:2;
    vel_index       = 4:5;
    ori_index       = 7:9;      % Only changes to yaw
    
    if path_type == "out-n-back"
         for i=1:(size(path_constraints,1))
            if i == 1 || i == size(path_constraints,1)
                % Changes to start point
                oriented_path_constraints(i,start_wp_index)     = path_constraints(i,start_wp_index);
                oriented_path_constraints(i,vel_index)          = [cosd(start_angle)*path_constraints(i,4),...
                                                                   sind(start_angle)*path_constraints(i,5)];
                
            else
                oriented_path_constraints(i,wp_index)    = [path_constraints(i,1)*cosd(start_angle)-path_constraints(i,2)*sind(start_angle),...
                                                   path_constraints(i,1)*sind(start_angle)+path_constraints(i,2)*cosd(start_angle)];
                oriented_path_constraints(i,vel_index)   = [path_constraints(i,4)*cosd(start_angle)-path_constraints(i,5)*sind(start_angle),...
                                                   path_constraints(i,4)*sind(start_angle)+path_constraints(i,5)*cosd(start_angle)];
                
            end
            oriented_path_constraints(i,ori_index) = [path_constraints(i,7)+start_angle,0,0];
        end

    elseif path_type == "side-to-side"
        for i=1:(size(path_constraints,1))
            if i == 1 || i == size(path_constraints,1)
                % Changes to start points
                oriented_path_constraints(i,start_wp_index)     = path_constraints(i,start_wp_index);
                oriented_path_constraints(i,vel_index)          = [cosd(start_angle)*path_constraints(i,4),...
                                                                   sind(start_angle)*path_constraints(i,5)];
            else
                oriented_path_constraints(i,wp_index)    = [path_constraints(i,1)*cosd(start_angle)-path_constraints(i,2)*sind(start_angle),...
                                                   path_constraints(i,1)*sind(start_angle)+path_constraints(i,2)*cosd(start_angle)];
                oriented_path_constraints(i,vel_index)   = [path_constraints(i,4)*cosd(start_angle)-path_constraints(i,5)*sind(start_angle),...
                                                   path_constraints(i,4)*sind(start_angle)+path_constraints(i,5)*cosd(start_angle)];                
            end
            oriented_path_constraints(i,ori_index) = [path_constraints(i,7)+start_angle,0,0];
        end
    else
        error('Use a valid trajectory type.')
    end
end