clc
clear
%% Generating waypoints and constraints for horizontal trajectory
surge_vel = 1; %[m/s]
turn_diam = 10; %[m]
half_dist = 100/2; %[m]

% Waypoint definitions
origo = [0,0,0];
start_offshore = [5,0,-2];
wp_pair_1  = [20,                -half_dist, -5,     20+turn_diam,      -half_dist, -5];
wp_pair_2  = [20+turn_diam,       half_dist, -10,    20+2*turn_diam,     half_dist, -12.5];
wp_pair_3  = [20+2*turn_diam,    -half_dist, -12.5,  20+3*turn_diam,    -half_dist, -15];
wp_pair_4  = [20+3*turn_diam,     half_dist, -20,    20+4*turn_diam,     half_dist, -20];
wp_pair_5  = [20+4*turn_diam,    -half_dist, -22,    20+5*turn_diam,    -half_dist, -22];
wp_pair_6  = [20+5*turn_diam,     half_dist, -22,    20+6*turn_diam,     half_dist, -24];
wp_pair_7  = [20+6*turn_diam,    -half_dist, -24,    20+7*turn_diam,    -half_dist, -24];
wp_pair_8  = [20+7*turn_diam,     half_dist, -24,    20+8*turn_diam,     half_dist, -24];
wp_pair_9  = [20+8*turn_diam,    -half_dist, -24,    20+9*turn_diam,    -half_dist, -24];
wp_pair_10 = [20+9*turn_diam,     half_dist, -24,    20+10*turn_diam,    half_dist, -24];
wp_pair_11 = [20+10*turn_diam,   -half_dist, -24,    20+11*turn_diam,   -half_dist, -24];
wp_pair_12 = [20+11*turn_diam,    half_dist, -24,    20+12*turn_diam,    half_dist, -24];
stop_offshore = [wp_pair_12(4),   8,         wp_pair_12(6)];

% Time of Arrival definitions
turn_offset = 10/surge_vel;
start_time = 0;
toa = @(cp,np) (sqrt( (np(1)-cp(1))^2 + (np(2)-cp(2))^2 + (np(3)-cp(3))^2 ) / surge_vel);

time_1 = start_time + toa(origo, start_offshore);
time_2 = time_1 + toa(start_offshore,       wp_pair_1(1:3));
time_3 = time_2 + toa(wp_pair_1(1:3),       wp_pair_1(4:end)) + turn_offset;
time_4 = time_3 + toa(wp_pair_1(4:end),     wp_pair_2(1:3));
time_5 = time_4 + toa(wp_pair_2(1:3),       wp_pair_2(4:end)) + turn_offset;
time_6 = time_5 + toa(wp_pair_2(4:end),     wp_pair_3(1:3));
time_7 = time_6 + toa(wp_pair_3(1:3),       wp_pair_3(4:end)) + turn_offset;
time_8 = time_7 + toa(wp_pair_3(4:end),     wp_pair_4(1:3));
time_9 = time_8 + toa(wp_pair_4(1:3),       wp_pair_4(4:end)) + turn_offset;
time_10 = time_9 + toa(wp_pair_4(4:end),    wp_pair_5(1:3));
time_11 = time_10 + toa(wp_pair_5(1:3),     wp_pair_5(4:end)) + turn_offset;
time_12 = time_11 + toa(wp_pair_5(4:end),   wp_pair_6(1:3));
time_13 = time_12 + toa(wp_pair_6(1:3),     wp_pair_6(4:end)) + turn_offset;
time_14 = time_13 + toa(wp_pair_6(4:end),   wp_pair_7(1:3));
time_15 = time_14 + toa(wp_pair_7(1:3),     wp_pair_7(4:end)) + turn_offset;
time_16 = time_15 + toa(wp_pair_7(4:end),   wp_pair_8(1:3));
time_17 = time_16 + toa(wp_pair_8(1:3),     wp_pair_8(4:end)) + turn_offset;
time_18 = time_17 + toa(wp_pair_8(4:end),   wp_pair_9(1:3));
time_19 = time_18 + toa(wp_pair_9(1:3),     wp_pair_9(4:end)) + turn_offset;
time_20 = time_19 + toa(wp_pair_9(4:end),   wp_pair_10(1:3));
time_21 = time_20 + toa(wp_pair_10(1:3),    wp_pair_10(4:end)) + turn_offset;
time_22 = time_21 + toa(wp_pair_10(4:end),  wp_pair_11(1:3));
time_23 = time_22 + toa(wp_pair_11(1:3),    wp_pair_11(4:end)) + turn_offset;
time_24 = time_23 + toa(wp_pair_11(4:end),  wp_pair_12(1:3));
time_25 = time_24 + toa(wp_pair_12(1:3),    wp_pair_12(4:end)) + turn_offset;
time_26 = time_25 + toa(wp_pair_12(4:end),  stop_offshore);
time_27 = time_26 + toa(stop_offshore,      start_offshore);
end_time = time_27 + toa(start_offshore,    origo);

% Velocity definitions
forward_vel = [surge_vel,0,0];
backward_vel = [-surge_vel,0,0];
left_vel = [0,surge_vel,0];
right_vel = [0,-surge_vel,0];

% Orientation definitions
forward_ori = [0,0,0];
backward_ori = [180,0,0];
left_ori = [-90,0,0];
right_ori = [90,0,0];


% Constraint definition: Time of arrival, Waypoints, Velocities
horizontal_constraints = [start_time,   origo,              forward_vel,    forward_ori;
                          time_1,       start_offshore,     forward_vel,    forward_ori;
                          time_2,       wp_pair_1(1:3),     right_vel,      right_ori;
                          time_3,       wp_pair_1(4:end),   left_vel,       left_ori;
                          time_4,       wp_pair_2(1:3),     left_vel,       left_ori;
                          time_5,       wp_pair_2(4:end),   right_vel,      right_ori;
                          time_6,       wp_pair_3(1:3),     right_vel,      right_ori;
                          time_7,       wp_pair_3(4:end),   left_vel,       left_ori;
                          time_8,       wp_pair_4(1:3),     left_vel,       left_ori;
                          time_9,       wp_pair_4(4:end),   right_vel,      right_ori;
                          time_10,      wp_pair_5(1:3),     right_vel,      right_ori;
                          time_11,      wp_pair_5(4:end),   left_vel,       left_ori;
                          time_12,      wp_pair_6(1:3),     left_vel,       left_ori;
                          time_13,      wp_pair_6(4:end),   right_vel,      right_ori;
                          time_14,      wp_pair_7(1:3),     right_vel,      right_ori;
                          time_15,      wp_pair_7(4:end),   left_vel,       left_ori;
                          time_16,      wp_pair_8(1:3),     left_vel,       left_ori;
                          time_17,      wp_pair_8(4:end),   right_vel,      right_ori;
                          time_18,      wp_pair_9(1:3),     right_vel,      right_ori;
                          time_19,      wp_pair_9(4:end),   left_vel,       left_ori;
                          time_20,      wp_pair_10(1:3),    left_vel,       left_ori;
                          time_21,      wp_pair_10(4:end),  right_vel,      right_ori;
                          time_22,      wp_pair_11(1:3),    right_vel,      right_ori;
                          time_23,      wp_pair_11(4:end),  left_vel,       left_ori;
                          time_24,      wp_pair_12(1:3),    left_vel,       left_ori;
                          time_25,      wp_pair_12(4:end),  right_vel,      right_ori;
                          time_26,      stop_offshore,      right_vel,      right_ori;
                          time_27,      start_offshore,     backward_vel,   backward_ori;
                          end_time,     origo,              backward_vel,   backward_ori]
                          
                          
