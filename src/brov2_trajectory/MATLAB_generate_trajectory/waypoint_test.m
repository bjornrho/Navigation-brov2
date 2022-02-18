
rotate3d on;
%% Creating a lawnmower trajectory
%Choose trajectory to create and simulate
constraints = horizontal_constraints;

trajectory = waypointTrajectory(constraints(:,2:4), ...
    'TimeOfArrival',constraints(:,1), ...
    'Velocities',constraints(:,5:7), ...
    'Orientation',quaternion(constraints(:,8:10),'eulerd','ZYX','frame'), ...
    'SampleRate',10);


tInfo = waypointInfo(trajectory)

%% Plotting
figure(1)
plot3(tInfo.Waypoints(1,1),tInfo.Waypoints(1,2),tInfo.Waypoints(1,3),'b*')
title('Position')
axis([-10,180,-75,75,-5,25])
%axis([-3,3,-3,3,-3,3])
xlabel('North')
ylabel('East')
grid on
daspect([1 1 1])
set(gca, 'ZDir','reverse')
hold on

%% Accumulating trajectory
pos = zeros(int64(tInfo.TimeOfArrival(end)*trajectory.SampleRate),3);
orient = zeros(int64(tInfo.TimeOfArrival(end)*trajectory.SampleRate),1,'quaternion');
vel = pos;
acc = pos;
angVel = pos;
angAcc = pos;

count = 1;
while ~isDone(trajectory)
   [pos(count,:),orient(count),vel(count,:),acc(count,:),angVel(count,:)] = trajectory();
   
   plot3(pos(count,1),pos(count,2),pos(count,3),'bo')

   %pause(trajectory.SamplesPerFrame/trajectory.SampleRate)
   count = count + 1;
end

%% Getting angular acceleration
roughAngAcc = diff(angVel,1,1)/(tInfo.TimeOfArrival(end)/(tInfo.TimeOfArrival(end)*trajectory.SampleRate));
roughAngAcc = [zeros(1,3); roughAngAcc];



%% Write to csv-file
%header = [         "pos - north, ",    "pos - east, ",     "pos - z, ", ...
%                    "ori - real, ",     "ori - i, ",        "ori - j, ", "ori - k, ", ...
%                    "vel - north, ",    "vel - east",       "vel - z, ", ...
%                    "acc - north, ",    "acc - east",       "acc - z, ", ...
%                    "angVel - north, ", "angVel - east, ", "angVel - z, "];
           
% fid = fopen('horizontal_trajectory.csv', 'w+');
% fprintf(fid, header);
% fclose(fid);
[real,i,j,k] = parts(orient)

trajectory_matrix = [pos,real,i,j,k,vel,acc,angVel,roughAngAcc];
trajectory_matrix(end,:) = []; % pop last row which doesn't contain trajectory data
writematrix(trajectory_matrix,'horizontal_trajectory.csv','Delimiter','comma')
%writematrix(trajectory_matrix,'pool_trajectory.csv','Delimiter','comma')


%% IMU rotation
rot_x = @(theta) [1 0 0;0 cos(theta) -sin(theta);0 sin(theta) cos(theta)];
rot_y = @(theta) [cos(theta) 0 sin(theta);0 1 0;-sin(theta) 0 cos(theta)];
rot_z = @(theta) [cos(theta) -sin(theta) 0;sin(theta) cos(theta) 0;0 0 1];

rot_xy = rot_x(pi/2) * rot_y(pi);
rot_zx = rot_z(pi/2) * rot_x(pi/2);
quat_yx = rotm2quat(rot_xy)
quat_zx = rotm2quat(rot_zx)

rotm2quat([0,1,0;1,0,0;0,0,-1])
