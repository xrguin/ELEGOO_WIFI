function PID_NatNet_Control
% =======================================================================
% PID_NatNet_Control.m (3D XZ-Plane Control Version)
%
% This script integrates NatNet motion capture feedback with robot control
% to autonomously drive a robot to a specified target position using PID.
% This version operates in the X-Z plane, suitable for ground robots.
% =======================================================================

% This script is now a function. To run, type 'PID_NatNet_Control' in the command window.

clc; close all;

% =======================================================================
% 0. SETUP - Load NatNet .NET Assembly
% =======================================================================
try
    NET.addAssembly('D:\NatNet_SDK_4.3\NatNetSDK\lib\x64\NatNetML.dll');
catch e
    error(['Failed to load NatNetML.dll. Please ensure the path is correct ' ...
           'and the .NET Framework is installed. Original error: %s'], e.message);
end

% =======================================================================
% 1. CONFIGURATION - !!! USER MUST EDIT THIS SECTION !!!
% =======================================================================

% -- Robot Configuration
robot_ip = "192.168.1.140"; % The IP address of your robot.

% -- NatNet Server Configuration
natnet_client_ip = '192.168.1.203';  % The IP of this computer (the client).
natnet_server_ip = '192.168.1.209'; % The IP of the computer running Motive (the server).

% -- Target and Robot ID Configuration
% IMPORTANT: All position values are in MILLIMETERS (mm).
%target_pos = [-120, 34, -540]; % Target position [x, y, z] in MILLIMETERS.
way_points = [1450 , 23, 707;...
               1136, 25, 348;...
               697, 33, -122;...
               374, 34, -287;...
               -133, 30, -415;...
               -613, 33, -638;...
               -748.5, 36, -1136.8;...
               -713.8, 38, -1598.8;...
               -972.4, 42, -2017.3];
target_pos = way_points(end, :);
robot_id = 1;                    % The Rigid Body ID of your robot in Motive.

% -- PID Controller Gains (THESE WILL REQUIRE EXPERIMENTAL TUNING)
Kp_h = 0.85;  % Heading Proportional Gain
Ki_h = 0.03; % Heading Integral Gain
Kd_h = 0.15;  % Heading Derivative Gain
Kp_d = 0.25;  % Distance Proportional Gain
Ki_d = 0.01; % Distance Integral Gain
Kd_d = 0.1;  % Distance Derivative Gain

% -- Control Loop & Physics Parameters
distance_tolerance = 80;   % Stop when within 50 mm (5 cm) of the target.
heading_tolerance = 10;   % Allowable heading error in degrees to switch to DRIVING.
max_turn_speed = 100;      % Maximum speed during turning (0-255).
max_forward_speed = 60;   % Maximum speed when driving forward (0-255).

% =======================================================================
% 2. INITIALIZATION
% =======================================================================

% -- Initialize NatNet Connection
disp('Initializing NatNet Client...');
natnetclient = natnet;
natnetclient.HostIP = natnet_server_ip;
natnetclient.ClientIP = natnet_client_ip;
natnetclient.ConnectionType = 'Multicast';
natnetclient.connect;
if (natnetclient.IsConnected == 0), error('NatNet client failed to connect.'); end
model = natnetclient.getModelDescription;
if (model.RigidBodyCount < 1), error('No rigid bodies found in model.'); end
disp('NatNet connection successful.');

% -- Initialize 3D Plot
figure;
ax = axes;
hold(ax, 'on'); grid on; axis equal;
title('3D Robot Trajectory Control (XZ Plane)');
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
view(ax, 0, 0); % Set view to Top-Down (X-Z plane)

% Define and draw the 3D Boundary
boundary_pts = [ 2467,   50,  148; ...
                 1814,   65, -2659;
                -1878,   37, -2732;
                -1101,  6.8,  1769];
closed_boundary = [boundary_pts; boundary_pts(1,:)];
plot3(ax, closed_boundary(:,1), closed_boundary(:,2), closed_boundary(:,3), 'k-', 'LineWidth', 2);

% Set axis limits to encompass the boundary with padding
padding = 500; % mm
xlim([min(boundary_pts(:,1)) - padding, max(boundary_pts(:,1)) + padding]);
ylim([min(boundary_pts(:,2)) - padding, max(boundary_pts(:,2)) + padding]);
zlim([min(boundary_pts(:,3)) - padding, max(boundary_pts(:,3)) + padding]);
set(ax, 'ZDir', 'reverse');

% -- Plot Handles
h_target = plot3(ax, target_pos(1), target_pos(2), target_pos(3), 'r*', 'MarkerSize', 15, 'LineWidth', 2);
if size(way_points, 1) > 1
     plot3(ax, way_points(1:end-1, 1), way_points(1:end-1, 2), way_points(1:end-1, 3), 'bo',...
      'MarkerSize', 10, 'MarkerFaceColor', 'b');
end
h_robot = plot3(ax, 0, 0, 0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
h_robot_orientation = line(ax, [0 0], [0 0], [0 0], 'Color', 'r', 'LineWidth', 2);
h_trajectory = plot3(ax, 0, 0, 0, 'b-');
legend(ax, {'Target', 'Robot', 'Orientation', 'Trajectory'}, 'Location', 'best');

% -- Initialize PID & Loop Variables
integral_h = 0;
prev_error_h = 0;
integral_d = 0;
prev_error_d = 0;
trajectory_points = [];
last_loop_time = tic;
current_waypoint_index = 1;
robot_state = 'TURNING'; % Initial state

disp('Initialization complete. Starting control loop...');

% =======================================================================
% 3. MAIN CONTROL LOOP
% =======================================================================
try
    while true
        dt = toc(last_loop_time);
        last_loop_time = tic;

        data = natnetclient.getFrame;
        if isempty(data.RigidBodies), disp('No rigid bodies in frame...'); pause(0.01); continue; end

        rb_data = data.RigidBodies(1);
      %  for i = 1:length(data.RigidBodies)
      %      if (data.RigidBodies(i).ID == robot_id), rb_data = data.RigidBodies(i); break; end
      %  end
      %  if isempty(rb_data), disp(['RB ID ', num2str(robot_id), ' not found...']); pause(0.01); continue; end

        current_pos_3d = [rb_data.x, rb_data.y, rb_data.z] * 1000;

        fprintf("x : %f,  y : %f  z:  %f\n",current_pos_3d(1),current_pos_3d(2),current_pos_3d(3));

        q = quaternion(rb_data.qw, rb_data.qx, rb_data.qy, rb_data.qz);
        angles = q.EulerAngles('YZX'); 
        current_angle = rad2deg(angles(1)) + 230;
        if current_angle >= 180
            current_angle = -360 + current_angle;
        end

        % -- ERROR CALCULATION (in XZ plane)
        current_target = way_points(current_waypoint_index, :);
        error_vec_xz = [current_target(1) - current_pos_3d(1), current_target(3) - current_pos_3d(3)];


       % error_vec_xz = [target_pos(1) - current_pos_3d(1), target_pos(3) - current_pos_3d(3)];
        distance_error = norm(error_vec_xz);

        if distance_error < distance_tolerance
            if current_waypoint_index == size(way_points, 1)
              disp('Target reached!');
              send_robot_command(robot_ip, 'S');
              break;
            else
              disp(['Waypoint ', num2str(current_waypoint_index), ' reached. Moving to the next.']);
              current_waypoint_index = current_waypoint_index + 1;
               % integral_h = 0;
               % prev_error_h = 0;
               % integral_d = 0;
               % prev_error_d = 0;
           end
        end

        target_angle = rad2deg(atan2(error_vec_xz(1), error_vec_xz(2)));
        fprintf("target angle : %f,  current angle : %f\n",target_angle,current_angle);
        heading_error = target_angle - current_angle;
        heading_error = atan2(sin(deg2rad(heading_error)), cos(deg2rad(heading_error)));
        heading_error = rad2deg(heading_error);

        % -- STATE MACHINE: Decide whether to TURN or DRIVE
        switch robot_state
            case 'TURNING'
                if abs(heading_error) < heading_tolerance
                    robot_state = 'DRIVING';
                    % Reset PID integrals when switching state
                    integral_h = 0;
                    integral_d = 0;
                    prev_error_h = 0;
                    prev_error_d = 0;
                    disp('State -> DRIVING');
                end
            case 'DRIVING'
                if abs(heading_error) > heading_tolerance  % Hysteresis
                    robot_state = 'TURNING';
                    disp('State -> TURNING');
                end
        end

        % -- PID CALCULATIONS & MOTOR COMMANDS
        forward_speed = 0;
        turn_speed = 0;

        if strcmp(robot_state, 'TURNING') || (strcmp(robot_state, 'DRIVING') && current_waypoint_index < size(way_points, 1))
            % --- Yaw PID Controller ---
            integral_h = integral_h + heading_error * dt;
            derivative_h = (heading_error - prev_error_h) / dt;
            turn_speed = Kp_h * heading_error + Ki_h * integral_h + Kd_h * derivative_h;
            prev_error_h = heading_error;
            
            % Ensure turn speed does not exceed max
            turn_speed = min(max(turn_speed, -max_turn_speed), max_turn_speed);

            forward_speed = 60;
            
            % Turn in place
            left_speed = round(forward_speed + turn_speed);
            right_speed = round(forward_speed - turn_speed);

        elseif strcmp(robot_state, 'DRIVING') && current_waypoint_index == size(way_points, 1)
            % --- Distance PID Controller ---
            % integral_d = integral_d + distance_error * dt;
            % derivative_d = (distance_error - prev_error_d) / dt;
            % forward_speed = Kp_d * distance_error + Ki_d * integral_d + Kd_d * derivative_d;
            % prev_error_d = distance_error;
            % 
            % % Ensure forward speed does not exceed max
            % forward_speed = min(max(forward_speed, 0), max_forward_speed);

            forward_speed = 60;

            % --- Minor Heading Correction ---
            % Use the heading PID to make small adjustments while driving
            integral_h = integral_h + heading_error * dt;
            derivative_h = (heading_error - prev_error_h) / dt;
            turn_speed = Kp_h * heading_error + Ki_h * integral_h + Kd_h * derivative_h;
            prev_error_h = heading_error;
            turn_speed = min(max(turn_speed, -max_turn_speed), max_turn_speed);

            left_speed  = round(forward_speed + turn_speed);
            right_speed = round(forward_speed - turn_speed);
    
        end

        % Final speed constraints
        left_speed  = round(min(max(left_speed, -255), 255));
        right_speed = round(min(max(right_speed, -255), 255));

        % Handle negative speeds for robots that support reverse
        % Assuming positive values are forward, negative are backward.
        % This part might need adjustment based on your robot's specific command protocol.
        % For now, we just ensure speeds are within a valid PWM range (e.g., 0-255)
        % and let the signs indicate direction.
        
        % This command structure might need to change if your robot needs
        % separate direction pins. The current one assumes sign = direction.
        json_command = sprintf('{"N":4,"D1":%d,"D2":%d,"H":"pid"}', left_speed, right_speed);
        send_robot_command(robot_ip, json_command);
        
        % -- VISUALIZATION UPDATE (in 3D)
        set(h_robot, 'XData', current_pos_3d(1), 'YData', current_pos_3d(2), 'ZData', current_pos_3d(3));
        orientation_length = 250;
        current_angle1 = deg2rad(current_angle);
        orientation_end_pos = [current_pos_3d(1) + orientation_length * cos(deg2rad(current_angle1)), ...
                               current_pos_3d(2), ...
                               current_pos_3d(3) + orientation_length * sin(deg2rad(current_angle1)-pi)];
        set(h_robot_orientation, 'XData', [current_pos_3d(1), orientation_end_pos(1)], ...
                               'YData', [current_pos_3d(2), orientation_end_pos(2)], ...
                               'ZData', [current_pos_3d(3), orientation_end_pos(3)]);
        
        trajectory_points(:, end+1) = current_pos_3d;
        set(h_trajectory, 'XData', trajectory_points(1,:), 'YData', trajectory_points(2,:), 'ZData', trajectory_points(3,:));
        drawnow limitrate;
    end
catch e
    disp('An error occurred. Stopping robot and disconnecting.');
    send_robot_command(robot_ip, 'S');
    natnetclient.disconnect;
    rethrow(e);
end

% =======================================================================
% 4. CLEANUP
% =======================================================================
disp('Disconnecting NatNet client...');
send_robot_command(robot_ip, 'S');
natnetclient.disconnect;
clear natnetclient;

end

% =======================================================================
% 5. HELPER FUNCTION (LOCAL FUNCTION)
% =======================================================================
function response = send_robot_command(ip, command)
    try
        t = tcpclient(ip, 80, 'ConnectTimeout', 1);
        data_to_send = [char(command), newline];
        write(t, data_to_send);
        response = 'OK';
        clear t;
    catch e
        response = ['Error: ', e.message];
    end
end