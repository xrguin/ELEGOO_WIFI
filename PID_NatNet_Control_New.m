function PID_NatNet_Control_New
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
robot_ip_1 = "192.168.1.140"; % The IP address of your robot.
robot_ip_2 = "192.168.1.84";

% -- NatNet Server Configuration
natnet_client_ip = '192.168.1.203';  % The IP of this computer (the client).
natnet_server_ip = '192.168.1.209'; % The IP of the computer running Motive (the server).

% -- Target and Robot ID Configuration
% IMPORTANT: All position values are in MILLIMETERS (mm).
%target_pos = [-120, 34, -540]; % Target position [x, y, z] in MILLIMETERS.
way_points_1 = [1450 , 23, 707;...
               1136, 25, 348;...
               697, 33, -122;...
               374, 34, -287;...
               -133, 30, -415;...
               -613, 33, -638;...
               -748.5, 36, -1136.8;...
               -713.8, 38, -1598.8;...
               -972.4, 42, -2017.3];
target_pos_1 = way_points_1(end, :);

way_points_2 = [850 , 23, 707;...
               536, 25, 348;...
               97, 33, -122;...
               -374, 34, -287;...
               -733, 30, -415;...
               -1213, 33, -638;...
               -1348.5, 36, -1136.8;...
               -1313.8, 38, -1598.8;...
               -1572.4, 42, -2017.3];
target_pos_2 = way_points_2(end, :);


% -- PID Controller Gains (THESE WILL REQUIRE EXPERIMENTAL TUNING)
Kp_h = 0.85;  % Heading Proportional Gain
Ki_h = 0.03; % Heading Integral Gain
Kd_h = 0.15;  % Heading Derivative Gain
Kp_d = 0.25;  % Distance Proportional Gain
Ki_d = 0.01; % Distance Integral Gain
Kd_d = 0.1;  % Distance Derivative Gain

% -- Control Loop & Physics Parameters
distance_tolerance = 80;   % Stop when within 50 mm (5 cm) of the target.
heading_tolerance = 5;   % Allowable heading error in degrees to switch to DRIVING.
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
%h_target_1 = plot3(ax, target_pos_1(1), target_pos_1(2), target_pos_1(3), 'r*', 'MarkerSize', 15, 'LineWidth', 2);
if size(way_points_1, 1) > 1
     plot3(ax, way_points_1(1:end-1, 1), way_points_1(1:end-1, 2), way_points_1(1:end-1, 3), 'bo',...
      'MarkerSize', 10, 'MarkerFaceColor', 'b');
end
if size(way_points_2, 1) > 1
     plot3(ax, way_points_2(1:end-1, 1), way_points_2(1:end-1, 2), way_points_2(1:end-1, 3), 'go',...
      'MarkerSize', 10, 'MarkerFaceColor', 'g');
end
h_robot_1 = plot3(ax, 0, 0, 0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
h_robot_orientation_1 = line(ax, [0 0], [0 0], [0 0], 'Color', 'r', 'LineWidth', 2);
h_trajectory_1 = plot3(ax, 0, 0, 0, 'b-');

h_robot_2 = plot3(ax, 0, 0, 0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
h_robot_orientation_2 = line(ax, [0 0], [0 0], [0 0], 'Color', 'r', 'LineWidth', 2);
h_trajectory_2 = plot3(ax, 0, 0, 0, 'b-');
legend(ax, {'Target', 'Robot', 'Orientation', 'Trajectory'}, 'Location', 'best');

% -- Initialize PID & Loop Variables
integral_h_1 = 0;
prev_error_h_1 = 0;
integral_d_1 = 0;
prev_error_d_1 = 0;
integral_h_2 = 0;
prev_error_h_2 = 0;
integral_d_2 = 0;
prev_error_d_2 = 0;
trajectory_points_1 = [];
trajectory_points_2 = [];
last_loop_time = tic;
current_waypoint_index_1 = 1;
current_waypoint_index_2 = 1;
robot_state_1 = 'TURNING'; % Initial state
robot_state_2 = 'TURNING';

both_reached = 0;
flag_1 = 0;
flag_2 = 0;

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

        rb_data_1 = data.RigidBodies(1);
        rb_data_2 = data.RigidBodies(2);
      %  for i = 1:length(data.RigidBodies)
      %      if (data.RigidBodies(i).ID == robot_id), rb_data = data.RigidBodies(i); break; end
      %  end
      %  if isempty(rb_data), disp(['RB ID ', num2str(robot_id), ' not found...']); pause(0.01); continue; end

        current_pos_3d_1 = [rb_data_1.x, rb_data_1.y, rb_data_1.z] * 1000;
        current_pos_3d_2 = [rb_data_2.x, rb_data_2.y, rb_data_2.z] * 1000;

        fprintf("x : %f,  y : %f  z:  %f\n",current_pos_3d_1(1),current_pos_3d_1(2),current_pos_3d_1(3));
        fprintf("x : %f,  y : %f  z:  %f\n",current_pos_3d_2(1),current_pos_3d_2(2),current_pos_3d_2(3));

        q_1 = quaternion(rb_data_1.qw, rb_data_1.qx, rb_data_1.qy, rb_data_1.qz);
        angles_1 = q_1.EulerAngles('YZX'); 
        current_angle_1 = rad2deg(angles_1(1)) + 230;
        if current_angle_1 >= 180
            current_angle_1 = -360 + current_angle_1;
        end

        q_2 = quaternion(rb_data_2.qw, rb_data_2.qx, rb_data_2.qy, rb_data_2.qz);
        angles_2 = q_2.EulerAngles('YZX'); 
        current_angle_2 = rad2deg(angles_2(1)) + 230;
        if current_angle_2 >= 180
            current_angle_2 = -360 + current_angle_2;
        end

        % -- ERROR CALCULATION (in XZ plane)
        current_target_1 = way_points_1(current_waypoint_index_1, :);
        error_vec_xz_1 = [current_target_1(1) - current_pos_3d_1(1), current_target_1(3) - current_pos_3d_1(3)];
        error_vec_xz_1_1 = [way_points_1(1) - current_pos_3d_1(1), way_points_1(3) - current_pos_3d_1(3)];

        current_target_2 = way_points_2(current_waypoint_index_2, :);
        error_vec_xz_2 = [current_target_2(1) - current_pos_3d_2(1), current_target_2(3) - current_pos_3d_2(3)];
        error_vec_xz_2_1 = [way_points_2(1) - current_pos_3d_1(1), way_points_2(3) - current_pos_3d_1(3)];


       % error_vec_xz = [target_pos(1) - current_pos_3d(1), target_pos(3) - current_pos_3d(3)];
        distance_error_1 = norm(error_vec_xz_1);
        distance_error_2 = norm(error_vec_xz_2);

        distance_error_1_1 = norm(error_vec_xz_1_1);
        distance_error_2_1 = norm(error_vec_xz_2_1);

       

        if distance_error_1 < distance_tolerance
            if current_waypoint_index_1 == size(way_points_1, 1)
              disp('Target reached!');
              send_robot_command(robot_ip_1, 'S');
             % if distance_error_2 < distance_tolerance
             %   break;
              %end
            else
              %disp(['Waypoint ', num2str(current_waypoint_index_1), ' reached. Moving to the next.']);
              current_waypoint_index_1 = current_waypoint_index_1 + 1;
               % integral_h = 0;
               % prev_error_h = 0;
               % integral_d = 0;
               % prev_error_d = 0;
           end
        end

        if distance_error_2 < distance_tolerance
            if current_waypoint_index_2 == size(way_points_2, 1)
              disp('Target reached!');
              send_robot_command(robot_ip_2, 'S');
             % if distance_error_1 < distance_tolerance
             %   break;
             % end
            else
         %     disp(['Waypoint ', num2str(current_waypoint_index_2), ' reached. Moving to the next.']);
              current_waypoint_index_2 = current_waypoint_index_2 + 1;
               % integral_h = 0;
               % prev_error_h = 0;
               % integral_d = 0;
               % prev_error_d = 0;
           end
        end

        if distance_error_1 < distance_tolerance && distance_error_2 < distance_tolerance...
                && current_waypoint_index_1 == size(way_points_1, 1) && current_waypoint_index_2 == size(way_points_2, 1)
            break;
        end

        target_angle_1 = rad2deg(atan2(error_vec_xz_1(1), error_vec_xz_1(2)));
        target_angle_2 = rad2deg(atan2(error_vec_xz_2(1), error_vec_xz_2(2)));
        fprintf("1:  target angle : %f,  current angle : %f\n",target_angle_1,current_angle_1);
        fprintf("2:  target angle : %f,  current angle : %f\n",target_angle_2,current_angle_2);

        %% robot1
        heading_error_1 = target_angle_1 - current_angle_1;
        heading_error_1 = atan2(sin(deg2rad(heading_error_1)), cos(deg2rad(heading_error_1)));
        heading_error_1 = rad2deg(heading_error_1);

        % -- STATE MACHINE: Decide whether to TURN or DRIVE
        switch robot_state_1
            case 'INITIAL'
                if abs(heading_error_1) < heading_tolerance 
                    integral_h_1 = 0;
                    integral_d_1 = 0;
                    prev_error_h_1 = 0;
                    prev_error_d_1 = 0;
                    robot_state_1 = 'DRIVING';
                end
            case 'TURNING'
                if  distance_error_1_1 < distance_tolerance && flag_1 == 0
                    flag_1 = 1;
                    send_robot_command(robot_ip_1, 'S');
                    current_waypoint_index_1 = 2;
                    robot_state_1 = 'INITIAL';
                elseif abs(heading_error_1) < heading_tolerance
                    robot_state_1 = 'DRIVING';
                    % Reset PID integrals when switching state
                    integral_h_1 = 0;
                    integral_d_1 = 0;
                    prev_error_h_1 = 0;
                    prev_error_d_1 = 0;
                    disp('State -> DRIVING');
                end
            case 'DRIVING'
                if  distance_error_1_1 < distance_tolerance && flag_1 == 0
                    flag_1 = 1;
                    send_robot_command(robot_ip_1, 'S');
                    current_waypoint_index_1 = 2;
                    robot_state_1 = 'INITIAL';
                elseif abs(heading_error_1) > heading_tolerance  % Hysteresis
                    robot_state_1 = 'TURNING';
                    disp('State -> TURNING');
                end
        end

        % -- PID CALCULATIONS & MOTOR COMMANDS
        forward_speed_1 = 0;
        turn_speed_1 = 0;

        if strcmp(robot_state_1, 'TURNING') || (strcmp(robot_state_1, 'DRIVING') && current_waypoint_index_1 < size(way_points_1, 1))
            % --- Yaw PID Controller ---
            integral_h_1 = integral_h_1 + heading_error_1 * dt;
            derivative_h_1 = (heading_error_1 - prev_error_h_1) / dt;
            turn_speed_1 = Kp_h * heading_error_1 + Ki_h * integral_h_1 + Kd_h * derivative_h_1;
            prev_error_h_1 = heading_error_1;
            
            % Ensure turn speed does not exceed max
            turn_speed_1 = min(max(turn_speed_1, -max_turn_speed), max_turn_speed);

            forward_speed_1 = 60;
            
            % Turn in place
            left_speed_1 = round(forward_speed_1 + turn_speed_1);
            right_speed_1 = round(forward_speed_1 - turn_speed_1);

        elseif strcmp(robot_state_1, 'DRIVING') && current_waypoint_index_1 == size(way_points_1, 1)
            % --- Distance PID Controller ---
            % integral_d = integral_d + distance_error * dt;
            % derivative_d = (distance_error - prev_error_d) / dt;
            % forward_speed = Kp_d * distance_error + Ki_d * integral_d + Kd_d * derivative_d;
            % prev_error_d = distance_error;
            % 
            % % Ensure forward speed does not exceed max
            % forward_speed = min(max(forward_speed, 0), max_forward_speed);

            forward_speed_1 = 60;

            % --- Minor Heading Correction ---
            % Use the heading PID to make small adjustments while driving
            integral_h_1 = integral_h_1 + heading_error_1 * dt;
            derivative_h_1 = (heading_error_1 - prev_error_h_1) / dt;
            turn_speed_1 = Kp_h * heading_error_1 + Ki_h * integral_h_1 + Kd_h * derivative_h_1;
            prev_error_h_1 = heading_error_1;
            turn_speed_1 = min(max(turn_speed_1, -max_turn_speed), max_turn_speed);

            left_speed_1  = round(forward_speed_1 + turn_speed_1);
            right_speed_1 = round(forward_speed_1 - turn_speed_1);

        elseif strcmp(robot_state_1, 'INITIAL')
            forward_speed_1 = 0;

            integral_h_1 = integral_h_1 + heading_error_1 * dt;
            derivative_h_1 = (heading_error_1 - prev_error_h_1) / dt;
            turn_speed_1 = Kp_h * heading_error_1 + Ki_h * integral_h_1 + Kd_h * derivative_h_1;
            prev_error_h_1 = heading_error_1;
            turn_speed_1 = min(max(turn_speed_1, -max_turn_speed), max_turn_speed);

            left_speed_1  = round(forward_speed_1 + turn_speed_1);
            right_speed_1 = round(forward_speed_1 - turn_speed_1);
    
        end

        % Final speed constraints
        left_speed_1  = round(min(max(left_speed_1, -255), 255));
        right_speed_1 = round(min(max(right_speed_1, -255), 255));

        % Handle negative speeds for robots that support reverse
        % Assuming positive values are forward, negative are backward.
        % This part might need adjustment based on your robot's specific command protocol.
        % For now, we just ensure speeds are within a valid PWM range (e.g., 0-255)
        % and let the signs indicate direction.
        
        % This command structure might need to change if your robot needs
        % separate direction pins. The current one assumes sign = direction.
        json_command_1 = sprintf('{"N":4,"D1":%d,"D2":%d,"H":"pid"}', left_speed_1, right_speed_1);
        send_robot_command(robot_ip_1, json_command_1);

        %% robot2

        heading_error_2 = target_angle_2 - current_angle_2;
        heading_error_2 = atan2(sin(deg2rad(heading_error_2)), cos(deg2rad(heading_error_2)));
        heading_error_2 = rad2deg(heading_error_2);

        % -- STATE MACHINE: Decide whether to TURN or DRIVE
        switch robot_state_2
            case 'INITIAL'
                if abs(heading_error_2) < heading_tolerance 
                    integral_h_2 = 0;
                    integral_d_2 = 0;
                    prev_error_h_2 = 0;
                    prev_error_d_2 = 0;
                    robot_state_2 = 'DRIVING';
                end
            case 'TURNING'
                if  distance_error_2_1 < distance_tolerance && flag_2 == 0
                    flag_2 = 1;
                    send_robot_command(robot_ip_2, 'S');
                    current_waypoint_index_2 = 2;
                    robot_state_2 = 'INITIAL';
                elseif abs(heading_error_2) < heading_tolerance
                    robot_state_2 = 'DRIVING';
                    % Reset PID integrals when switching state
                    integral_h_2 = 0;
                    integral_d_2 = 0;
                    prev_error_h_2 = 0;
                    prev_error_d_2 = 0;
                    disp('State -> DRIVING');
                end
            case 'DRIVING'
                if  distance_error_2_1 < distance_tolerance && flag_2 == 0
                    flag_2 = 1;
                    send_robot_command(robot_ip_2, 'S');
                    current_waypoint_index_2 = 2;
                    robot_state_2 = 'INITIAL';
                elseif abs(heading_error_2) > heading_tolerance  % Hysteresis
                    robot_state_2 = 'TURNING';
                    disp('State -> TURNING');
                end
        end

        % -- PID CALCULATIONS & MOTOR COMMANDS
        forward_speed_2 = 0;
        turn_speed_2 = 0;

        if strcmp(robot_state_2, 'TURNING') || (strcmp(robot_state_2, 'DRIVING') && current_waypoint_index_2 < size(way_points_2, 1))
            % --- Yaw PID Controller ---
            integral_h_2 = integral_h_2 + heading_error_2 * dt;
            derivative_h_2 = (heading_error_2 - prev_error_h_2) / dt;
            turn_speed_2 = Kp_h * heading_error_2 + Ki_h * integral_h_2 + Kd_h * derivative_h_2;
            prev_error_h_2 = heading_error_2;
            
            % Ensure turn speed does not exceed max
            turn_speed_2 = min(max(turn_speed_2, -max_turn_speed), max_turn_speed);

            forward_speed_2 = 60;
            
            % Turn in place
            left_speed_2 = round(forward_speed_2 + turn_speed_2);
            right_speed_2 = round(forward_speed_2 - turn_speed_2);

        elseif strcmp(robot_state_2, 'DRIVING') && current_waypoint_index_2 == size(way_points_2, 1)
            % --- Distance PID Controller ---
            % integral_d = integral_d + distance_error * dt;
            % derivative_d = (distance_error - prev_error_d) / dt;
            % forward_speed = Kp_d * distance_error + Ki_d * integral_d + Kd_d * derivative_d;
            % prev_error_d = distance_error;
            % 
            % % Ensure forward speed does not exceed max
            % forward_speed = min(max(forward_speed, 0), max_forward_speed);

            forward_speed_2 = 60;

            % --- Minor Heading Correction ---
            % Use the heading PID to make small adjustments while driving
            integral_h_2 = integral_h_2 + heading_error_2 * dt;
            derivative_h_2 = (heading_error_2 - prev_error_h_2) / dt;
            turn_speed_2 = Kp_h * heading_error_2 + Ki_h * integral_h_2 + Kd_h * derivative_h_2;
            prev_error_h_2 = heading_error_2;
            turn_speed_2 = min(max(turn_speed_2, -max_turn_speed), max_turn_speed);

            left_speed_2  = round(forward_speed_2 + turn_speed_2);
            right_speed_2 = round(forward_speed_2 - turn_speed_2);

        elseif strcmp(robot_state_2, 'INITIAL')
            forward_speed_2 = 0;

            integral_h_2 = integral_h_2 + heading_error_2 * dt;
            derivative_h_2 = (heading_error_2 - prev_error_h_2) / dt;
            turn_speed_2 = Kp_h * heading_error_2 + Ki_h * integral_h_2 + Kd_h * derivative_h_2;
            prev_error_h_2 = heading_error_2;
            turn_speed_2 = min(max(turn_speed_2, -max_turn_speed), max_turn_speed);

            left_speed_2  = round(forward_speed_2 + turn_speed_2);
            right_speed_2 = round(forward_speed_2 - turn_speed_2);
    
        end

        % Final speed constraints
        left_speed_2  = round(min(max(left_speed_2, -255), 255));
        right_speed_2 = round(min(max(right_speed_2, -255), 255));

        % Handle negative speeds for robots that support reverse
        % Assuming positive values are forward, negative are backward.
        % This part might need adjustment based on your robot's specific command protocol.
        % For now, we just ensure speeds are within a valid PWM range (e.g., 0-255)
        % and let the signs indicate direction.
        
        % This command structure might need to change if your robot needs
        % separate direction pins. The current one assumes sign = direction.
        json_command_2 = sprintf('{"N":4,"D1":%d,"D2":%d,"H":"pid"}', left_speed_2, right_speed_2);
        send_robot_command(robot_ip_2, json_command_2);
        
        %% -- VISUALIZATION UPDATE (in 3D)
        set(h_robot_1, 'XData', current_pos_3d_1(1), 'YData', current_pos_3d_1(2), 'ZData', current_pos_3d_1(3));
        orientation_length = 250;
        current_angle1_1 = deg2rad(current_angle_1);
        orientation_end_pos_1 = [current_pos_3d_1(1) + orientation_length * cos(deg2rad(current_angle1_1)), ...
                               current_pos_3d_1(2), ...
                               current_pos_3d_1(3) + orientation_length * sin(deg2rad(current_angle1_1)-pi)];
        set(h_robot_orientation_1, 'XData', [current_pos_3d_1(1), orientation_end_pos_1(1)], ...
                               'YData', [current_pos_3d_1(2), orientation_end_pos_1(2)], ...
                               'ZData', [current_pos_3d_1(3), orientation_end_pos_1(3)]);
        
        trajectory_points_1(:, end+1) = current_pos_3d_1;
        set(h_trajectory_1, 'XData', trajectory_points_1(1,:), 'YData', trajectory_points_1(2,:), 'ZData', trajectory_points_1(3,:));
        drawnow limitrate;

        set(h_robot_2, 'XData', current_pos_3d_2(1), 'YData', current_pos_3d_2(2), 'ZData', current_pos_3d_2(3));
        orientation_length = 250;
        current_angle1_2 = deg2rad(current_angle_2);
        orientation_end_pos_2 = [current_pos_3d_2(1) + orientation_length * cos(deg2rad(current_angle1_2)), ...
                               current_pos_3d_2(2), ...
                               current_pos_3d_2(3) + orientation_length * sin(deg2rad(current_angle1_2)-pi)];
        set(h_robot_orientation_2, 'XData', [current_pos_3d_2(1), orientation_end_pos_2(1)], ...
                               'YData', [current_pos_3d_2(2), orientation_end_pos_2(2)], ...
                               'ZData', [current_pos_3d_2(3), orientation_end_pos_2(3)]);
        
        trajectory_points_2(:, end+1) = current_pos_3d_2;
        set(h_trajectory_2, 'XData', trajectory_points_2(1,:), 'YData', trajectory_points_2(2,:), 'ZData', trajectory_points_2(3,:));
        drawnow limitrate;
    end
catch e
    disp('An error occurred. Stopping robot and disconnecting.');
    send_robot_command(robot_ip_1, 'S');
    send_robot_command(robot_ip_2, 'S');
    natnetclient.disconnect;
    rethrow(e);
end

% =======================================================================
% 4. CLEANUP
% =======================================================================
disp('Disconnecting NatNet client...');
send_robot_command(robot_ip_1, 'S');
send_robot_command(robot_ip_2, 'S');
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