% =======================================================================
% PID_Velocity_Profile_Control.m
%
% Velocity profile-based control with dual PID controllers:
% - PID 1: Controls linear velocity to track desired velocity profile
% - PID 2: Controls angular velocity to track desired heading profile
% =======================================================================

clc; close all;

% =======================================================================
% 0. SETUP - Load NatNet .NET Assembly
% =======================================================================
try
    % NET.addAssembly('D:\NatNet_SDK_4.3\NatNetSDK\lib\x64\NatNetML.dll');
    NET.addAssembly('D:\MATLAB_Local\NatNetSDK\lib\x64\NatNetML.dll');
catch e
    error(['Failed to load NatNetML.dll. Please ensure the path is correct ' ...
           'and the .NET Framework is installed. Original error: %s'], e.message);
end

% =======================================================================
% 1. CONFIGURATION
% =======================================================================

% -- Robot Configuration
robot_ip = "192.168.1.140";

% -- NatNet Server Configuration
natnet_client_ip = '192.168.1.70';
natnet_server_ip = '192.168.1.209';

% -- Waypoints (in millimeters)
way_points = [1450, 23, 707;
    1371.5, 23.5, 617.25;
    1293, 24, 527.5;
    1214.5, 24.5, 437.75;
    1136, 25, 348;
    1026.25, 27, 230.5;
    916.5, 29, 113;
    806.75, 31, -4.5;
    697, 33, -122;
    604.75, 33.25, -163.25;
    512.5, 33.5, -204.5;
    420.25, 33.75, -245.75;
    374, 34, -287;
    247.25, 33, -319;
    120.5, 32, -351;
    -6.25, 31, -383;
    -133, 30, -415;
    -243.25, 30.75, -470.75;
    -353.5, 31.5, -526.5;
    -463.75, 32.25, -582.25;
    -613, 33, -638;
    -649.625, 33.75, -762.7;
    -686.25, 34.5, -887.4;
    -722.875, 35.25, -1012.1;
    -748.5, 36, -1136.8;
    -722.475, 36.5, -1253.3;
    -696.45, 37, -1370.8;
    -670.425, 37.5, -1484.3;
    -713.8, 38, -1598.8;
    -778.45, 39, -1703.425;
    -843.1, 40, -1808.05;
    -907.75, 41, -1912.675;
    -972.4, 42, -2017.3];

way_points = way_points - [100, 0, 100];
target_pos = way_points(end, :);
robot_id = 1;

% -- Velocity Profile Parameters
desired_motor_speed = 80;  % Constant motor speed (PWM units)
max_motor_speed = 255;     % Maximum motor PWM value

% Velocity conversion from calibration: velocity (mm/s) = 3.65 * motor_speed - 32
velocity_conversion_factor = 3.65;  % mm/s per PWM unit
velocity_offset = -32;              % mm/s offset
desired_velocity_mms = desired_motor_speed * velocity_conversion_factor + velocity_offset;  % ~260 mm/s

% -- PID Controller Gains for Linear Velocity
Kp_v = 2.0;   % Velocity Proportional Gain
Ki_v = 0.5;   % Velocity Integral Gain
Kd_v = 0.1;   % Velocity Derivative Gain

% -- PID Controller Gains for Angular Velocity (Heading)
Kp_w = 1.5;   % Angular Proportional Gain
Ki_w = 0.05;  % Angular Integral Gain
Kd_w = 0.2;   % Angular Derivative Gain

% -- Control Parameters
distance_tolerance = 80;           % Waypoint reached tolerance (mm)
alignment_heading_tolerance = 5;   % Alignment phase tolerance (degrees)
velocity_smoothing_alpha = 0.3;    % Exponential smoothing for velocity calculation

% =======================================================================
% 2. GENERATE TIME-BASED ANGULAR PROFILE
% =======================================================================

% Calculate total path length and estimated duration
total_path_length = 0;
for i = 1:size(way_points, 1)-1
    segment_length = norm([way_points(i+1, 1) - way_points(i, 1), way_points(i+1, 3) - way_points(i, 3)]);
    total_path_length = total_path_length + segment_length;
end

% Estimate total time based on desired velocity (~260 mm/s for 80 motor speed)
estimated_total_time = total_path_length / desired_velocity_mms;
fprintf('Total path length: %.1f mm, Estimated duration: %.1f seconds\n', ...
        total_path_length, estimated_total_time);

% Generate waypoint-based angular profile with uniform time intervals
num_waypoints = size(way_points, 1);
time_per_waypoint = estimated_total_time / num_waypoints;  % Uniform time intervals
time_points = 0:time_per_waypoint:estimated_total_time;

% Initialize angular profile structure
angular_profile = struct();
angular_profile.time_points = time_points;
angular_profile.desired_headings = zeros(1, length(time_points));
angular_profile.waypoint_headings = zeros(1, num_waypoints);  % Store individual waypoint headings

% Calculate desired heading for each waypoint (pointing to next waypoint)
for i = 1:num_waypoints-1
    delta_x = way_points(i+1, 1) - way_points(i, 1);
    delta_z = way_points(i+1, 3) - way_points(i, 3);
    angular_profile.waypoint_headings(i) = rad2deg(atan2(delta_x, delta_z));
end
% Last waypoint maintains the previous heading
angular_profile.waypoint_headings(end) = angular_profile.waypoint_headings(end-1);

% Assign headings to time points (one heading per time interval)
for i = 1:min(length(time_points), num_waypoints)
    angular_profile.desired_headings(i) = angular_profile.waypoint_headings(i);
end
% Fill any remaining time points with final heading
if length(time_points) > num_waypoints
    angular_profile.desired_headings(num_waypoints+1:end) = angular_profile.waypoint_headings(end);
end

fprintf('Waypoint-based angular profile generated:\n');
fprintf('- %d waypoints with %d different headings\n', num_waypoints, num_waypoints);
fprintf('- Time per waypoint: %.2f seconds\n', time_per_waypoint);
fprintf('- Total profile duration: %.1f seconds\n', estimated_total_time);
for i = 1:min(5, num_waypoints)  % Show first 5 waypoints
    fprintf('  WP %d: Heading %.1f° at time %.1f s\n', i, angular_profile.waypoint_headings(i), (i-1)*time_per_waypoint);
end
if num_waypoints > 5
    fprintf('  ... and %d more waypoints\n', num_waypoints-5);
end

% =======================================================================
% 3. INITIALIZATION
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

% -- Initialize Dual Plot
figure('Position', [100, 100, 1400, 600]);

% Subplot 1: XZ Plane trajectory
subplot(1, 2, 1);
ax1 = gca;
hold(ax1, 'on'); grid on; axis equal;
title('Robot Trajectory (XZ Plane)');
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
view(ax1, 0, 0);

% Subplot 2: Heading vs Time
subplot(1, 2, 2);
ax2 = gca;
hold(ax2, 'on'); grid on;
title('Heading Control Performance');
xlabel('Time (s)'); ylabel('Heading (degrees)');
ylim([-180, 180]);
xlim([0, estimated_total_time]);

% Define and draw the 3D Boundary
boundary_pts = [ 2467,   50,  148; ...
                 1814,   65, -2659;
                -1878,   37, -2732;
                -1101,  6.8,  1769];
closed_boundary = [boundary_pts; boundary_pts(1,:)];
plot3(ax1, closed_boundary(:,1), closed_boundary(:,2), closed_boundary(:,3), 'k-', 'LineWidth', 2, 'HandleVisibility', 'off');

% Set axis limits for XZ plot
padding = 500;
xlim(ax1, [min(boundary_pts(:,1)) - padding, max(boundary_pts(:,1)) + padding]);
ylim(ax1, [min(boundary_pts(:,2)) - padding, max(boundary_pts(:,2)) + padding]);
zlim(ax1, [min(boundary_pts(:,3)) - padding, max(boundary_pts(:,3)) + padding]);
set(ax1, 'ZDir', 'reverse');

% -- Plot Handles for XZ trajectory
if size(way_points, 1) > 1
    h_waypoints = plot3(ax1, way_points(1:end-1, 1), way_points(1:end-1, 2), way_points(1:end-1, 3), 'bo',...
         'MarkerSize', 8, 'MarkerFaceColor', 'b', 'DisplayName', 'Waypoints');
end
h_start = plot3(ax1, way_points(1,1), way_points(1,2), way_points(1,3), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g', 'DisplayName', 'Start');
h_target = plot3(ax1, target_pos(1), target_pos(2), target_pos(3), 'r*', 'MarkerSize', 15, 'LineWidth', 2, 'DisplayName', 'Goal');
h_robot = plot3(ax1, 0, 0, 0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k', 'DisplayName', 'Robot');
h_robot_orientation = line(ax1, [0 0], [0 0], [0 0], 'Color', 'r', 'LineWidth', 2, 'DisplayName', 'Orientation');
h_trajectory = plot3(ax1, 0, 0, 0, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Trajectory');
legend(ax1, 'Location', 'best');

% -- Plot Handles for Heading plot
% Plot desired heading profile as step function
plot(ax2, angular_profile.time_points, angular_profile.desired_headings, 'r--', 'LineWidth', 2, 'DisplayName', 'Desired Heading');
h_current_heading = plot(ax2, 0, 0, 'b-', 'LineWidth', 2, 'DisplayName', 'Current Heading');
h_heading_point = plot(ax2, 0, 0, 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b', 'DisplayName', 'Current');
legend(ax2, 'Location', 'best');

% Initialize data storage for heading plot
heading_time_data = [];
heading_angle_data = [];

% -- Initialize PID Variables
% Linear velocity PID
integral_v = 0;
prev_error_v = 0;

% Angular velocity PID
integral_w = 0;
prev_error_w = 0;

% -- Initialize Control Variables
trajectory_points = [];
last_loop_time = tic;
current_waypoint_index = 1;
control_state = 'INITIAL_APPROACH';
alignment_pause_timer = 0;
first_waypoint_reached = false;

% Velocity estimation variables
prev_position = [0, 0, 0];
last_valid_position = [0, 0, 0];  % Store last non-zero position
current_velocity_mms = 0;  % mm/s
velocity_history = [];
position_init = false;

% Profile following variables
profile_start_time = [];  % Will be initialized when profile actually starts
current_desired_heading = 0;

disp('Initialization complete. Starting control loop...');

% =======================================================================
% 4. MAIN CONTROL LOOP
% =======================================================================
try
    while true
        dt = toc(last_loop_time);
        last_loop_time = tic;

        data = natnetclient.getFrame;
        if isempty(data.RigidBodies), disp('No rigid bodies in frame...'); pause(0.01); continue; end

        rb_data = data.RigidBodies(1);
        raw_position = [rb_data.x, rb_data.y, rb_data.z] * 1000;
        
        % Check for connection lost (0,0,0 position)
        if norm(raw_position) < 1  % Position is essentially 0,0,0
            if norm(last_valid_position) > 1  % We have a valid previous position
                current_pos_3d = last_valid_position;
                fprintf('Connection lost detected (0,0,0). Using last valid position: [%.1f, %.1f, %.1f]\n', ...
                       current_pos_3d(1), current_pos_3d(2), current_pos_3d(3));
            else
                % No valid position available, skip this iteration
                disp('Connection lost and no valid previous position. Skipping iteration...');
                pause(0.01);
                continue;
            end
        else
            % Valid position, update both current and last valid
            current_pos_3d = raw_position;
            last_valid_position = current_pos_3d;
        end

        q = quaternion(rb_data.qw, rb_data.qx, rb_data.qy, rb_data.qz);
        angles = q.EulerAngles('YZX');
        current_angle = rad2deg(angles(1)) + 230;
        if current_angle >= 180
            current_angle = -360 + current_angle;
        end

        % -- VELOCITY ESTIMATION
        if position_init
            % Calculate instantaneous velocity from position change
            position_delta = norm([current_pos_3d(1) - prev_position(1), ...
                                  current_pos_3d(3) - prev_position(3)]);
            if dt > 0
                instant_velocity = position_delta / dt;  % mm/s
                % Apply exponential smoothing
                current_velocity_mms = velocity_smoothing_alpha * instant_velocity + ...
                                      (1 - velocity_smoothing_alpha) * current_velocity_mms;
            end
        else
            position_init = true;
        end
        prev_position = current_pos_3d;

        % -- CHECK IF ALREADY NEAR FIRST WAYPOINT (skip initial approach)
        if strcmp(control_state, 'INITIAL_APPROACH') && position_init
            first_waypoint = way_points(1, :);
            distance_to_first = norm([first_waypoint(1) - current_pos_3d(1), first_waypoint(3) - current_pos_3d(3)]);
            
            if distance_to_first < distance_tolerance * 2  % Use 2x tolerance for initial check
                disp(['Already near first waypoint (distance: ' num2str(distance_to_first) ' mm). Skipping to alignment prep...']);
                first_waypoint_reached = true;
                control_state = 'ALIGNMENT_PREP';
                alignment_pause_timer = tic;
                % Reset PID integrals
                integral_v = 0; prev_error_v = 0;
                integral_w = 0; prev_error_w = 0;
            end
        end

        % -- WAYPOINT TRACKING
        current_target = way_points(current_waypoint_index, :);
        error_vec_xz = [current_target(1) - current_pos_3d(1), current_target(3) - current_pos_3d(3)];
        distance_error = norm(error_vec_xz);

        % Check if waypoint reached
        if distance_error < distance_tolerance
            if current_waypoint_index == size(way_points, 1)
                disp('Final target reached!');
                send_robot_command(robot_ip, 'S');
                break;
            elseif current_waypoint_index == 1 && ~first_waypoint_reached
                disp('First waypoint reached. Preparing for alignment to final goal...');
                first_waypoint_reached = true;
                control_state = 'ALIGNMENT_PREP';
                alignment_pause_timer = tic;
                % Reset PID integrals
                integral_v = 0; prev_error_v = 0;
                integral_w = 0; prev_error_w = 0;
            else
                % Move to next waypoint - instant heading switch
                disp(['Waypoint ', num2str(current_waypoint_index), ' reached. Moving to next.']);
                current_waypoint_index = current_waypoint_index + 1;
                % Reset angular PID for new heading
                integral_w = 0;
                prev_error_w = 0;
            end
        end

        % -- CONTROL STATE MACHINE
        forward_speed = 0;
        turn_speed = 0;
        left_speed = 0;
        right_speed = 0;

        switch control_state
            case 'INITIAL_APPROACH'
                % Approach first waypoint with basic control
                fprintf('INITIAL_APPROACH | Distance: %.1f mm\n', distance_error);
                
                % Simple proportional control to reach first waypoint
                forward_speed = min(0.3 * distance_error, 80);
                
                % Calculate heading to first waypoint
                target_angle = rad2deg(atan2(error_vec_xz(1), error_vec_xz(2)));
                heading_error = target_angle - current_angle;
                heading_error = atan2(sin(deg2rad(heading_error)), cos(deg2rad(heading_error)));
                heading_error = rad2deg(heading_error);
                
                turn_speed = 1.0 * heading_error;
                turn_speed = min(max(turn_speed, -100), 100);
                
                left_speed = round(forward_speed + turn_speed);
                right_speed = round(forward_speed - turn_speed);

            case 'ALIGNMENT_PREP'
                % Stop and wait for 1 second
                fprintf('ALIGNMENT_PREP | Pausing...\n');
                left_speed = 0;
                right_speed = 0;
                
                if toc(alignment_pause_timer) >= 1
                    disp('Starting alignment to final goal...');
                    control_state = 'ALIGNING';
                    current_waypoint_index = 2;
                end

            case 'ALIGNING'
                % Align to face final goal position
                fprintf('ALIGNING | ');
                
                % Calculate heading to final goal
                goal_vec = [target_pos(1) - current_pos_3d(1), target_pos(3) - current_pos_3d(3)];
                target_angle = rad2deg(atan2(goal_vec(1), goal_vec(2)));
                heading_error = target_angle - current_angle;
                heading_error = atan2(sin(deg2rad(heading_error)), cos(deg2rad(heading_error)));
                heading_error = rad2deg(heading_error);
                
                fprintf('Heading error to goal: %.1f deg\n', abs(heading_error));
                
                if abs(heading_error) < alignment_heading_tolerance
                    disp('Alignment complete. Starting velocity profile control...');
                    control_state = 'VELOCITY_PROFILE_CONTROL';
                    % Reset PID integrals
                    integral_v = 0; prev_error_v = 0;
                    integral_w = 0; prev_error_w = 0;
                else
                    % Pure rotation
                    turn_speed = 1.2 * heading_error;
                    turn_speed = min(max(turn_speed, -100), 100);
                    left_speed = round(turn_speed);
                    right_speed = round(turn_speed);
                end

            case 'VELOCITY_PROFILE_CONTROL'
                % Time-based profile following
                if isempty(profile_start_time)
                    % First time entering profile control - start the timer
                    profile_start_time = tic;
                    current_time = 0;
                    disp('Profile timer started!');
                else
                    current_time = toc(profile_start_time);
                end
                
                % Get desired heading from time-based angular profile
                if current_time <= angular_profile.time_points(end)
                    % Interpolate desired heading based on current time
                    current_desired_heading = interp1(angular_profile.time_points, ...
                                                     angular_profile.desired_headings, ...
                                                     current_time, 'previous', 'extrap');
                else
                    % Use final heading if beyond profile duration
                    current_desired_heading = angular_profile.desired_headings(end);
                end
                
                fprintf('PROFILE_CONTROL | Time: %.1f/%.1f s | ', current_time, estimated_total_time);
                
                % -- Linear Velocity: Fixed 80 motor speed (no PID)
                forward_speed = desired_motor_speed;
                
                % -- Angular Velocity Control (Heading PID)
                heading_error = current_desired_heading - current_angle;
                heading_error = atan2(sin(deg2rad(heading_error)), cos(deg2rad(heading_error)));
                heading_error = rad2deg(heading_error);
                
                integral_w = integral_w + heading_error * dt;
                derivative_w = (heading_error - prev_error_w) / dt;
                
                % PID output for angular adjustment
                angular_adjustment = Kp_w * heading_error + Ki_w * integral_w + Kd_w * derivative_w;
                prev_error_w = heading_error;
                
                % Constrain angular adjustment
                angular_adjustment = min(max(angular_adjustment, -max_motor_speed), max_motor_speed);
                
                % Combine fixed linear velocity with angular control
                left_speed = round(forward_speed + angular_adjustment);
                right_speed = round(forward_speed - angular_adjustment);
                
                % Calculate distance to goal
                distance_to_goal = norm([target_pos(1) - current_pos_3d(1), target_pos(3) - current_pos_3d(3)]);
                
                fprintf('Vel: %.1f mm/s (Fixed: %d) | Head: %.1f/%.1f | Goal dist: %.1f mm\n', ...
                        current_velocity_mms, desired_motor_speed, ...
                        current_angle, current_desired_heading, distance_to_goal);
                
                % Stop if profile time exceeded (safety)
                if current_time > estimated_total_time + 5
                    disp('Profile time exceeded. Stopping.');
                    send_robot_command(robot_ip, 'S');
                    break;
                end
        end

        % Final speed constraints
        left_speed = round(min(max(left_speed, -255), 255));
        right_speed = round(min(max(right_speed, -255), 255));

        % Send command to robot
        if strcmp(control_state, 'ALIGNING')
            json_command = sprintf('{"N":3,"D1":1,"D2":%d,"H":"pid"}', 75);
            send_robot_command(robot_ip, json_command);
        else
            json_command = sprintf('{"N":4,"D1":%d,"D2":%d,"H":"pid"}', left_speed, right_speed);
            send_robot_command(robot_ip, json_command);
        end

        % -- VISUALIZATION UPDATE
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
        
        % Update heading plot (only during profile control)
        if strcmp(control_state, 'VELOCITY_PROFILE_CONTROL') && ~isempty(profile_start_time)
            current_profile_time = toc(profile_start_time);
            heading_time_data(end+1) = current_profile_time;
            heading_angle_data(end+1) = current_angle;
            
            % Update current heading line
            set(h_current_heading, 'XData', heading_time_data, 'YData', heading_angle_data);
            set(h_heading_point, 'XData', current_profile_time, 'YData', current_angle);
            
            % Update heading plot limits if needed
            if current_profile_time > estimated_total_time
                xlim(ax2, [0, current_profile_time + 1]);
            end
        end
        
        drawnow limitrate;
    end
catch e
    disp('An error occurred. Stopping robot and disconnecting.');
    send_robot_command(robot_ip, 'S');
    natnetclient.disconnect;
    rethrow(e);
end

% =======================================================================
% 5. CLEANUP
% =======================================================================
disp('Disconnecting NatNet client...');
send_robot_command(robot_ip, 'S');
natnetclient.disconnect;
clear natnetclient;

% =======================================================================
% 6. HELPER FUNCTION
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