% =======================================================================
% APF_Static_Obstacle_Avoidance.m
%
% APF-based obstacle avoidance with static obstacle using OptiTrack
% - Constant linear velocity (80 motor speed)
% - APF controls heading direction only
% - Dual visualization: trajectory + obstacle, heading comparison
% =======================================================================

clc; close all;

% =======================================================================
% 0. SETUP - Load NatNet .NET Assembly
% =======================================================================
try
    NET.addAssembly('D:\MATLAB_Local\NatNetSDK\lib\x64\NatNetML.dll');
catch e
    error(['Failed to load NatNetML.dll. Please ensure the path is correct ' ...
           'and the .NET Framework is installed. Original error: %s'], e.message);
end

% =======================================================================
% 1. ROBOT AND OBSTACLE CONFIGURATION
% =======================================================================

% -- Robot Configuration (IP to Rigid Body ID mapping)
robot_configs = struct();
robot_configs(1).ip = "192.168.1.140";
robot_configs(1).rigid_body_id = 1;
robot_configs(1).name = "Robot_1";

robot_configs(2).ip = "192.168.1.84";
robot_configs(2).rigid_body_id = 2;
robot_configs(2).name = "Robot_84";

% Select which robot to control (change this to control different robots)
active_robot_id = 1;
robot_ip = robot_configs(active_robot_id).ip;
robot_rigid_body_id = robot_configs(active_robot_id).rigid_body_id;
obstacle_rigid_body_id = 4;  % Static obstacle rigid body ID

fprintf('Controlling %s (IP: %s, Rigid Body ID: %d)\n', ...
        robot_configs(active_robot_id).name, robot_ip, robot_rigid_body_id);
fprintf('Obstacle rigid body ID: %d\n', obstacle_rigid_body_id);

% -- NatNet Server Configuration
natnet_client_ip = '192.168.1.70';
natnet_server_ip = '192.168.1.209';

% -- Start and Goal Positions (first and last waypoints from original code)
start_position = [1450, 23, 707] - [100, 0, 100];     % [1350, 23, 607]
goal_position = [-972.4, 42, -2017.3] - [100, 0, 100]; % [-1072.4, 42, -2117.3]

fprintf('Start position: [%.1f, %.1f, %.1f] mm\n', start_position(1), start_position(2), start_position(3));
fprintf('Goal position: [%.1f, %.1f, %.1f] mm\n', goal_position(1), goal_position(2), goal_position(3));

% -- Velocity Profile Parameters
desired_motor_speed = 80;  % Constant motor speed (PWM units)
max_motor_speed = 255;     % Maximum motor PWM value

% Velocity conversion from calibration: velocity (mm/s) = 3.65 * motor_speed - 32
velocity_conversion_factor = 3.65;
velocity_offset = -32;
desired_velocity_mms = desired_motor_speed * velocity_conversion_factor + velocity_offset;  % ~260 mm/s

% -- APF Parameters
attraction_factor = 1.2;           % Goal attraction strength
repulsion_factor = 5;            % Obstacle repulsion strength
detection_radius_mm = 800;         % Obstacle detection radius (mm)
safe_radius_mm = 300;              % Preferred distance from obstacle (mm)

% -- PID Controller Gains for Angular Control
Kp_w = 1.5;   % Angular Proportional Gain
Ki_w = 0.2;  % Angular Integral Gain
Kd_w = 0.2;   % Angular Derivative Gain

% -- Control Parameters
distance_tolerance = 80;           % Goal reached tolerance (mm)
alignment_heading_tolerance = 20;   % Alignment phase tolerance (degrees)
velocity_smoothing_alpha = 0.3;    % Exponential smoothing for velocity calculation

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

% -- Initialize Dual Plot
figure('Position', [100, 100, 1400, 600]);

% Subplot 1: XZ Plane trajectory with obstacle
subplot(1, 2, 1);
ax1 = gca;
hold(ax1, 'on'); grid on; axis equal;
title('APF Obstacle Avoidance (XZ Plane)');
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
view(ax1, 0, 0);

% Subplot 2: Heading comparison (APF vs Actual)
subplot(1, 2, 2);
ax2 = gca;
hold(ax2, 'on'); grid on;
title('APF Heading Control');
xlabel('Time (s)'); ylabel('Heading (degrees)');
ylim([-180, 180]);

% Define and draw the 3D Boundary on XZ plot
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
h_start = plot3(ax1, start_position(1), start_position(2), start_position(3), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g', 'DisplayName', 'Start');
h_goal = plot3(ax1, goal_position(1), goal_position(2), goal_position(3), 'r*', 'MarkerSize', 15, 'LineWidth', 2, 'DisplayName', 'Goal');
h_robot = plot3(ax1, 0, 0, 0, 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b', 'DisplayName', 'Robot');
h_robot_orientation = line(ax1, [0 0], [0 0], [0 0], 'Color', 'b', 'LineWidth', 2, 'DisplayName', 'Robot Heading');
h_obstacle = plot3(ax1, 0, 0, 0, 'ko', 'MarkerSize', 80, 'MarkerFaceColor', 'k', 'DisplayName', 'Obstacle');
h_trajectory = plot3(ax1, 0, 0, 0, 'b-', 'LineWidth', 2, 'DisplayName', 'Trajectory');

% Draw direct path line (start to goal)
direct_path = plot3(ax1, [start_position(1), goal_position(1)], [start_position(2), goal_position(2)], ...
                   [start_position(3), goal_position(3)], 'r--', 'LineWidth', 1, 'DisplayName', 'Direct Path');

% Draw obstacle detection radius (will be updated in main loop)
% theta_circle = linspace(0, 2*pi, 50);
% h_detection_circle = plot3(ax1, 0, 0, 0, 'k--', 'LineWidth', 1, 'DisplayName', 'Detection Radius');
% h_safe_circle = plot3(ax1, 0, 0, 0, 'r:', 'LineWidth', 1, 'DisplayName', 'Safe Radius');

legend(ax1, 'Location', 'best');

% -- Plot Handles for Heading plot
h_apf_heading = plot(ax2, 0, 0, 'r-', 'LineWidth', 2, 'DisplayName', 'APF Desired Heading');
h_current_heading = plot(ax2, 0, 0, 'b-', 'LineWidth', 2, 'DisplayName', 'Actual Heading');
h_heading_point = plot(ax2, 0, 0, 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b', 'DisplayName', 'Current');
legend(ax2, 'Location', 'best');

% -- Initialize Variables
% PID variables
integral_w = 0;
prev_error_w = 0;

% Control variables
trajectory_points = [];
last_loop_time = tic;
control_state = 'INITIAL_APPROACH';
alignment_pause_timer = 0;
first_start_reached = false;

% Velocity estimation variables
prev_position = [0, 0, 0];
last_valid_position = [0, 0, 0];
current_velocity_mms = 0;
velocity_history = [];
position_init = false;

% APF tracking variables
apf_start_time = [];
heading_time_data = [];
heading_angle_data = [];
apf_heading_data = [];
obstacle_position = [0, 0, 0];
obstacle_detected = false;

disp('Initialization complete. Starting APF obstacle avoidance...');

% =======================================================================
% 3. MAIN CONTROL LOOP
% =======================================================================
try
    while true
        dt = toc(last_loop_time);
        last_loop_time = tic;

        data = natnetclient.getFrame;
        if isempty(data.RigidBodies), disp('No rigid bodies in frame...'); pause(0.01); continue; end

        % Get robot data
        if length(data.RigidBodies) < robot_rigid_body_id
            disp(['Robot rigid body ID ', num2str(robot_rigid_body_id), ' not found...']); 
            pause(0.01); continue; 
        end
        rb_data = data.RigidBodies(robot_rigid_body_id);
        raw_position = [rb_data.x, rb_data.y, rb_data.z] * 1000;
        
        % Check for connection lost (0,0,0 position)
        if norm(raw_position) < 1
            if norm(last_valid_position) > 1
                current_pos_3d = last_valid_position;
                fprintf('Robot connection lost. Using last valid position: [%.1f, %.1f, %.1f]\n', ...
                       current_pos_3d(1), current_pos_3d(2), current_pos_3d(3));
            else
                disp('Robot connection lost and no valid previous position. Skipping iteration...');
                pause(0.01); continue;
            end
        else
            current_pos_3d = raw_position;
            last_valid_position = current_pos_3d;
        end

        % Get robot orientation
        q = quaternion(rb_data.qw, rb_data.qx, rb_data.qy, rb_data.qz);
        angles = q.EulerAngles('YZX');
        current_angle = rad2deg(angles(1)) + 230;
        if current_angle >= 180
            current_angle = -360 + current_angle;
        end

        % Get obstacle data
        obstacle_detected = false;
        if data.nRigidBodies >= obstacle_rigid_body_id
            obstacle_rb = data.RigidBodies(obstacle_rigid_body_id);
            obstacle_raw = [obstacle_rb.x, obstacle_rb.y, obstacle_rb.z] * 1000;
            
            if norm(obstacle_raw) > 1  % Valid obstacle position
                obstacle_position = obstacle_raw;
                obstacle_detected = true;
            end
        end

        % -- VELOCITY ESTIMATION
        if position_init
            position_delta = norm([current_pos_3d(1) - prev_position(1), ...
                                  current_pos_3d(3) - prev_position(3)]);
            if dt > 0
                instant_velocity = position_delta / dt;
                current_velocity_mms = velocity_smoothing_alpha * instant_velocity + ...
                                      (1 - velocity_smoothing_alpha) * current_velocity_mms;
            end
        else
            position_init = true;
        end
        prev_position = current_pos_3d;

        % -- CHECK IF ALREADY NEAR START POSITION (skip initial approach)
        if strcmp(control_state, 'INITIAL_APPROACH') && position_init
            distance_to_start = norm([start_position(1) - current_pos_3d(1), start_position(3) - current_pos_3d(3)]);
            
            if distance_to_start < distance_tolerance * 2
                disp(['Already near start position (distance: ' num2str(distance_to_start) ' mm). Preparing for alignment...']);
                first_start_reached = true;
                control_state = 'ALIGNMENT_PREP';
                alignment_pause_timer = tic;
                integral_w = 0; prev_error_w = 0;
            end
        end

        % -- POSITION TRACKING
        if strcmp(control_state, 'INITIAL_APPROACH')
            distance_to_start = norm([start_position(1) - current_pos_3d(1), start_position(3) - current_pos_3d(3)]);
            
            if distance_to_start < distance_tolerance && ~first_start_reached
                disp('Start position reached. Preparing for alignment...');
                first_start_reached = true;
                control_state = 'ALIGNMENT_PREP';
                alignment_pause_timer = tic;
                integral_w = 0; prev_error_w = 0;
            end
        else
            % Check if goal reached (only during APF control)
            if strcmp(control_state, 'APF_CONTROL')
                distance_to_goal = norm([goal_position(1) - current_pos_3d(1), goal_position(3) - current_pos_3d(3)]);
                
                if distance_to_goal < distance_tolerance
                    disp('Goal reached!');
                    send_robot_command(robot_ip, 'S');
                    break;
                end
            end
        end

        % -- CONTROL STATE MACHINE
        forward_speed = 0;
        turn_speed = 0;
        left_speed = 0;
        right_speed = 0;
        apf_desired_heading = 0;

        switch control_state
            case 'INITIAL_APPROACH'
                % Simple control to reach start position
                fprintf('INITIAL_APPROACH | ');
                
                distance_to_start = norm([start_position(1) - current_pos_3d(1), start_position(3) - current_pos_3d(3)]);
                forward_speed = min(0.3 * distance_to_start, 60);
                
                % Calculate heading to start position
                start_vec = [start_position(1) - current_pos_3d(1), start_position(3) - current_pos_3d(3)];
                target_angle = rad2deg(atan2(start_vec(1), start_vec(2)));
                heading_error = target_angle - current_angle;
                heading_error = atan2(sin(deg2rad(heading_error)), cos(deg2rad(heading_error)));
                heading_error = rad2deg(heading_error);
                
                turn_speed = 1.0 * heading_error;
                turn_speed = min(max(turn_speed, -100), 100);
                
                left_speed = round(forward_speed + turn_speed);
                right_speed = round(forward_speed - turn_speed);
                
                fprintf('Distance: %.1f mm\n', distance_to_start);

            case 'ALIGNMENT_PREP'
                % Pause for 1 second before alignment
                fprintf('ALIGNMENT_PREP | Pausing...\n');
                
                % Stop the robot
                left_speed = 0;
                right_speed = 0;
                
                % Check if 1 second has passed
                if toc(alignment_pause_timer) >= 1
                    disp('Starting alignment to goal...');
                    control_state = 'ALIGNING';
                end

            case 'ALIGNING'
                % Align to face goal position using PID control
                fprintf('ALIGNING | ');
                
                % Calculate heading to goal
                goal_vec = [goal_position(1) - current_pos_3d(1), goal_position(3) - current_pos_3d(3)];
                target_angle = rad2deg(atan2(goal_vec(1), goal_vec(2)));
                heading_error = target_angle - current_angle;
                heading_error = atan2(sin(deg2rad(heading_error)), cos(deg2rad(heading_error)));
                heading_error = rad2deg(heading_error);
                
                fprintf('Heading error to goal: %.1f deg\n', abs(heading_error));
                
                if abs(heading_error) < alignment_heading_tolerance
                    disp('Alignment complete. Starting APF control...');
                    control_state = 'APF_CONTROL';
                    apf_start_time = tic;
                    % Reset PID integrals for APF control
                    integral_w = 0; prev_error_w = 0;
                else
                    % PID control for precise alignment
                    integral_w = integral_w + heading_error * dt;
                    derivative_w = (heading_error - prev_error_w) / dt;
                    
                    pid_output = Kp_w * heading_error + Ki_w * integral_w + Kd_w * derivative_w;
                    prev_error_w = heading_error;
                    
                    % Determine turn direction and magnitude
                    turn_magnitude = min(abs(pid_output), 100);  % Magnitude only (0-100)
                    
                    if pid_output > 0
                        % Turn right (positive error - need to turn clockwise)
                        left_speed = round(turn_magnitude);
                        right_speed = round(-turn_magnitude);
                    else
                        % Turn left (negative error - need to turn counter-clockwise)  
                        left_speed = round(-turn_magnitude);
                        right_speed = round(turn_magnitude);
                    end
                end

            case 'APF_CONTROL'
                % APF-based obstacle avoidance
                current_time = toc(apf_start_time);
                fprintf('APF_CONTROL | Time: %.1f s | ', current_time);
                
                % -- Calculate APF Forces
                current_pos_2d = [current_pos_3d(1); current_pos_3d(3)];  % XZ plane
                goal_pos_2d = [goal_position(1); goal_position(3)];
                
                if obstacle_detected
                    obstacle_pos_2d = [obstacle_position(1); obstacle_position(3)];
                    [F_att, F_rep, F_combined] = calculate_apf_forces(current_pos_2d, goal_pos_2d, ...
                        obstacle_pos_2d, detection_radius_mm, attraction_factor, repulsion_factor);
                else
                    % No obstacle detected, just attraction to goal
                    goal_vector = goal_pos_2d - current_pos_2d;
                    if norm(goal_vector) > 0
                        F_att = attraction_factor * (goal_vector / norm(goal_vector));
                    else
                        F_att = [0; 0];
                    end
                    F_rep = [0; 0];
                    F_combined = F_att;
                end
                
                % -- Calculate desired heading from APF forces
                if norm(F_combined) > 0.01
                    apf_desired_heading = rad2deg(atan2(F_combined(1), F_combined(2)));
                else
                    % Fallback: head directly to goal
                    goal_vec = [goal_position(1) - current_pos_3d(1), goal_position(3) - current_pos_3d(3)];
                    apf_desired_heading = rad2deg(atan2(goal_vec(1), goal_vec(2)));
                end
                
                % -- PID Control for heading
                heading_error = apf_desired_heading - current_angle;
                heading_error = atan2(sin(deg2rad(heading_error)), cos(deg2rad(heading_error)));
                heading_error = rad2deg(heading_error);
                
                integral_w = integral_w + heading_error * dt;
                derivative_w = (heading_error - prev_error_w) / dt;
                
                angular_adjustment = Kp_w * heading_error + Ki_w * integral_w + Kd_w * derivative_w;
                prev_error_w = heading_error;
                
                angular_adjustment = min(max(angular_adjustment, -max_motor_speed), max_motor_speed);
                
                % -- Fixed linear velocity with angular control
                forward_speed = desired_motor_speed;
                left_speed = round(forward_speed + angular_adjustment);
                right_speed = round(forward_speed - angular_adjustment);
                
                distance_to_goal = norm([goal_position(1) - current_pos_3d(1), goal_position(3) - current_pos_3d(3)]);
                
                fprintf('APF Head: %.1f° | Actual: %.1f° | Goal dist: %.1f mm', ...
                        apf_desired_heading, current_angle, distance_to_goal);
                
                if obstacle_detected
                    obstacle_distance = norm([obstacle_position(1) - current_pos_3d(1), obstacle_position(3) - current_pos_3d(3)]);
                    fprintf(' | Obs dist: %.1f mm', obstacle_distance);
                end
                fprintf('\n');
        end

        % Final speed constraints
        left_speed = round(min(max(left_speed, -255), 255));
        right_speed = round(min(max(right_speed, -255), 255));

        % Send command to robot
        json_command = sprintf('{"N":4,"D1":%d,"D2":%d,"H":"apf"}', left_speed, right_speed);
        send_robot_command(robot_ip, json_command);

        % -- DUAL VISUALIZATION UPDATE
        % Update XZ trajectory plot
        set(h_robot, 'XData', current_pos_3d(1), 'YData', current_pos_3d(2), 'ZData', current_pos_3d(3));
        
        % Update robot orientation
        % current_angle = 0° means robot points in +Z direction
        % current_angle = 90° means robot points in +X direction
        orientation_length = 250;
        current_angle_rad = deg2rad(current_angle);
        orientation_end_pos = [current_pos_3d(1) + orientation_length * sin(current_angle_rad), ...
                              current_pos_3d(2), ...
                              current_pos_3d(3) + orientation_length * cos(current_angle_rad)];
        set(h_robot_orientation, 'XData', [current_pos_3d(1), orientation_end_pos(1)], ...
                              'YData', [current_pos_3d(2), orientation_end_pos(2)], ...
                              'ZData', [current_pos_3d(3), orientation_end_pos(3)]);
        
        % Update obstacle position and detection circles
        if obstacle_detected
            set(h_obstacle, 'XData', obstacle_position(1), 'YData', obstacle_position(2), 'ZData', obstacle_position(3));
            
            % Update detection and safe radius circles
            detection_circle_x = obstacle_position(1) + detection_radius_mm * cos(theta_circle);
            detection_circle_z = obstacle_position(3) + detection_radius_mm * sin(theta_circle);
            detection_circle_y = obstacle_position(2) * ones(size(theta_circle));
            % set(h_detection_circle, 'XData', detection_circle_x, 'YData', detection_circle_y, 'ZData', detection_circle_z);
            
            safe_circle_x = obstacle_position(1) + safe_radius_mm * cos(theta_circle);
            safe_circle_z = obstacle_position(3) + safe_radius_mm * sin(theta_circle);
            safe_circle_y = obstacle_position(2) * ones(size(theta_circle));
            % set(h_safe_circle, 'XData', safe_circle_x, 'YData', safe_circle_y, 'ZData', safe_circle_z);
        end
        
        % Update trajectory
        trajectory_points(:, end+1) = current_pos_3d;
        set(h_trajectory, 'XData', trajectory_points(1,:), 'YData', trajectory_points(2,:), 'ZData', trajectory_points(3,:));
        
        % Update heading plot (only during APF control)
        if strcmp(control_state, 'APF_CONTROL') && ~isempty(apf_start_time)
            current_apf_time = toc(apf_start_time);
            heading_time_data(end+1) = current_apf_time;
            heading_angle_data(end+1) = current_angle;
            apf_heading_data(end+1) = apf_desired_heading;
            
            % Update heading lines
            set(h_current_heading, 'XData', heading_time_data, 'YData', heading_angle_data);
            set(h_apf_heading, 'XData', heading_time_data, 'YData', apf_heading_data);
            set(h_heading_point, 'XData', current_apf_time, 'YData', current_angle);
            
            % Update time axis if needed
            if current_apf_time > 10
                xlim(ax2, [0, current_apf_time + 2]);
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
% 4. CLEANUP
% =======================================================================
disp('Disconnecting NatNet client...');
send_robot_command(robot_ip, 'S');
natnetclient.disconnect;
clear natnetclient;

% =======================================================================
% 5. HELPER FUNCTIONS
% =======================================================================

function [F_att, F_rep, F_combined] = calculate_apf_forces(current_pos, goal, obstacle, ...
    detection_radius, attraction_factor, repulsion_factor)
    % Calculate APF forces based on the original apf_obstacle_avoidance.m
    
    % Calculate attraction to goal
    goal_vector = goal - current_pos;
    if norm(goal_vector) > 0
        F_att = attraction_factor * (goal_vector / norm(goal_vector));
    else
        F_att = [0; 0];
    end
    
    % Calculate repulsion from obstacle
    F_rep = [0; 0];
    if ~isempty(obstacle)
        obstacle_distance = norm(obstacle - current_pos);
        
        % Apply repulsion if within detection range
        if obstacle_distance <= detection_radius && obstacle_distance > 0
            obstacle_vector = current_pos - obstacle;
            F_rep = (obstacle_vector / norm(obstacle_vector)) * ...
                   (detection_radius - obstacle_distance) / detection_radius;
            F_rep = repulsion_factor * F_rep;
        end
    end
    
    % Combine forces
    F_combined = F_att + F_rep;
    
    % Normalize combined force
    if norm(F_combined) > 0
        F_combined = F_combined / norm(F_combined);
    end
end

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