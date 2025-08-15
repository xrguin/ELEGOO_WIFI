function apf_waypoint_pid
% =======================================================================
% apf_waypoint_pid.m
%
% This script has been modified to control TWO robots simultaneously using
% an Artificial Potential Field (APF) algorithm for obstacle avoidance.
%
% Key Features:
% - Connects to OptiTrack (NatNet) for live pose data of two robots.
% - Uses `generateInterceptingPathSimple` to create goals for both robots.
% - APF calculates a desired heading based on attraction to a goal and
%   repulsion from the other robot.
% - A dynamic, temporary waypoint is generated based on the APF heading.
% - A PID controller steers the robot towards the temporary waypoint while
%   maintaining a constant base forward speed.
% - Commands are sent to each robot via TCP/IP.
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
% 1. CONFIGURATION - !!! USER MUST EDIT THIS SECTION !!!
% =======================================================================

% -- Robot Configuration
robot_ips = {"192.168.1.140", "192.168.1.141"}; % IPs for Robot 1 and Robot 2
robot_ids = [1, 2];                             % Rigid Body IDs for Robot 1 and 2 in Motive

% -- NatNet Server Configuration
natnet_client_ip = '192.168.1.70';
natnet_server_ip = '192.168.1.209';

% -- Coordinate System Transformation (from OptiTrack to Simulator)
% This script uses a 2D coordinate system for APF calculations.
% new_X = optitrack_Z + z_offset
% new_Y = optitrack_X + x_offset
x_offset = 2200; % Amount to add to OptiTrack X to get Simulator Y (mm)
z_offset = 2400; % Amount to add to OptiTrack Z to get Simulator X (mm)

% -- APF & Path Generation Parameters (all units in mm)
attraction_factor = 1.0;
repulsion_factor = 1.5;
detection_radius = 1000; % APF repulsion activates within this radius (1m)
safe_radius = 300;       % Safety radius for path generation (0.3m)
dynamic_waypoint_distance = 200; % How far ahead to set the temp waypoint (mm)

% -- PID Controller Gains (Shared for both robots, tune as needed)
Kp_h = 0.85;  % Heading Proportional Gain
Ki_h = 0.03;  % Heading Integral Gain
Kd_h = 0.15;  % Heading Derivative Gain

% -- Control & Physics Parameters
distance_tolerance = 100;  % Stop when within 100 mm (10 cm) of the final goal.
base_forward_speed = 70;   % Constant base speed when driving (0-255).
max_turn_component = 50;   % Max speed adjustment for turning.

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
if (model.RigidBodyCount < 2), error('This script requires at least 2 rigid bodies.'); end
disp('NatNet connection successful.');

% -- Path Generation (in Simulator Coordinates)
disp('Generating intercepting paths for robots...');
% Arena bounds (in mm, used for placing start/goal points)
x_bound_gen = 2000;
y_bound_gen = 2000;
min_path_length = 1500;

% Generate paths using helper functions
[start1_sim, goal1_sim] = generate_random_path(x_bound_gen, y_bound_gen, min_path_length);
[start2_sim, goal2_sim, ~] = generateInterceptingPathSimple(start1_sim, goal1_sim, ...
    x_bound_gen, y_bound_gen, min_path_length, safe_radius * 2);

goals_sim = [goal1_sim, goal2_sim]; % Store goals in a 2x2 matrix [x1, x2; y1, y2]

fprintf('Robot 1 Path (Sim Coords): Start [%.0f, %.0f], Goal [%.0f, %.0f]\n', start1_sim(1), start1_sim(2), goal1_sim(1), goal1_sim(2));
fprintf('Robot 2 Path (Sim Coords): Start [%.0f, %.0f], Goal [%.0f, %.0f]\n', start2_sim(1), start2_sim(2), goal2_sim(1), goal2_sim(2));

% -- Initialize 2D Plot
figure;
ax = axes;
hold(ax, 'on'); grid on; axis equal;
title('Dual Robot APF Control');
xlabel('Simulator X (mm)'); ylabel('Simulator Y (mm)');
view(ax, 2); % Set to 2D view

% -- Plot Handles
h_goal1 = plot(ax, goal1_sim(1), goal1_sim(2), 'gp', 'MarkerSize', 15, 'MarkerFaceColor', 'g');
h_goal2 = plot(ax, goal2_sim(1), goal2_sim(2), 'bp', 'MarkerSize', 15, 'MarkerFaceColor', 'b');
h_robot1 = plot(ax, 0, 0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
h_robot2 = plot(ax, 0, 0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
h_traj1 = plot(ax, NaN, NaN, 'g-'); % Use NaN to initialize empty plot
h_traj2 = plot(ax, NaN, NaN, 'b-');
h_temp_wp1 = plot(ax, 0, 0, 'gx', 'MarkerSize', 8); % Temporary waypoint for Robot 1
h_temp_wp2 = plot(ax, 0, 0, 'bx', 'MarkerSize', 8); % Temporary waypoint for Robot 2
legend(ax, {'Goal 1', 'Goal 2', 'Robot 1', 'Robot 2', 'Traj 1', 'Traj 2', 'Temp WP1', 'Temp WP2'}, 'Location', 'best');

% -- Initialize PID & Loop Variables for TWO robots
integral_h = [0, 0];
prev_error_h = [0, 0];
trajectory_points_1 = zeros(2, 0);  % Initialize as 2x0 empty array
trajectory_points_2 = zeros(2, 0);  % Initialize as 2x0 empty array
last_loop_time = tic;

disp('Initialization complete. Starting control loop...');

% =======================================================================
% 3. MAIN CONTROL LOOP
% =======================================================================
try
    while true
        dt = toc(last_loop_time);
        last_loop_time = tic;

        frame = natnetclient.getFrame;
        if isempty(frame.RigidBodies)
            disp('No rigid bodies in frame...');
            pause(0.01);
            continue;
        end

        % --- Find and validate robot data ---
        rb_idx_1 = arrayfun(@(rb) rb.ID == robot_ids(1), frame.RigidBodies);
        rb_idx_2 = arrayfun(@(rb) rb.ID == robot_ids(2), frame.RigidBodies);

        if ~any(rb_idx_1) || ~any(rb_idx_2)
            disp('Waiting for both robots to be tracked...');
            pause(0.1);
            continue;
        end
        
        rb1 = frame.RigidBodies(rb_idx_1);
        rb2 = frame.RigidBodies(rb_idx_2);

        % --- Data Acquisition for Both Robots ---
        pos_opti_1 = [rb1.x, rb1.y, rb1.z] * 1000;
        pos_opti_2 = [rb2.x, rb2.y, rb2.z] * 1000;

        q1 = quaternion(rb1.qw, rb1.qx, rb1.qy, rb1.qz);
        angles1 = q1.EulerAngles('YZX');
        current_angle_1 = wrapToPi(deg2rad(rad2deg(angles1(1)) + 230));

        q2 = quaternion(rb2.qw, rb2.qx, rb2.qy, rb2.qz);
        angles2 = q2.EulerAngles('YZX');
        current_angle_2 = wrapToPi(deg2rad(rad2deg(angles2(1)) + 230));

        % --- Coordinate Transformation ---
        pos_sim_1 = [pos_opti_1(3) + z_offset; pos_opti_1(1) + x_offset];
        pos_sim_2 = [pos_opti_2(3) + z_offset; pos_opti_2(1) + x_offset];

        positions_sim = [pos_sim_1, pos_sim_2];
        current_angles_rad = [current_angle_1, current_angle_2];

        % --- Check for Goal Reached ---
        dist_to_goal_1 = norm(pos_sim_1 - goal1_sim);
        dist_to_goal_2 = norm(pos_sim_2 - goal2_sim);

        if dist_to_goal_1 < distance_tolerance && dist_to_goal_2 < distance_tolerance
            disp('Both robots reached their goals!');
            send_robot_command(robot_ips{1}, 'S');
            send_robot_command(robot_ips{2}, 'S');
            break;
        end

        % --- Loop through each robot to calculate and send commands ---
        for i = 1:2
            current_pos = positions_sim(:, i);
            current_angle = current_angles_rad(i);
            goal_pos = goals_sim(:, i);
            other_robot_pos = positions_sim(:, 3-i); % If i=1, other robot is 2. If i=2, other robot is 1.

            % --- APF Calculation ---
            [~, ~, F_combined] = calculate_apf_forces(current_pos, goal_pos, ...
                other_robot_pos, detection_radius, attraction_factor, repulsion_factor);

            % --- Dynamic Waypoint Generation ---
            if norm(F_combined) > 0.01
                desired_heading = atan2(F_combined(2), F_combined(1));
            else
                goal_direction = goal_pos - current_pos;
                desired_heading = atan2(goal_direction(2), goal_direction(1));
            end
            temp_waypoint = current_pos + dynamic_waypoint_distance * [cos(desired_heading); sin(desired_heading)];

            % --- PID Control to reach temporary waypoint ---
            error_vec = temp_waypoint - current_pos;
            target_angle = atan2(error_vec(2), error_vec(1));
            heading_error = wrapToPi(target_angle - current_angle);
            
            integral_h(i) = integral_h(i) + rad2deg(heading_error) * dt;
            derivative_h = (rad2deg(heading_error) - prev_error_h(i)) / dt;
            turn_component = Kp_h * rad2deg(heading_error) + Ki_h * integral_h(i) + Kd_h * derivative_h;
            prev_error_h(i) = rad2deg(heading_error);
            
            turn_component = min(max(turn_component, -max_turn_component), max_turn_component);

            % --- Motor Command Generation ---
            if norm(goal_pos - current_pos) < distance_tolerance
                left_speed = 0; right_speed = 0;
            else
                left_speed = round(base_forward_speed + turn_component);
                right_speed = round(base_forward_speed - turn_component);
            end

            left_speed  = round(min(max(left_speed, -255), 255));
            right_speed = round(min(max(right_speed, -255), 255));

            json_command = sprintf('{"N":4,"D1":%d,"D2":%d,"H":"pid"}', left_speed, right_speed);
            send_robot_command(robot_ips{i}, json_command);

            % --- Store data for plotting ---
            if i == 1
                trajectory_points_1(:, end+1) = current_pos;
                set(h_robot1, 'XData', current_pos(1), 'YData', current_pos(2));
                set(h_traj1, 'XData', trajectory_points_1(1,:), 'YData', trajectory_points_1(2,:));
                set(h_temp_wp1, 'XData', temp_waypoint(1), 'YData', temp_waypoint(2));
            else
                trajectory_points_2(:, end+1) = current_pos;
                set(h_robot2, 'XData', current_pos(1), 'YData', current_pos(2));
                set(h_traj2, 'XData', trajectory_points_2(1,:), 'YData', trajectory_points_2(2,:));
                set(h_temp_wp2, 'XData', temp_waypoint(1), 'YData', temp_waypoint(2));
            end
        end
        drawnow limitrate;
    end
catch e
    disp('An error occurred. Stopping robots and disconnecting.');
    send_robot_command(robot_ips{1}, 'S');
    send_robot_command(robot_ips{2}, 'S');
    natnetclient.disconnect;
    rethrow(e);
end

% =======================================================================
% 4. CLEANUP
% =======================================================================
disp('Disconnecting NatNet client...');
send_robot_command(robot_ips{1}, 'S');
send_robot_command(robot_ips{2}, 'S');
natnetclient.disconnect;
clear natnetclient;
disp('Script finished.');

end

% =======================================================================
% 5. HELPER FUNCTIONS
% =======================================================================

function response = send_robot_command(ip, command)
    try
        t = tcpclient(ip, 80, 'ConnectTimeout', 0.5, 'Timeout', 1);
        data_to_send = [char(command), newline];
        write(t, data_to_send);
        
        % Optional: Wait for acknowledgment if your robot sends one
        % pause(0.01);  % Small delay to ensure data is sent
        
        response = 'OK';
        clear t;
    catch e
        response = ['Error sending to ', char(ip), ': ', e.message];
        fprintf('WARNING: Failed to send command to robot at %s\n', char(ip));
        fprintf('  Command: %s\n', char(command));
        fprintf('  Error: %s\n', e.message);
    end
end

function q = quaternion(w, x, y, z)
    q.w = w;
    q.x = x;
    q.y = y;
    q.z = z;
    q.EulerAngles = @(order) quat2eul([q.w, q.x, q.y, q.z], order);
end

function angle = wrapToPi(angle)
    angle = atan2(sin(angle), cos(angle));
end

function [F_att, F_rep, F_combined] = calculate_apf_forces(current_pos, goal, other_robot_pos, ...
    detection_radius, attraction_factor, repulsion_factor)
    
    % Calculate attractive force towards goal
    goal_vector = goal - current_pos;
    goal_dist = norm(goal_vector);
    if goal_dist > 1e-6  % Avoid division by zero
        F_att = attraction_factor * (goal_vector / goal_dist);
    else
        F_att = [0; 0];
    end
    
    % Calculate repulsive force from other robot
    F_rep = [0; 0];
    if ~isempty(other_robot_pos)
        obstacle_vector = current_pos - other_robot_pos;
        dist_to_obstacle = norm(obstacle_vector);
        
        if dist_to_obstacle <= detection_radius && dist_to_obstacle > 1e-6  % Avoid division by zero
            % Normalize the repulsion vector and scale by distance
            F_rep = (obstacle_vector / dist_to_obstacle) * ...
                   (detection_radius - dist_to_obstacle) / dist_to_obstacle;
            F_rep = repulsion_factor * F_rep;
        end
    end
    
    % Combine forces
    F_combined = F_att + F_rep;
    
    % Normalize combined force
    combined_norm = norm(F_combined);
    if combined_norm > 1e-6  % Avoid division by zero
        F_combined = F_combined / combined_norm;
    end
end

function [start, goal] = generate_random_path(x_bound, y_bound, min_path_length)
    max_attempts = 20;
    for attempt = 1:max_attempts
        start = [(rand()*2-1)*x_bound; (rand()*2-1)*y_bound];
        angle = rand() * 2 * pi;
        path_length = min_path_length + rand() * (min_path_length/2);
        goal = start + path_length * [cos(angle); sin(angle)];
        
        goal(1) = max(-x_bound, min(x_bound, goal(1)));
        goal(2) = max(-y_bound, min(y_bound, goal(2)));
        
        if norm(goal - start) >= min_path_length
            return;
        end
    end
    error('Failed to generate a valid random path.');
end

function [newStart, newGoal, interceptPoint] = generateInterceptingPathSimple(refStart, refGoal, ...
    x_bound, y_bound, min_path_length, safeRadius)
    
    max_attempts = 100;
    for attempt = 1:max_attempts
        dir = refGoal - refStart;
        t = 0.4 + rand() * 0.2;
        interceptPoint = refStart + t * dir;
        
        if all(abs(interceptPoint) <= [x_bound; y_bound])
            radius = norm(interceptPoint - refStart);
            newStart = generateRandomPointOnCircleBounded(interceptPoint, radius, x_bound, y_bound);
            
            if norm(newStart - refStart) > safeRadius
                dir_to_goal = interceptPoint - newStart;
                newGoal = interceptPoint + dir_to_goal;
                newGoal(1) = max(-x_bound, min(x_bound, newGoal(1)));
                newGoal(2) = max(-y_bound, min(y_bound, newGoal(2)));
                
                if norm(newGoal - newStart) >= min_path_length
                    return;
                end
            end
        end
    end
    
    % Fallback if no intercepting path is found
    [newStart, newGoal] = generate_random_path(x_bound, y_bound, min_path_length);
    interceptPoint = (newStart + newGoal) / 2;
end

function point = generateRandomPointOnCircleBounded(center, radius, x_bound, y_bound)
    for attempt = 1:50
        theta = 2 * pi * rand();
        point = center + radius * [cos(theta); sin(theta)];
        if all(abs(point) <= [x_bound; y_bound])
            return;
        end
    end
    % Fallback
    point = center;
end
