% =======================================================================
% APF_Dual_Robot_Avoidance.m
%
% APF-based mutual avoidance with two synchronized robots using OptiTrack.
% - Each robot treats the other as a dynamic obstacle.
% - Robots have swapped start and goal positions.
% - Robots wait for each other to finish alignment before moving.
% - APF logic is structurally identical to the original static example.
% - Includes plots for Robot 1 perspective and heading control.
% =======================================================================

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
% 1. ROBOT AND SCENARIO CONFIGURATION
% =======================================================================

% -- Robot Configuration (IP to Rigid Body ID mapping)
robot_configs = struct();
robot_configs(1).ip = "192.168.1.140";
robot_configs(1).rigid_body_id = 1;
robot_configs(1).name = "Robot_1";

robot_configs(2).ip = "192.168.1.84";
robot_configs(2).rigid_body_id = 2;
robot_configs(2).name = "Robot_2";

% -- Active Robot IDs
robot1_id = 1;
robot2_id = 2;

robot1_ip = robot_configs(robot1_id).ip;
robot1_rigid_body_id = robot_configs(robot1_id).rigid_body_id;
robot2_ip = robot_configs(robot2_id).ip;
robot2_rigid_body_id = robot_configs(robot2_id).rigid_body_id;

fprintf('Controlling Robot 1 (ID: %d) and Robot 2 (ID: %d)\n', robot1_rigid_body_id, robot2_rigid_body_id);

% -- NatNet Server Configuration
natnet_client_ip = '192.168.1.203';
natnet_server_ip = '192.168.1.209';

% -- Start and Goal Positions (Swapped for each robot)
original_start = [1450, 23, 707] - [100, 0, 100];
original_goal = [-972.4, 42, -2017.3] - [100, 0, 100];

robot1_start_pos = original_start;
robot1_goal_pos = original_goal;
robot2_start_pos = original_goal;
robot2_goal_pos = original_start;

fprintf('Robot 1: Start [%.1f, %.1f, %.1f], Goal [%.1f, %.1f, %.1f]\n', robot1_start_pos, robot1_goal_pos);
fprintf('Robot 2: Start [%.1f, %.1f, %.1f], Goal [%.1f, %.1f, %.1f]\n', robot2_start_pos, robot2_goal_pos);

% -- Velocity Profile Parameters
desired_motor_speed = 80;

% -- APF Parameters (same for both robots)
attraction_factor = 1.2;
repulsion_factor = 2; 
detection_radius_mm = 800;

% -- PID Controller Gains for Angular Control (same for both robots)
Kp_w = 0.8;
Ki_w = 0.2;
Kd_w = 0.15;

% -- Control Parameters
distance_tolerance = 80; % Goal reached tolerance (mm)
alignment_heading_tolerance = 20; % Alignment phase tolerance (degrees)

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
disp('NatNet connection successful.');

% -- Initialize World View Plot
figure('Position', [50, 100, 800, 700]);
ax_world = gca;
hold(ax_world, 'on'); grid on; axis equal;
title(ax_world, 'APF Dual Robot Mutual Avoidance (World View)');
xlabel(ax_world, 'X (mm)'); ylabel(ax_world, 'Y (mm)'); zlabel(ax_world, 'Z (mm)');
view(ax_world, 0, 0);

% -- Plot Boundaries and Axis Limits
boundary_pts = [ 2467,   50,  148; 1814,   65, -2659; -1878,   37, -2732; -1101,  6.8,  1769];
closed_boundary = [boundary_pts; boundary_pts(1,:)];
plot3(ax_world, closed_boundary(:,1), closed_boundary(:,2), closed_boundary(:,3), 'k-', 'LineWidth', 2, 'HandleVisibility', 'off');
padding = 500;
xlim(ax_world, [min(boundary_pts(:,1)) - padding, max(boundary_pts(:,1)) + padding]);
ylim(ax_world, [min(boundary_pts(:,2)) - padding, max(boundary_pts(:,2)) + padding]);
zlim(ax_world, [min(boundary_pts(:,3)) - padding, max(boundary_pts(:,3)) + padding]);
set(ax_world, 'ZDir', 'reverse');

% -- World Plot Handles
plot3(ax_world, robot1_start_pos(1), robot1_start_pos(2), robot1_start_pos(3), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'c');
plot3(ax_world, robot1_goal_pos(1), robot1_goal_pos(2), robot1_goal_pos(3), 'b*', 'MarkerSize', 15, 'LineWidth', 2, 'DisplayName', 'R1 Goal');
h_robot1 = plot3(ax_world, 0, 0, 0, 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b', 'DisplayName', 'Robot 1');
h_robot1_orientation = line(ax_world, [0 0], [0 0], [0 0], 'Color', 'b', 'LineWidth', 2);
h_trajectory1 = plot3(ax_world, 0, 0, 0, 'b-', 'LineWidth', 2, 'DisplayName', 'R1 Trajectory');
plot3(ax_world, robot2_start_pos(1), robot2_start_pos(2), robot2_start_pos(3), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'c');
plot3(ax_world, robot2_goal_pos(1), robot2_goal_pos(2), robot2_goal_pos(3), 'g*', 'MarkerSize', 15, 'LineWidth', 2, 'DisplayName', 'R2 Goal');
h_robot2 = plot3(ax_world, 0, 0, 0, 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'Robot 2');
h_robot2_orientation = line(ax_world, [0 0], [0 0], [0 0], 'Color', 'g', 'LineWidth', 2);
h_trajectory2 = plot3(ax_world, 0, 0, 0, 'g-', 'LineWidth', 2, 'DisplayName', 'R2 Trajectory');
legend(ax_world, 'Location', 'best');

% -- Initialize Robot 1 Perspective Plot
% figure('Position', [900, 450, 600, 500]);
% ax_vision = gca;
% hold(ax_vision, 'on'); grid on; axis equal;
% title(ax_vision, 'Robot 1 Perspective');
% xlabel(ax_vision, 'Relative X (mm)'); ylabel(ax_vision, 'Relative Z (mm)');
% h_vision_self = plot(ax_vision, 0, 0, 'bo', 'MarkerSize', 12, 'MarkerFaceColor', 'b', 'DisplayName', 'Robot 1');
% h_vision_self_orient = line(ax_vision, [0 0], [0 250], 'Color', 'b', 'LineWidth', 3, 'DisplayName', 'R1 Heading');
% h_vision_obstacle = plot(ax_vision, 0, 0, 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'Obstacle (R2)');
% h_vision_goal = plot(ax_vision, 0, 0, 'r*', 'MarkerSize', 15, 'LineWidth', 2, 'DisplayName', 'Goal');
% h_vision_traj = plot(ax_vision, 0, 0, 'k:', 'LineWidth', 1, 'DisplayName', 'Path History');
% legend(ax_vision, 'Location', 'best');

% -- Initialize Robot 1 Heading Plot
figure('Position', [900, 50, 600, 350]);
ax_heading = gca;
hold(ax_heading, 'on'); grid on;
title(ax_heading, 'Robot 1 APF Heading Control');
xlabel(ax_heading, 'Time (s)'); ylabel(ax_heading, 'Heading (degrees)');
ylim(ax_heading, [-180, 180]);
h_apf_heading = plot(ax_heading, 0, 0, 'r-', 'LineWidth', 2, 'DisplayName', 'APF Desired Heading');
h_current_heading = plot(ax_heading, 0, 0, 'b-', 'LineWidth', 2, 'DisplayName', 'Actual Heading');
h_heading_point = plot(ax_heading, 0, 0, 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b', 'DisplayName', 'Current');
legend(ax_heading, 'Location', 'best');

figure('Position', [900, 50, 600, 350]);
ax_heading = gca;
hold(ax_heading, 'on'); grid on;
title(ax_heading, 'Robot 2 APF Heading Control');
xlabel(ax_heading, 'Time (s)'); ylabel(ax_heading, 'Heading (degrees)');
ylim(ax_heading, [-180, 180]);
h_apf_heading = plot(ax_heading, 0, 0, 'r-', 'LineWidth', 2, 'DisplayName', 'APF Desired Heading');
h_current_heading = plot(ax_heading, 0, 0, 'b-', 'LineWidth', 2, 'DisplayName', 'Actual Heading');
h_heading_point = plot(ax_heading, 0, 0, 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b', 'DisplayName', 'Current');
legend(ax_heading, 'Location', 'best');

% -- Initialize State Variables
% Robot 1
r1_state = 'INITIAL_APPROACH'; r1_integral_w = 0; r1_prev_error_w = 0; r1_traj_pts = []; r1_last_valid_pos = [0,0,0]; r1_goal_reached = false; r1_alignment_timer = 0;
r1_apf_start_time = []; r1_heading_time_data = []; r1_heading_angle_data = []; r1_apf_heading_data = [];

% Robot 2
r2_state = 'INITIAL_APPROACH'; r2_integral_w = 0; r2_prev_error_w = 0; r2_traj_pts = []; r2_last_valid_pos = [0,0,0]; r2_goal_reached = false; r2_alignment_timer = 0;
r2_apf_start_time = []; r2_heading_time_data = []; r2_heading_angle_data = []; r2_apf_heading_data = [];

last_loop_time = tic;
disp('Initialization complete. Starting dual robot avoidance...');

% =======================================================================
% 3. MAIN CONTROL LOOP
% =======================================================================
try
    while true
        dt = toc(last_loop_time);
        last_loop_time = tic;

        data = natnetclient.getFrame;
        if isempty(data.RigidBodies) || data.nRigidBodies < 2, disp('Waiting for at least two rigid bodies...'); pause(0.01); continue; end

        % -- Get Data for Both Robots
        r1_data = data.RigidBodies(robot1_rigid_body_id);
        r1_pos_raw = [r1_data.x, r1_data.y, r1_data.z] * 1000;
        r2_data = data.RigidBodies(robot2_rigid_body_id);
        r2_pos_raw = [r2_data.x, r2_data.y, r2_data.z] * 1000;

        % -- Process Robot 1 Data
        if norm(r1_pos_raw) < 1, r1_pos = r1_last_valid_pos; else, r1_pos = r1_pos_raw; r1_last_valid_pos = r1_pos; end
        q1 = quaternion(r1_data.qw, r1_data.qx, r1_data.qy, r1_data.qz);
        angles1 = q1.EulerAngles('YZX');
        r1_angle = rad2deg(angles1(1)) + 230;
        if r1_angle >= 180, r1_angle = -360 + r1_angle; end

        % -- Process Robot 2 Data
        if norm(r2_pos_raw) < 1, r2_pos = r2_last_valid_pos; else, r2_pos = r2_pos_raw; r2_last_valid_pos = r2_pos; end
        q2 = quaternion(r2_data.qw, r2_data.qx, r2_data.qy, r2_data.qz);
        angles2 = q2.EulerAngles('YZX');
        r2_angle = rad2deg(angles2(1)) + 230;
        if r2_angle >= 180, r2_angle = -360 + r2_angle; end
        
        if r1_goal_reached && r2_goal_reached, disp('Both robots reached goals!'); send_robot_command(robot1_ip, 'S'); send_robot_command(robot2_ip, 'S'); break; end

        % -- ROBOT 1 CONTROL
        [r1_left, r1_right, r1_state, r1_iw, r1_pew, r1_gr, r1_at, r1_apf_head] = ...
            robot_control_logic(dt, r1_state, r1_pos, r1_angle, robot1_start_pos, robot1_goal_pos, r2_pos, r1_integral_w, r1_prev_error_w, r1_goal_reached, r1_alignment_timer, Kp_w, Ki_w, Kd_w, attraction_factor, repulsion_factor, detection_radius_mm, distance_tolerance, alignment_heading_tolerance, desired_motor_speed, 1);
        r1_integral_w = r1_iw; r1_prev_error_w = r1_pew; r1_goal_reached = r1_gr; r1_alignment_timer = r1_at;

        % -- ROBOT 2 CONTROL
        [r2_left, r2_right, r2_state, r2_iw, r2_pew, r2_gr, r2_at, r2_apf_head] = ... % No need for R2 heading plot
            robot_control_logic(dt, r2_state, r2_pos, r2_angle, robot2_start_pos, robot2_goal_pos, r1_pos, r2_integral_w, r2_prev_error_w, r2_goal_reached, r2_alignment_timer, Kp_w, Ki_w, Kd_w, attraction_factor, repulsion_factor, detection_radius_mm, distance_tolerance, alignment_heading_tolerance, desired_motor_speed, 2);
        r2_integral_w = r2_iw; r2_prev_error_w = r2_pew; r2_goal_reached = r2_gr; r2_alignment_timer = r2_at;

        % -- SYNCHRONIZATION
        if strcmp(r1_state, 'WAITING_FOR_PARTNER') && strcmp(r2_state, 'WAITING_FOR_PARTNER'), disp('Both aligned. Starting APF.'); r1_state = 'APF_CONTROL'; r2_state = 'APF_CONTROL'; end

        % -- SEND COMMANDS
        if ~r1_goal_reached, json_cmd1 = sprintf('{"N":4,"D1":%d,"D2":%d,"H":"apf1"}', r1_left, r1_right); send_robot_command(robot1_ip, json_cmd1); end
        if ~r2_goal_reached, json_cmd2 = sprintf('{"N":4,"D1":%d,"D2":%d,"H":"apf2"}', r2_left, r2_right); send_robot_command(robot2_ip, json_cmd2); end

        % ===============================================================
        % VISUALIZATION UPDATE
        % ===============================================================
        % -- World View
        set(h_robot1, 'XData', r1_pos(1), 'YData', r1_pos(2), 'ZData', r1_pos(3));
        orientation_len = 250;
        r1_angle_rad = deg2rad(r1_angle);
        r1_orient_end = [r1_pos(1) + orientation_len * sin(r1_angle_rad), r1_pos(2), r1_pos(3) + orientation_len * cos(r1_angle_rad)];
        set(h_robot1_orientation, 'XData', [r1_pos(1), r1_orient_end(1)], 'YData', [r1_pos(2), r1_orient_end(2)], 'ZData', [r1_pos(3), r1_orient_end(3)]);
        r1_traj_pts(:, end+1) = r1_pos;
        set(h_trajectory1, 'XData', r1_traj_pts(1,:), 'YData', r1_traj_pts(2,:), 'ZData', r1_traj_pts(3,:));

        set(h_robot2, 'XData', r2_pos(1), 'YData', r2_pos(2), 'ZData', r2_pos(3));
        r2_angle_rad = deg2rad(r2_angle);
        r2_orient_end = [r2_pos(1) + orientation_len * sin(r2_angle_rad), r2_pos(2), r2_pos(3) + orientation_len * cos(r2_angle_rad)];
        set(h_robot2_orientation, 'XData', [r2_pos(1), r2_orient_end(1)], 'YData', [r2_pos(2), r2_orient_end(2)], 'ZData', [r2_pos(3), r2_orient_end(3)]);
        r2_traj_pts(:, end+1) = r2_pos;
        set(h_trajectory2, 'XData', r2_traj_pts(1,:), 'YData', r2_traj_pts(2,:), 'ZData', r2_traj_pts(3,:));
        
        % -- Robot 1 Perspective View
        % angle_rad = -deg2rad(r1_angle); % Negative angle for inverse rotation
        % rot_matrix = [cos(angle_rad), -sin(angle_rad); sin(angle_rad), cos(angle_rad)];
        % obstacle_vec_2d = (r2_pos([1,3]) - r1_pos([1,3]))';
        % obstacle_relative = rot_matrix * obstacle_vec_2d;
        % set(h_vision_obstacle, 'XData', obstacle_relative(1), 'YData', obstacle_relative(2));
        % goal_vec_2d = (robot1_goal_pos([1,3]) - r1_pos([1,3]))';
        % goal_relative = rot_matrix * goal_vec_2d;
        % set(h_vision_goal, 'XData', goal_relative(1), 'YData', goal_relative(2));
        % if ~isempty(r1_traj_pts), traj_vec_2d = r1_traj_pts([1,3],:) - r1_pos([1,3])'; traj_relative = rot_matrix * traj_vec_2d; set(h_vision_traj, 'XData', traj_relative(1,:), 'YData', traj_relative(2,:)); end
        % axis(ax_vision, [-4000 4000 -4000 4000]);

        % -- Robot 1 Heading Plot
        if strcmp(r1_state, 'APF_CONTROL')
            if isempty(r1_apf_start_time), r1_apf_start_time = tic; end
            current_apf_time = toc(r1_apf_start_time);
            r1_heading_time_data(end+1) = current_apf_time;
            r1_heading_angle_data(end+1) = r1_angle;
            r1_apf_heading_data(end+1) = r1_apf_head;
            set(h_current_heading, 'XData', r1_heading_time_data, 'YData', r1_heading_angle_data);
            set(h_apf_heading, 'XData', r1_heading_time_data, 'YData', r1_apf_heading_data);
            set(h_heading_point, 'XData', current_apf_time, 'YData', r1_angle);
        end

        if strcmp(r2_state, 'APF_CONTROL')
            if isempty(r2_apf_start_time), r2_apf_start_time = tic; end
            current_apf_time = toc(r2_apf_start_time);
            r2_heading_time_data(end+1) = current_apf_time;
            r2_heading_angle_data(end+1) = r2_angle;
            r2_apf_heading_data(end+1) = r2_apf_head;
            set(h_current_heading, 'XData', r2_heading_time_data, 'YData', r2_heading_angle_data);
            set(h_apf_heading, 'XData', r2_heading_time_data, 'YData', r2_apf_heading_data);
            set(h_heading_point, 'XData', current_apf_time, 'YData', r2_angle);
        end

        drawnow limitrate;
    end
catch e
    disp('An error occurred. Stopping robots and disconnecting.');
    send_robot_command(robot1_ip, 'S'); send_robot_command(robot2_ip, 'S');
    natnetclient.disconnect;
    rethrow(e);
end

% =======================================================================
% 4. CLEANUP
% =======================================================================
disp('Disconnecting NatNet client...');
send_robot_command(robot1_ip, 'S'); send_robot_command(robot2_ip, 'S');
natnetclient.disconnect;
clear natnetclient;

% =======================================================================
% 5. HELPER FUNCTIONS
% =======================================================================

function [left_speed, right_speed, state, integral_w, prev_error_w, goal_reached, alignment_timer, apf_desired_heading] = ...
    robot_control_logic(dt, state, current_pos_3d, current_angle, start_pos, goal_pos, obstacle_pos_3d, ...
    integral_w, prev_error_w, goal_reached, alignment_timer, ...
    Kp_w, Ki_w, Kd_w, attraction_factor, repulsion_factor, detection_radius, ...
    distance_tolerance, alignment_tolerance, desired_speed, robot_num)
    
    left_speed = 0; right_speed = 0; apf_desired_heading = NaN;
    if goal_reached, state = 'GOAL_REACHED'; end

    dist_to_goal = norm([goal_pos(1) - current_pos_3d(1), goal_pos(3) - current_pos_3d(3)]);
    if dist_to_goal < distance_tolerance && ~strcmp(state, 'INITIAL_APPROACH'), fprintf('Robot %d: Goal reached!\n', robot_num); goal_reached = true; state = 'GOAL_REACHED'; end

    dist_to_start = norm([start_pos(1) - current_pos_3d(1), start_pos(3) - current_pos_3d(3)]);
    if strcmp(state, 'INITIAL_APPROACH') && dist_to_start < distance_tolerance, fprintf('Robot %d: Start position reached. Preparing for alignment...\n', robot_num); state = 'ALIGNMENT_PREP'; alignment_timer = tic; integral_w = 0; prev_error_w = 0; end

    switch state
        case 'INITIAL_APPROACH'
            target_vec = [start_pos(1) - current_pos_3d(1), start_pos(3) - current_pos_3d(3)];
            target_angle = rad2deg(atan2(target_vec(1), target_vec(2)));
            heading_error = normalize_angle(target_angle - current_angle);
            fwd_speed = min(0.3 * dist_to_start, 60);
            turn_speed = 1.0 * heading_error;
            left_speed = fwd_speed + turn_speed;
            right_speed = fwd_speed - turn_speed;
            fprintf('R%d State: %s | Dist to Start: %.1f mm | Fwd: %.1f | Turn: %.1f\n', robot_num, state, dist_to_start, fwd_speed, turn_speed);

        case 'ALIGNMENT_PREP'
            fprintf('R%d State: %s | Pausing...\n', robot_num, state);
            if toc(alignment_timer) >= 1, fprintf('Robot %d: Starting alignment to goal...\n', robot_num); state = 'ALIGNING'; end

        case 'ALIGNING'
            goal_vec = [goal_pos(1) - current_pos_3d(1), goal_pos(3) - current_pos_3d(3)];
            target_angle = rad2deg(atan2(goal_vec(1), goal_vec(2)));
            heading_error = normalize_angle(target_angle - current_angle);
            fprintf('R%d State: %s | Heading Error: %.1f deg\n', robot_num, state, heading_error);
            if abs(heading_error) < alignment_tolerance, fprintf('Robot %d: Alignment complete. Waiting for partner...\n', robot_num); state = 'WAITING_FOR_PARTNER'; integral_w = 0; prev_error_w = 0;
            else, [left_speed, right_speed, integral_w, prev_error_w] = pid_turn(heading_error, dt, Kp_w, Ki_w, Kd_w, integral_w, prev_error_w); end
        
        case 'WAITING_FOR_PARTNER'
            fprintf('R%d State: %s\n', robot_num, state);
            left_speed = 0; right_speed = 0;

        case 'APF_CONTROL'
            fprintf('R%d State: %s | ', robot_num, state);
            current_pos_2d = [current_pos_3d(1); current_pos_3d(3)];
            goal_pos_2d = [goal_pos(1); goal_pos(3)];
            obstacle_pos_2d = [obstacle_pos_3d(1); obstacle_pos_3d(3)];
            obstacle_detected = true; % In dual robot case, the other robot is always the obstacle

            if obstacle_detected
                [~, ~, F_combined] = calculate_apf_forces(current_pos_2d, goal_pos_2d, obstacle_pos_2d, detection_radius, attraction_factor, repulsion_factor);
            else
                goal_vector = goal_pos_2d - current_pos_2d;
                if norm(goal_vector) > 0, F_att = attraction_factor * (goal_vector / norm(goal_vector)); else, F_att = [0; 0]; end
                F_combined = F_att;
            end
            
            if norm(F_combined) > 0.01, apf_desired_heading = rad2deg(atan2(F_combined(1), F_combined(2)));
            else, goal_vec_fb = [goal_pos(1) - current_pos_3d(1), goal_pos(3) - current_pos_3d(3)]; apf_desired_heading = rad2deg(atan2(goal_vec_fb(1), goal_vec_fb(2))); end
            
            heading_error = normalize_angle(apf_desired_heading - current_angle);
            [turn_adjust, integral_w, prev_error_w] = pid_control(heading_error, dt, Kp_w, Ki_w, Kd_w, integral_w, prev_error_w);
            
            left_speed = desired_speed + turn_adjust;
            right_speed = desired_speed - turn_adjust;
            fprintf('APF Head: %.1f | Actual: %.1f\n', apf_desired_heading, current_angle);

        case 'GOAL_REACHED'
            left_speed = 0; right_speed = 0;
    end
    
    left_speed = round(min(max(left_speed, -255), 255));
    right_speed = round(min(max(right_speed, -255), 255));
end

function [F_att, F_rep, F_combined] = calculate_apf_forces(current_pos, goal, obstacle, detection_radius, attraction_factor, repulsion_factor)
    goal_vector = goal - current_pos;
    if norm(goal_vector) > 0, F_att = attraction_factor * (goal_vector / norm(goal_vector)); else, F_att = [0; 0]; end
    
    F_rep = [0; 0];
    if ~isempty(obstacle)
        obstacle_distance = norm(obstacle - current_pos);
        if obstacle_distance <= detection_radius && obstacle_distance > 0
            obstacle_vector = current_pos - obstacle;
            F_rep = (obstacle_vector / norm(obstacle_vector)) * (detection_radius - obstacle_distance) / detection_radius;
            F_rep = repulsion_factor * F_rep;
        end
    end
    
    F_combined = F_att + F_rep;
    if norm(F_combined) > 0, F_combined = F_combined / norm(F_combined); end
end

function [turn_adjust, new_integral, new_prev_error] = pid_control(error, dt, Kp, Ki, Kd, integral, prev_error)
    new_integral = integral + error * dt;
    if dt > 0, derivative = (error - prev_error) / dt; else, derivative = 0; end
    turn_adjust = Kp * error + Ki * new_integral + Kd * derivative;
    new_prev_error = error;
end

function [left, right, new_integral, new_prev_error] = pid_turn(error, dt, Kp, Ki, Kd, integral, prev_error)
    [turn_magnitude, new_integral, new_prev_error] = pid_control(error, dt, Kp, Ki, Kd, integral, prev_error);
    turn_magnitude = min(abs(turn_magnitude), 100);
    if error > 0, left = round(turn_magnitude); right = round(-turn_magnitude);
    else, left = round(-turn_magnitude); right = round(turn_magnitude); end
end

function normalized_angle = normalize_angle(angle)
    normalized_angle = atan2(sin(deg2rad(angle)), cos(deg2rad(angle)));
    normalized_angle = rad2deg(normalized_angle);
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
