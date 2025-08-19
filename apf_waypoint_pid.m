% =======================================================================
% apf_waypoint_pid.m (V4 - Integrated Trajectory Generation)
% 
% This script first generates collision-free paths for two robots offline
% using an Artificial Potential Field (APF) method. Then, it connects to
% the robots and controls them along these pre-computed paths.
% 
% Key Features:
% - Offline APF trajectory generation based on defined start/goal points.
% - Real-time PID control to follow the generated waypoints.
% - Real-time APF for minor corrections and avoiding unexpected obstacles.
% - Confinement to a specific area with "Virtual Wall" forces.
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
% 1. CONFIGURATION - !!! USER MUST EDIT THIS SECTION !!!
% =======================================================================

% -- Robot Configuration
robot_ips = {"192.168.1.140", "192.168.1.84"}; % IP for Robot 1 (ID 1) and Robot 2 (ID 2)
robot_ids = [1, 2];                             % Rigid Body IDs for Robot 1 and 2 in Motive

% -- NatNet Server Configuration
natnet_client_ip = '192.168.1.203';
natnet_server_ip = '192.168.1.209';

% -- Coordinate System & Boundary Definition --
x_offset = 2200; % Amount to add to OptiTrack X to get Simulator Y (mm)
z_offset = 2400; % Amount to add to OptiTrack Z to get Simulator X (mm)
opti_x_min = -1878; opti_x_max = 2467;
opti_z_min = -2732; opti_z_max = 1769;
sim_x_min = opti_z_min + z_offset; % -332
sim_x_max = opti_z_max + z_offset; % 4169
sim_y_min = opti_x_min + x_offset; % 322
sim_y_max = opti_x_max + x_offset; % 4667

% -- OFFLINE Trajectory Generation CONFIG --
start_pos_1 = [500, 1000];
goal_pos_1  = [3500, 3000];
start_pos_2 = [3500, 1000];
goal_pos_2  = [500, 3000];

gen_config.attraction_factor = 0.01;
gen_config.repulsion_factor_robot = 5; % Adjusted for the new, more effective repulsion formula
gen_config.repulsion_factor_wall = 300;
gen_config.detection_radius_robot = 1000;
gen_config.detection_radius_wall = 400;
gen_config.step_size = 50;
gen_config.goal_tolerance = 100;
gen_config.max_steps = 500;
gen_config.sim_bounds = [sim_x_min, sim_x_max, sim_y_min, sim_y_max];

% -- REAL-TIME APF Parameters (for robot control)
rt_config.repulsion_factor_robot = 2.0;
rt_config.repulsion_factor_wall = 2.5;
rt_config.detection_radius_robot = 800;
rt_config.detection_radius_wall = 400;

% -- PID Controller Gains
pid_gains.Kp_h = 1.2;  pid_gains.Ki_h = 0.03; pid_gains.Kd_h = 0.15;
pid_gains.Kp_d = 0.25; pid_gains.Ki_d = 0.01; pid_gains.Kd_d = 0.1;

% -- Control Loop & Physics Parameters
control_params.distance_tolerance = 80;
control_params.alignment_heading_tolerance = 12;
control_params.max_turn_speed = 60;
control_params.max_forward_speed = 80;
control_params.constant_forward_speed = 60;

% =======================================================================
% 2. INITIALIZATION
% =======================================================================

% -- Step 1: Generate Trajectories Offline --
disp('Generating APF trajectories offline...');
[waypoints1, waypoints2] = generate_apf_trajectories(start_pos_1, goal_pos_1, start_pos_2, goal_pos_2, gen_config);

% -- Step 2: Initialize NatNet Connection --
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

% -- Step 3: Initialize 2D Plot for Real-Time Control --
figure;
ax = axes;
hold(ax, 'on'); grid on; axis equal;
title('Dual Robot Bounded APF Control');
xlabel('Simulator X (mm)'); ylabel('Simulator Y (mm)');
view(ax, 2);
boundary_rect = [sim_x_min, sim_y_min, sim_x_max-sim_x_min, sim_y_max-sim_y_min];
rectangle(ax, 'Position', boundary_rect, 'EdgeColor', 'k', 'LineWidth', 2, 'LineStyle', '--');

% -- Step 4: Initialize Robot Structs and Plot Handles --
colors = ['g', 'b'];
robots = struct('ip', [], 'id', [], 'waypoints', [], 'pos', [], 'angle', [], ...
                'state', [], 'current_waypoint_index', [], 'integral_h', [], ...
                'prev_error_h', [], 'integral_d', [], 'prev_error_d', [], ...
                'alignment_pause_timer', [], ...
                'trajectory', [], 'h_robot', [], 'h_orient', [], 'h_traj', [], ...
                'h_waypoints', [], 'h_target', []);

for i = 1:2
    robots(i).ip = robot_ips{i};
    robots(i).id = robot_ids(i);
    if i == 1, robots(i).waypoints = waypoints1; else, robots(i).waypoints = waypoints2; end
    robots(i).pos = [0; 0];
    robots(i).angle = 0;
    robots(i).state = 'INITIAL_APPROACH'; % Start with approaching the first waypoint
    robots(i).current_waypoint_index = 1;
    robots(i).integral_h = 0;
    robots(i).prev_error_h = 0;
    robots(i).integral_d = 0;
    robots(i).prev_error_d = 0;
    robots(i).alignment_pause_timer = tic; % Initialize timer
    robots(i).trajectory = zeros(2, 0);

    % Plot handles
    target_pos = robots(i).waypoints(end, :);
    robots(i).h_target = plot(ax, target_pos(1), target_pos(2), [colors(i) 'p'], 'MarkerSize', 15, 'MarkerFaceColor', colors(i));
    robots(i).h_waypoints = plot(ax, robots(i).waypoints(1:end-1, 1), robots(i).waypoints(1:end-1, 2), [colors(i) 'o'], 'MarkerSize', 8, 'MarkerFaceColor', colors(i));
    robots(i).h_robot = plot(ax, 0, 0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', colors(i));
    robots(i).h_orient = line(ax, [0 0], [0 0], 'Color', 'r', 'LineWidth', 2);
    robots(i).h_traj = plot(ax, NaN, NaN, [colors(i) '-']);
end

last_loop_time = tic;
disp('Initialization complete. Starting control loop...');

% =======================================================================
% 3. MAIN CONTROL LOOP (Pure PID Waypoint Following with Sync Start)
% =======================================================================
try
    while true
        dt = toc(last_loop_time);
        last_loop_time = tic;

        frame = natnetclient.getFrame;
        if isempty(frame.RigidBodies), disp('No rigid bodies in frame...'); pause(0.01); continue; end

        % --- Find and validate data for both robots ---
        rb_data = cell(1, 2);
        found_robots = 0;
        for i = 1:2
            for j = 1:2
                if frame.RigidBodies(i).ID == robots(j).id
                    rb_data{j} = frame.RigidBodies(i);
                    found_robots = found_robots + 1;
                end
            end
        end
        if found_robots < 2, disp('Waiting for both robots to be tracked...'); pause(0.1); continue; end

        % --- Update kinematics for both robots ---
        for i = 1:2
            pos_opti = [rb_data{i}.x, rb_data{i}.y, rb_data{i}.z] * 1000;
            q = quaternion(rb_data{i}.qw, rb_data{i}.qx, rb_data{i}.qy, rb_data{i}.qz);
            angles = q.EulerAngles('ZYX');
            robots(i).pos = [pos_opti(3) + z_offset; pos_opti(1) + x_offset];
            robots(i).angle = wrapToPi(deg2rad(rad2deg(angles(2)) + 230));
        end

        % --- Check for Overall Goal Reached ---
        if strcmp(robots(1).state, 'FINISHED') && strcmp(robots(2).state, 'FINISHED')
            disp('Both robots reached their final goals!');
            send_robot_command(robots(1).ip, 'S'); send_robot_command(robots(2).ip, 'S');
            break;
        end

        % --- SYNCHRONIZATION LOGIC ---
        if strcmp(robots(1).state, 'READY_TO_GO') && strcmp(robots(2).state, 'READY_TO_GO')
            disp('Both robots aligned! Starting synchronized movement.');
            for i = 1:2
                robots(i).state = 'NORMAL_OPERATION';
                robots(i).current_waypoint_index = 2; % Start moving to the 2nd waypoint
                robots(i).integral_h = 0; robots(i).prev_error_h = 0;
                robots(i).integral_d = 0; robots(i).prev_error_d = 0;
            end
        end

        % --- Main logic loop for each robot ---
        for i = 1:2
            robot = robots(i);
            
            if strcmp(robot.state, 'FINISHED'), continue; end

            % --- Error Calculation ---
            current_target_pos = robot.waypoints(robot.current_waypoint_index, :)';
            distance_error = norm(robot.pos - current_target_pos);

            % --- Waypoint Reached Logic ---
            if distance_error < control_params.distance_tolerance && ~strcmp(robot.state, 'ALIGNING') && ~strcmp(robot.state, 'ALIGNMENT_PREP')
                if robot.current_waypoint_index == size(robot.waypoints, 1)
                    disp(['Robot ', num2str(robot.id), ' reached FINAL target!']);
                    robot.state = 'FINISHED';
                    send_robot_command(robot.ip, 'S');
                elseif robot.current_waypoint_index == 1
                    disp(['Robot ', num2str(robot.id), ' reached first waypoint. Preparing for alignment...']);
                    robot.state = 'ALIGNMENT_PREP';
                    robot.alignment_pause_timer = tic;
                    robot.integral_h = 0; robot.prev_error_h = 0;
                    robot.integral_d = 0; robot.prev_error_d = 0;
                else
                    disp(['Robot ', num2str(robot.id), ' reached waypoint ', num2str(robot.current_waypoint_index), '.']);
                    robot.current_waypoint_index = robot.current_waypoint_index + 1;
                end
            end
            
            % --- Pure Heading Calculation (No APF) ---
            target_vec = [0; 0];
            if strcmp(robot.state, 'ALIGNMENT_PREP') || strcmp(robot.state, 'ALIGNING')
                if size(robot.waypoints, 1) >= 2
                    next_target_pos = robot.waypoints(2, :)';
                    target_vec = next_target_pos - robot.pos;
                else 
                    target_vec = robot.waypoints(1, :)' - robot.pos;
                end
            else
                target_vec = current_target_pos - robot.pos;
            end
            
            target_angle = atan2(target_vec(2), target_vec(1));
            heading_error = wrapToPi(target_angle - robot.angle);
            heading_error_deg = rad2deg(heading_error);

            % --- State Machine for Motor Control ---
            forward_speed = 0; turn_speed = 0;
            
            switch robot.state
                case 'INITIAL_APPROACH'
                    robot.integral_d = robot.integral_d + distance_error * dt;
                    derivative_d = (distance_error - robot.prev_error_d) / dt;
                    forward_speed = pid_gains.Kp_d * distance_error + pid_gains.Ki_d * robot.integral_d + pid_gains.Kd_d * derivative_d;
                    robot.prev_error_d = distance_error;
                    
                    robot.integral_h = robot.integral_h + heading_error_deg * dt;
                    derivative_h = (heading_error_deg - robot.prev_error_h) / dt;
                    turn_speed = pid_gains.Kp_h * heading_error_deg + pid_gains.Ki_h * robot.integral_h + pid_gains.Kd_h * derivative_h;
                    robot.prev_error_h = heading_error_deg;

                case 'ALIGNMENT_PREP'
                    forward_speed = 0; turn_speed = 0; % Stop
                    if toc(robot.alignment_pause_timer) >= 1.0 % Wait for 1 sec
                        disp(['Robot ', num2str(robot.id), ' starting alignment...']);
                        robot.state = 'ALIGNING';
                        robot.integral_h = 0; robot.prev_error_h = 0;
                    end
                    
                case 'ALIGNING'
                    forward_speed = 0; % Pure rotation
                    if abs(heading_error_deg) < control_params.alignment_heading_tolerance
                        disp(['Robot ', num2str(robot.id), ' alignment complete. Waiting for other robot...']);
                        robot.state = 'READY_TO_GO';
                    else
                        robot.integral_h = robot.integral_h + heading_error_deg * dt;
                        derivative_h = (heading_error_deg - robot.prev_error_h) / dt;
                        turn_speed = pid_gains.Kp_h * heading_error_deg + pid_gains.Ki_h * robot.integral_h + pid_gains.Kd_h * derivative_h;
                        robot.prev_error_h = heading_error_deg;
                    end

                case 'READY_TO_GO'
                    forward_speed = 0; turn_speed = 0; % Wait

                case 'NORMAL_OPERATION'
                    forward_speed = control_params.constant_forward_speed;
                    robot.integral_h = robot.integral_h + heading_error_deg * dt;
                    derivative_h = (heading_error_deg - robot.prev_error_h) / dt;
                    turn_speed = pid_gains.Kp_h * heading_error_deg + pid_gains.Ki_h * robot.integral_h + pid_gains.Kd_h * derivative_h;
                    robot.prev_error_h = heading_error_deg;
            end

            % --- Motor Command Generation (Strictly matching single-robot script) ---
            json_command = '';
            if strcmp(robot.state, 'ALIGNING')
                % Special hardcoded command for alignment, as in the reference script
                json_command = sprintf('{"N":3,"D1":1,"D2":%d,"H":"pid"}', 75);
            else
                % Standard PID speed calculation for all other moving states
                forward_speed = min(max(forward_speed, 0), control_params.max_forward_speed);
                turn_speed = min(max(turn_speed, -control_params.max_turn_speed), control_params.max_turn_speed);
                
                left_speed = round(forward_speed + turn_speed);
                right_speed = round(forward_speed - turn_speed);

                left_speed  = round(min(max(left_speed, -255), 255));
                right_speed = round(min(max(right_speed, -255), 255));
                
                json_command = sprintf('{"N":4,"D1":%d,"D2":%d,"H":"pid"}', left_speed, right_speed);
            end

            if ~strcmp(robot.state, 'FINISHED') && ~strcmp(robot.state, 'READY_TO_GO')
                 if ~(strcmp(robot.state, 'ALIGNMENT_PREP') && toc(robot.alignment_pause_timer) < 1.0)
                    send_robot_command(robot.ip, json_command);
                 else
                    send_robot_command(robot.ip, 'S'); % Send stop during prep pause
                 end
            end

            robots(i) = robot;

            % --- Update Visualization ---
            set(robots(i).h_robot, 'XData', robot.pos(1), 'YData', robot.pos(2));
            orientation_length = 250;
            orient_end = robot.pos + orientation_length * [cos(robot.angle); sin(robot.angle)];
            set(robots(i).h_orient, 'XData', [robot.pos(1), orient_end(1)], 'YData', [robot.pos(2), orient_end(2)]);
            robots(i).trajectory(:, end+1) = robot.pos;
            set(robots(i).h_traj, 'XData', robots(i).trajectory(1,:), 'YData', robots(i).trajectory(2,:));
        end
        drawnow limitrate;
    end
catch e
    disp('An error occurred. Stopping robots and disconnecting.');
    send_robot_command(robots(1).ip, 'S');
    send_robot_command(robots(2).ip, 'S');
    natnetclient.disconnect;
    rethrow(e);
end

% =======================================================================
% 3. MAIN CONTROL LOOP (with Synchronized Start Logic)
% =======================================================================
try
    while true
        dt = toc(last_loop_time);
        last_loop_time = tic;

        frame = natnetclient.getFrame;
        if isempty(frame.RigidBodies), disp('No rigid bodies in frame...'); pause(0.01); continue; end

        % --- Find and validate data for both robots ---
        rb_data = cell(1, 2);
        found_robots = 0;
        for i = 1:2
            for j = 1:2
                if frame.RigidBodies(i).ID == robots(j).id
                    rb_data{j} = frame.RigidBodies(i);
                    found_robots = found_robots + 1;
                end
            end
        end
        if found_robots < 2, disp('Waiting for both robots to be tracked...'); pause(0.1); continue; end

        % --- Update kinematics for both robots ---
        for i = 1:2
            pos_opti = [rb_data{i}.x, rb_data{i}.y, rb_data{i}.z] * 1000;
            q = quaternion(rb_data{i}.qw, rb_data{i}.qx, rb_data{i}.qy, rb_data{i}.qz);
            angles = q.EulerAngles('ZYX');
            robots(i).pos = [pos_opti(3) + z_offset; pos_opti(1) + x_offset];
            robots(i).angle = wrapToPi(deg2rad(rad2deg(angles(2)) + 230));
        end

        % --- Check for Overall Goal Reached ---
        dist1 = norm(robots(1).pos - robots(1).waypoints(end, :)');
        dist2 = norm(robots(2).pos - robots(2).waypoints(end, :)');
        if dist1 < control_params.distance_tolerance && dist2 < control_params.distance_tolerance
            disp('Both robots reached their final goals!');
            send_robot_command(robots(1).ip, 'S'); send_robot_command(robots(2).ip, 'S');
            break;
        end

        % --- SYNCHRONIZATION LOGIC ---
        if strcmp(robots(1).state, 'READY_TO_GO') && strcmp(robots(2).state, 'READY_TO_GO')
            disp('Both robots aligned! Starting synchronized movement.');
            for i = 1:2
                robots(i).state = 'NORMAL_OPERATION';
                robots(i).integral_h = 0; robots(i).prev_error_h = 0;
                robots(i).integral_d = 0; robots(i).prev_error_d = 0;
            end
        end

        % --- Main logic loop for each robot ---
        for i = 1:2
            robot = robots(i);
            other_robot = robots(3-i);

            if robot.current_waypoint_index > size(robot.waypoints, 1)
                send_robot_command(robot.ip, 'S'); continue;
            end
            
            % --- Error Calculation ---
            current_target_pos = robot.waypoints(robot.current_waypoint_index, :)';
            distance_error = norm(robot.pos - current_target_pos);

            % --- Waypoint Reached Logic ---
            if distance_error < control_params.distance_tolerance && ~strcmp(robot.state, 'ALIGNING')
                if robot.current_waypoint_index == size(robot.waypoints, 1)
                    disp(['Robot ', num2str(robot.id), ' reached FINAL target!']);
                    robot.state = 'FINISHED';
                    robot.current_waypoint_index = robot.current_waypoint_index + 1;
                    send_robot_command(robot.ip, 'S');
                elseif robot.current_waypoint_index == 1 && ~strcmp(robot.state, 'ALIGNMENT_PREP')
                    disp(['Robot ', num2str(robot.id), ' reached first waypoint. Preparing for alignment...']);
                    robot.state = 'ALIGNMENT_PREP';
                    robot.alignment_pause_timer = tic;
                    robot.integral_h = 0; robot.prev_error_h = 0;
                    robot.integral_d = 0; robot.prev_error_d = 0;
                else
                    disp(['Robot ', num2str(robot.id), ' reached waypoint ', num2str(robot.current_waypoint_index), '.']);
                    robot.current_waypoint_index = robot.current_waypoint_index + 1;
                end
            end
            
            % --- Heading Calculation ---
            target_vec = [0; 0];
            if strcmp(robot.state, 'ALIGNMENT_PREP') || strcmp(robot.state, 'ALIGNING')
                % Look at the next waypoint to align
                if size(robot.waypoints, 1) >= 2
                    next_target_pos = robot.waypoints(2, :)';
                    target_vec = next_target_pos - robot.pos;
                else % Edge case: path has only one point
                    target_vec = robot.waypoints(1, :)' - robot.pos;
                end
            else
                % Default: look at the current target
                target_vec = current_target_pos - robot.pos;
            end

            % --- Real-time APF Application (only in normal operation) ---
            F_total = target_vec;
            if strcmp(robot.state, 'NORMAL_OPERATION')
                F_att = target_vec;
                % Repulsive force from other robot
                F_rep_robot = [0; 0];
                obstacle_vector = robot.pos - other_robot.pos;
                dist_to_obstacle = norm(obstacle_vector);
                if dist_to_obstacle <= rt_config.detection_radius_robot && dist_to_obstacle > 1e-6
                    F_rep_magnitude = rt_config.repulsion_factor_robot * (rt_config.detection_radius_robot - dist_to_obstacle) / dist_to_obstacle;
                    F_rep_robot = (obstacle_vector / dist_to_obstacle) * F_rep_magnitude;
                end
                % Repulsive force from walls
                F_rep_wall = [0; 0];
                if robot.pos(1) < sim_x_min + rt_config.detection_radius_wall, F_rep_wall(1) = F_rep_wall(1) + rt_config.repulsion_factor_wall * (1 - (robot.pos(1) - sim_x_min)/rt_config.detection_radius_wall); end
                if robot.pos(1) > sim_x_max - rt_config.detection_radius_wall, F_rep_wall(1) = F_rep_wall(1) - rt_config.repulsion_factor_wall * (1 - (sim_x_max - robot.pos(1))/rt_config.detection_radius_wall); end
                if robot.pos(2) < sim_y_min + rt_config.detection_radius_wall, F_rep_wall(2) = F_rep_wall(2) + rt_config.repulsion_factor_wall * (1 - (robot.pos(2) - sim_y_min)/rt_config.detection_radius_wall); end
                if robot.pos(2) > sim_y_max - rt_config.detection_radius_wall, F_rep_wall(2) = F_rep_wall(2) - rt_config.repulsion_factor_wall * (1 - (sim_y_max - robot.pos(2))/rt_config.detection_radius_wall); end
                
                F_total = F_att + F_rep_robot + F_rep_wall;
            end
            
            target_angle = atan2(F_total(2), F_total(1));
            heading_error = wrapToPi(target_angle - robot.angle);
            heading_error_deg = rad2deg(heading_error);

            % --- State Machine for Motor Control ---
            forward_speed = 0; turn_speed = 0;
            
            switch robot.state
                case 'INITIAL_APPROACH'
                    robot.integral_d = robot.integral_d + distance_error * dt;
                    derivative_d = (distance_error - robot.prev_error_d) / dt;
                    forward_speed = pid_gains.Kp_d * distance_error + pid_gains.Ki_d * robot.integral_d + pid_gains.Kd_d * derivative_d;
                    robot.prev_error_d = distance_error;
                    
                    robot.integral_h = robot.integral_h + heading_error_deg * dt;
                    derivative_h = (heading_error_deg - robot.prev_error_h) / dt;
                    turn_speed = pid_gains.Kp_h * heading_error_deg + pid_gains.Ki_h * robot.integral_h + pid_gains.Kd_h * derivative_h;
                    robot.prev_error_h = heading_error_deg;

                case 'ALIGNMENT_PREP'
                    forward_speed = 0; turn_speed = 0; % Stop
                    if toc(robot.alignment_pause_timer) >= 1.0 % Wait for 1 sec
                        disp(['Robot ', num2str(robot.id), ' starting alignment...']);
                        robot.state = 'ALIGNING';
                        robot.integral_h = 0; robot.prev_error_h = 0;
                    end
                    
                case 'ALIGNING'
                    forward_speed = 0; % Pure rotation
                    if abs(heading_error_deg) < control_params.alignment_heading_tolerance
                        disp(['Robot ', num2str(robot.id), ' alignment complete. Waiting for other robot...']);
                        robot.state = 'READY_TO_GO';
                    else
                        robot.integral_h = robot.integral_h + heading_error_deg * dt;
                        derivative_h = (heading_error_deg - robot.prev_error_h) / dt;
                        turn_speed = pid_gains.Kp_h * heading_error_deg + pid_gains.Ki_h * robot.integral_h + pid_gains.Kd_h * derivative_h;
                        robot.prev_error_h = heading_error_deg;
                    end

                case 'READY_TO_GO'
                    forward_speed = 0; turn_speed = 0; % Wait

                case 'NORMAL_OPERATION'
                    forward_speed = control_params.constant_forward_speed;
                    robot.integral_h = robot.integral_h + heading_error_deg * dt;
                    derivative_h = (heading_error_deg - robot.prev_error_h) / dt;
                    turn_speed = pid_gains.Kp_h * heading_error_deg + pid_gains.Ki_h * robot.integral_h + pid_gains.Kd_h * derivative_h;
                    robot.prev_error_h = heading_error_deg;
            end

            % --- Motor Command Generation ---
            forward_speed = min(max(forward_speed, 0), control_params.max_forward_speed);
            turn_speed = min(max(turn_speed, -control_params.max_turn_speed), control_params.max_turn_speed);
            
            if strcmp(robot.state, 'ALIGNING')
                left_speed = round(turn_speed);
                right_speed = round(-turn_speed); % In-place rotation
            else
                left_speed = round(forward_speed - turn_speed); % Adjusted for standard differential drive
                right_speed = round(forward_speed + turn_speed);
            end

            left_speed  = round(min(max(left_speed, -255), 255));
            right_speed = round(min(max(right_speed, -255), 255));

            if ~strcmp(robot.state, 'FINISHED')
                json_command = sprintf('{"N":4,"D1":%d,"D2":%d,"H":"pid"}', left_speed, right_speed);
                send_robot_command(robot.ip, json_command);
            end

            robots(i) = robot;

            % --- Update Visualization ---
            set(robots(i).h_robot, 'XData', robot.pos(1), 'YData', robot.pos(2));
            orientation_length = 250;
            orient_end = robot.pos + orientation_length * [cos(robot.angle); sin(robot.angle)];
            set(robots(i).h_orient, 'XData', [robot.pos(1), orient_end(1)], 'YData', [robot.pos(2), orient_end(2)]);
            robots(i).trajectory(:, end+1) = robot.pos;
            set(robots(i).h_traj, 'XData', robots(i).trajectory(1,:), 'YData', robots(i).trajectory(2,:));
        end
        drawnow limitrate;
    end
catch e
    disp('An error occurred. Stopping robots and disconnecting.');
    send_robot_command(robots(1).ip, 'S');
    send_robot_command(robots(2).ip, 'S');
    natnetclient.disconnect;
    rethrow(e);
end

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

        % --- Find and validate data for both robots ---
        rb_data = cell(1, 2);
        found_robots = 0;
        for i = 1:2
            if frame.RigidBodies(i).ID == robots(1).id
                rb_data{1} = frame.RigidBodies(i);
                found_robots = found_robots + 1;
            elseif frame.RigidBodies(i).ID == robots(2).id
                rb_data{2} = frame.RigidBodies(i);
                found_robots = found_robots + 1;
            end
        end

        if found_robots < 2
            disp('Waiting for both robots to be tracked...');
            pause(0.1);
            continue;
        end

        % --- Update kinematics for both robots ---
        for i = 1:2
            pos_opti = [rb_data{i}.x, rb_data{i}.y, rb_data{i}.z] * 1000;
            q = quaternion(rb_data{i}.qw, rb_data{i}.qx, rb_data{i}.qy, rb_data{i}.qz);
            angles = q.EulerAngles('ZYX');
            robots(i).pos = [pos_opti(3) + z_offset; pos_opti(1) + x_offset];
            robots(i).angle = wrapToPi(deg2rad(rad2deg(angles(2)) + 230));
        end

        % --- Check for Goal Reached for BOTH robots ---
        dist1 = norm(robots(1).pos - robots(1).waypoints(end, :)');
        dist2 = norm(robots(2).pos - robots(2).waypoints(end, :)');
        if dist1 < control_params.distance_tolerance && dist2 < control_params.distance_tolerance
            disp('Both robots reached their final goals!');
            send_robot_command(robots(1).ip, 'S');
            send_robot_command(robots(2).ip, 'S');
            break;
        end

        % --- Main logic loop for each robot ---
        for i = 1:2
            robot = robots(i);
            other_robot = robots(3-i);

            if robot.current_waypoint_index > size(robot.waypoints, 1)
                send_robot_command(robot.ip, 'S');
                continue;
            end
            current_target_pos = robot.waypoints(robot.current_waypoint_index, :)';
            distance_error = norm(robot.pos - current_target_pos);

            if distance_error < control_params.distance_tolerance
                if robot.current_waypoint_index == size(robot.waypoints, 1)
                    disp(['Robot ', num2str(robot.id), ' reached FINAL target!']);
                    robot.current_waypoint_index = robot.current_waypoint_index + 1;
                    send_robot_command(robot.ip, 'S');
                    robots(i) = robot;
                    continue;
                elseif robot.current_waypoint_index == 1 && ~robot.first_waypoint_reached
                    disp(['Robot ', num2str(robot.id), ' reached first waypoint. Preparing for alignment...']);
                    robot.first_waypoint_reached = true;
                    robot.state = 'ALIGNMENT_PREP';
                    robot.alignment_pause_timer = tic;
                    robot.integral_h = 0; robot.prev_error_h = 0;
                    robot.integral_d = 0; robot.prev_error_d = 0;
                else
                    disp(['Robot ', num2str(robot.id), ' reached waypoint ', num2str(robot.current_waypoint_index), '.']);
                    robot.current_waypoint_index = robot.current_waypoint_index + 1;
                    if strcmp(robot.state, 'ALIGNING')
                        robot.state = 'NORMAL_OPERATION';
                    end
                end
            end
            
            F_att = current_target_pos - robot.pos;
            F_rep_robot = [0; 0];
            obstacle_vector = robot.pos - other_robot.pos;
            dist_to_obstacle = norm(obstacle_vector);
            if dist_to_obstacle <= rt_config.detection_radius_robot && dist_to_obstacle > 1e-6
                F_rep_robot = (obstacle_vector / dist_to_obstacle) * ...
                       (rt_config.detection_radius_robot - dist_to_obstacle) / dist_to_obstacle;
                F_rep_robot = rt_config.repulsion_factor_robot * F_rep_robot;
            end

            F_rep_wall = [0; 0];
            if robot.pos(1) < sim_x_min + rt_config.detection_radius_wall, F_rep_wall(1) = F_rep_wall(1) + rt_config.repulsion_factor_wall * (1 - (robot.pos(1) - sim_x_min)/rt_config.detection_radius_wall); end
            if robot.pos(1) > sim_x_max - rt_config.detection_radius_wall, F_rep_wall(1) = F_rep_wall(1) - rt_config.repulsion_factor_wall * (1 - (sim_x_max - robot.pos(1))/rt_config.detection_radius_wall); end
            if robot.pos(2) < sim_y_min + rt_config.detection_radius_wall, F_rep_wall(2) = F_rep_wall(2) + rt_config.repulsion_factor_wall * (1 - (robot.pos(2) - sim_y_min)/rt_config.detection_radius_wall); end
            if robot.pos(2) > sim_y_max - rt_config.detection_radius_wall, F_rep_wall(2) = F_rep_wall(2) - rt_config.repulsion_factor_wall * (1 - (sim_y_max - robot.pos(2))/rt_config.detection_radius_wall); end

            F_combined = F_att + F_rep_robot + F_rep_wall;
            
            if strcmp(robot.state, 'ALIGNMENT_PREP') || strcmp(robot.state, 'ALIGNING')
                if robot.current_waypoint_index + 1 <= size(robot.waypoints, 1)
                    next_target_pos = robot.waypoints(robot.current_waypoint_index + 1, :)';
                    F_att_align = next_target_pos - robot.pos;
                    F_combined = F_att_align + F_rep_robot + F_rep_wall;
                end
            end
            target_angle = atan2(F_combined(2), F_combined(1));
            heading_error = wrapToPi(target_angle - robot.angle);
            heading_error_deg = rad2deg(heading_error);

            forward_speed = 0; turn_speed = 0;
            
            switch robot.state
                case 'INITIAL_APPROACH'
                    robot.integral_d = robot.integral_d + distance_error * dt;
                    derivative_d = (distance_error - robot.prev_error_d) / dt;
                    forward_speed = pid_gains.Kp_d * distance_error + pid_gains.Ki_d * robot.integral_d + pid_gains.Kd_d * derivative_d;
                    robot.prev_error_d = distance_error;
                    forward_speed = min(max(forward_speed, 0), control_params.max_forward_speed);
                    
                    robot.integral_h = robot.integral_h + heading_error_deg * dt;
                    derivative_h = (heading_error_deg - robot.prev_error_h) / dt;
                    turn_speed = pid_gains.Kp_h * heading_error_deg + pid_gains.Ki_h * robot.integral_h + pid_gains.Kd_h * derivative_h;
                    robot.prev_error_h = heading_error_deg;
                    
                case 'ALIGNMENT_PREP'
                    if toc(robot.alignment_pause_timer) >= 1
                        robot.state = 'ALIGNING';
                        robot.current_waypoint_index = robot.current_waypoint_index + 1;
                        robot.integral_h = 0; robot.prev_error_h = 0;
                    end
                    
                case 'ALIGNING'
                    if abs(heading_error_deg) < control_params.alignment_heading_tolerance
                        robot.state = 'NORMAL_OPERATION';
                        robot.integral_h = 0; robot.prev_error_h = 0;
                    else
                        robot.integral_h = robot.integral_h + heading_error_deg * dt;
                        derivative_h = (heading_error_deg - robot.prev_error_h) / dt;
                        turn_speed = pid_gains.Kp_h * heading_error_deg + pid_gains.Ki_h * robot.integral_h + pid_gains.Kd_h * derivative_h;
                        robot.prev_error_h = heading_error_deg;
                    end
                    
                case 'NORMAL_OPERATION'
                    forward_speed = control_params.constant_forward_speed;
                    robot.integral_h = robot.integral_h + heading_error_deg * dt;
                    derivative_h = (heading_error_deg - robot.prev_error_h) / dt;
                    turn_speed = pid_gains.Kp_h * heading_error_deg + pid_gains.Ki_h * robot.integral_h + pid_gains.Kd_h * derivative_h;
                    robot.prev_error_h = heading_error_deg;
            end

            turn_speed = min(max(turn_speed, -control_params.max_turn_speed), control_params.max_turn_speed);
            
            if strcmp(robot.state, 'ALIGNING')
                left_speed = round(turn_speed);
                right_speed = round(-turn_speed);
            else
                left_speed = round(forward_speed + turn_speed);
                right_speed = round(forward_speed - turn_speed);
            end

            left_speed  = round(min(max(left_speed, -255), 255));
            right_speed = round(min(max(right_speed, -255), 255));

            json_command = sprintf('{"N":4,"D1":%d,"D2":%d,"H":"pid"}', left_speed, right_speed);
            send_robot_command(robot.ip, json_command);

            robots(i) = robot;

            set(robots(i).h_robot, 'XData', robot.pos(1), 'YData', robot.pos(2));
            orientation_length = 250;
            orient_end = robot.pos + orientation_length * [cos(robot.angle); sin(robot.angle)];
            set(robots(i).h_orient, 'XData', [robot.pos(1), orient_end(1)], 'YData', [robot.pos(2), orient_end(2)]);
            robots(i).trajectory(:, end+1) = robot.pos;
            set(robots(i).h_traj, 'XData', robots(i).trajectory(1,:), 'YData', robots(i).trajectory(2,:));
        end
        drawnow limitrate;
    end
catch e
    disp('An error occurred. Stopping robots and disconnecting.');
    send_robot_command(robots(1).ip, 'S');
    send_robot_command(robots(2).ip, 'S');
    natnetclient.disconnect;
    rethrow(e);
end

% =======================================================================
% 4. CLEANUP
% =======================================================================
disp('Disconnecting NatNet client...');
send_robot_command(robots(1).ip, 'S');
send_robot_command(robots(2).ip, 'S');
natnetclient.disconnect;
clear natnetclient;
disp('Script finished.');

% =======================================================================
% 5. HELPER FUNCTIONS
% =======================================================================

function response = send_robot_command(ip, command)
    try
        t = tcpclient(ip, 80, 'ConnectTimeout', 1, 'Timeout', 1);
        write(t, [char(command), newline]);
        clear t;
        response = 'OK';
    catch e
        fprintf('WARNING: Failed to send command to robot at %s. Error: %s', char(ip), e.message);
        response = 'Error';
    end
end

function q = quaternion(w, x, y, z)
    q.w = w; q.x = x; q.y = y; q.z = z;
    q.EulerAngles = @(order) quat2eul([q.w, q.x, q.y, q.z], order);
end

function angle = wrapToPi(angle)
    angle = atan2(sin(angle), cos(angle));
end

function [trajectory1, trajectory2] = generate_apf_trajectories(start1, goal1, start2, goal2, config)
    disp('Generating trajectories...');
    trajectory1 = start1;
    trajectory2 = start2;
    current_pos_1 = start1;
    current_pos_2 = start2;
    goal1_reached = false;
    goal2_reached = false;

    for step = 1:config.max_steps
        if goal1_reached && goal2_reached, break; end

        % Robot 1 Calculation
        if ~goal1_reached
            F_att_1 = (goal1 - current_pos_1) * config.attraction_factor;
            
            robot_vector_1 = current_pos_1 - current_pos_2;
            dist_1 = norm(robot_vector_1);
            F_rep_robot_1 = [0, 0];
            if dist_1 < config.detection_radius_robot && dist_1 > 1e-6
                % Using a more effective repulsion formula
                force_magnitude = config.repulsion_factor_robot * (config.detection_radius_robot - dist_1) / dist_1;
                F_rep_robot_1 = (robot_vector_1 / dist_1) * force_magnitude;
            end

            F_rep_wall_1 = calculate_wall_repulsion(current_pos_1, config.sim_bounds, config.detection_radius_wall, config.repulsion_factor_wall);
            F_total_1 = F_att_1 + F_rep_robot_1 + F_rep_wall_1;
            current_pos_1 = current_pos_1 + (F_total_1 / norm(F_total_1)) * config.step_size;
            trajectory1 = [trajectory1; current_pos_1];
            if norm(current_pos_1 - goal1) < config.goal_tolerance, goal1_reached = true; disp('Robot 1 trajectory complete.'); end
        end

        % Robot 2 Calculation
        if ~goal2_reached
            F_att_2 = (goal2 - current_pos_2) * config.attraction_factor;

            robot_vector_2 = current_pos_2 - current_pos_1;
            dist_2 = norm(robot_vector_2);
            F_rep_robot_2 = [0, 0];
            if dist_2 < config.detection_radius_robot && dist_2 > 1e-6
                % Using a more effective repulsion formula
                force_magnitude = config.repulsion_factor_robot * (config.detection_radius_robot - dist_2) / dist_2;
                F_rep_robot_2 = (robot_vector_2 / dist_2) * force_magnitude;
            end

            F_rep_wall_2 = calculate_wall_repulsion(current_pos_2, config.sim_bounds, config.detection_radius_wall, config.repulsion_factor_wall);
            F_total_2 = F_att_2 + F_rep_robot_2 + F_rep_wall_2;
            current_pos_2 = current_pos_2 + (F_total_2 / norm(F_total_2)) * config.step_size;
            trajectory2 = [trajectory2; current_pos_2];
            if norm(current_pos_2 - goal2) < config.goal_tolerance, goal2_reached = true; disp('Robot 2 trajectory complete.'); end
        end
    end
    if step == config.max_steps, disp('Max steps reached in generation.'); end
    
    % Plot generated trajectories for verification
    figure; hold on; grid on; axis equal;
    title('Generated APF Trajectories');
    xlabel('Simulator X (mm)'); ylabel('Simulator Y (mm)');
    rectangle('Position', [config.sim_bounds(1), config.sim_bounds(3), config.sim_bounds(2)-config.sim_bounds(1), config.sim_bounds(4)-config.sim_bounds(3)], 'EdgeColor', 'k', 'LineStyle', '--');
    plot(trajectory1(:,1), trajectory1(:,2), 'g-', 'LineWidth', 2);
    plot(trajectory2(:,1), trajectory2(:,2), 'b-', 'LineWidth', 2);
    plot(start1(1), start1(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot(goal1(1), goal1(2), 'gp', 'MarkerSize', 15, 'MarkerFaceColor', 'g');
    plot(start2(1), start2(2), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
    plot(goal2(1), goal2(2), 'bp', 'MarkerSize', 15, 'MarkerFaceColor', 'b');
    legend('Robot 1 Trajectory', 'Robot 2 Trajectory', 'R1 Start', 'R1 Goal', 'R2 Start', 'R2 Goal');
    hold off;
    disp('Trajectory generation plot displayed. Press any key to continue to robot control...');
    pause;
end

function F_rep_wall = calculate_wall_repulsion(pos, bounds, radius, factor)
    F_rep_wall = [0, 0];
    x_min = bounds(1); x_max = bounds(2); y_min = bounds(3); y_max = bounds(4);
    % Towards right from left wall
    dist_left = pos(1) - x_min;
    if dist_left < radius, F_rep_wall(1) = F_rep_wall(1) + factor * (1/dist_left - 1/radius) / dist_left^2;
    end
    % Towards left from right wall
    dist_right = x_max - pos(1);
    if dist_right < radius, F_rep_wall(1) = F_rep_wall(1) - factor * (1/dist_right - 1/radius) / dist_right^2;
    end
    % Towards up from bottom wall
    dist_bottom = pos(2) - y_min;
    if dist_bottom < radius, F_rep_wall(2) = F_rep_wall(2) + factor * (1/dist_bottom - 1/radius) / dist_bottom^2;
    end
    % Towards down from top wall
    dist_top = y_max - pos(2);
    if dist_top < radius, F_rep_wall(2) = F_rep_wall(2) - factor * (1/dist_top - 1/radius) / dist_top^2;
    end
end
