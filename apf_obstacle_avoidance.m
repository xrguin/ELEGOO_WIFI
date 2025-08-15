%% Artificial Potential Field (APF) Obstacle Avoidance with Constant Velocity
% This script demonstrates APF-based navigation where robots avoid each other
% while moving towards their goals using Artificial Potential Fields
% Modified to maintain CONSTANT LINEAR VELOCITY throughout the simulation
% Multi-trial version for data collection based on simulation_APF.m
init

%% Test Settings (adapted from simulation_APF.m)
animation = false;        % Set true to show real-time animation
simulation = true;       % Set true to run simulation
record_data = true;     % Set true to save data to .mat file

%% Turning Detection Parameters
angular_velocity_threshold = 0.2;  % Angular velocity threshold to start/stop recording (rad/s)
n = 0;                             % Trial counter
samples = 7000;                       % Number of trials to run
apf_constant_vel_data = cell(samples, 2); % Data storage: {trial, robot} format

%% TUNABLE PARAMETERS - ADJUST THESE TO CHANGE ROBOT BEHAVIOR

% === APF Parameters (from simulation_APF.m) ===
attraction_factor_robot1 = 1;    % Robot 1 (green) attraction factor
repulsion_factor_robot1 = 1.7;     % Robot 1 (green) repulsion factor
attraction_factor_robot2 = 1;    % Robot 2 (blue) attraction factor  
repulsion_factor_robot2 = 1.5;   % Robot 2 (blue) repulsion factor (higher as in original)
detection_radius = 1;          % Detection range for APF
safe_radius = 0.3;               % Safety radius around robots
constant_linear_velocity = 0.1;  % CONSTANT linear velocity (m/s) - NEVER CHANGES

% === Robotarium Constants ===
N = 2; % Number of robots
max_linear_velocity = 0.08;       % Maximum linear velocity
max_angular_velocity = 3.5;      % Maximum angular velocity (rad/s)
robot_diameter = 0.11;
collision_diameter = 0.135;
boundaries = [-1.6, 1.6, -1, 1]; % [x_min, x_max, y_min, y_max]

% === Simulation Parameters ===
sampleTime = 0.033;              % Robotarium time step
simulationTime = 30;             % Maximum simulation time
iterations = ceil(simulationTime / sampleTime);
goal_radius = 0.1;               % Distance to consider goal reached

% === Arena Bounds ===
boundary_buffer = 0.3;           % Keep robots away from walls
x_bound = boundaries(2) - boundary_buffer;
y_bound = boundaries(4) - boundary_buffer;

% Safe generation bounds
x_bound_gen = x_bound - 0.2;
y_bound_gen = y_bound - 0.2;

% === Path Generation Parameters ===
min_path_length = 1;             % Minimum distance between start and goal
max_path_length = 2;             % Maximum distance between start and goal
min_robot_start_separation = 1.5; % Minimum distance between robot starts
max_generation_attempts = 20;     % Max attempts to generate good paths

% === Visualization Parameters (simplified like simulation_APF.m) ===
show_trajectories = true;        % Show real-time trajectory trails
show_start_goal = true;          % Show start and goal positions

%% MULTI-TRIAL SIMULATION LOOP
if simulation
    fprintf('Starting APF constant velocity simulation with %d trials\n', samples);
    rng('shuffle'); % Ensure randomness across trials
    
    while n < samples
        %% PATH GENERATION FOR THIS TRIAL
        % Robot 1 (green) - random start and goal with minimum path length
        start1 = [(rand()*2-1)*x_bound_gen; (rand()*2-1)*y_bound_gen];
        
        % Generate goal ensuring minimum distance
        path_length = min_path_length + rand() * (max_path_length - min_path_length);
        angle1 = rand() * 2 * pi;
        goal1 = start1 + path_length * [cos(angle1); sin(angle1)];
        goal1(1) = max(-x_bound_gen, min(x_bound_gen, goal1(1)));
        goal1(2) = max(-y_bound_gen, min(y_bound_gen, goal1(2)));
        
        % start1 = [1.0831;-0.2311];
        % goal1 = [-0.5552; -0.0281];
        % Regenerate if path is too short after bounding
        max_attempts = 20;
        attempt = 0;
        while norm(goal1 - start1) < min_path_length && attempt < max_attempts
            angle1 = rand() * 2 * pi;
            goal1 = start1 + path_length * [cos(angle1); sin(angle1)];
            goal1(1) = max(-x_bound_gen, min(x_bound_gen, goal1(1)));
            goal1(2) = max(-y_bound_gen, min(y_bound_gen, goal1(2)));
            attempt = attempt + 1;
        end
        
        % Robot 2 (blue) - intercepting path
        [start2, goal2, interceptPoint] = generateInterceptingPathSimple(start1, goal1, ...
            x_bound_gen, y_bound_gen, min_path_length, safe_radius*2);
        
        % start2 = [-0.52891;-0.0864];
        % goal2 = [1.0880; -0.1766];
        % Ensure robots don't start too close together
        attempts = 0;
        while norm(start2 - start1) < min_robot_start_separation && attempts < max_generation_attempts
            [start2, goal2, interceptPoint] = generateInterceptingPathSimple(start1, goal1, ...
                x_bound_gen, y_bound_gen, min_path_length, safe_radius*2);
            attempts = attempts + 1;
        end
        
        if attempts >= max_generation_attempts && n == 0
            fprintf('Warning: Could not find well-separated initial positions\n');
        end
        
        % Calculate initial headings (facing towards goals)
        initial_heading1 = atan2(goal1(2) - start1(2), goal1(1) - start1(1));
        initial_heading2 = atan2(goal2(2) - start2(2), goal2(1) - start2(1));
        
        % Set initial conditions with proper headings
        initial_conditions = [start1, start2; initial_heading1, initial_heading2];
        
        % Create new Robotarium instance for this trial
        if n == 0
            % First trial - create with animation if enabled
            r = Robotarium('NumberOfRobots', N, 'ShowFigure', animation, 'InitialConditions', initial_conditions);
        else
            % Subsequent trials - close previous and create new
            if exist('r', 'var')
                r.debug(); % Clean up previous instance
            end
            % Keep animation setting for all trials if enabled
            r = Robotarium('NumberOfRobots', N, 'ShowFigure', animation, 'InitialConditions', initial_conditions);
        end

        %% TRIAL INITIALIZATION
        % === Data Storage for this trial ===
        pose_history = zeros(4, N, iterations); % [x; y; theta; omega] to match MbRobot
        velocity_history = zeros(2, N, iterations); % [v; omega]
        force_history = struct(); % Store APF forces
        for i = 1:N
            force_history(i).attraction = zeros(2, iterations);
            force_history(i).repulsion = zeros(2, iterations);
            force_history(i).combined = zeros(2, iterations);
        end
        pose_idx = 1;
        
        % === Turning Detection Storage ===
        poses_robot1_apf = [];  % Only record when turning
        poses_robot2_apf = [];  % Only record when turning
        recording = false;      % Start recording when significant turning detected
        
        % Store previous angles for angular velocity calculation
        prev_angle1 = initial_heading1;
        prev_angle2 = initial_heading2;
        
        % === MbRobot-style state tracking ===
        angular_velocity1 = 0;
        angular_velocity2 = 0;
        
        % === Performance Metrics ===
        path_start_time = zeros(N, 1);
        path_end_time = zeros(N, 1);
        straight_line_distance = [norm(goal1 - start1); norm(goal2 - start2)];
        total_distance_traveled = zeros(N, 1);
        last_positions = [start1, start2];

        %% MAIN SIMULATION LOOP FOR THIS TRIAL
        for t = 1:iterations
            % Get current poses
            x = r.get_poses();
            current_time = t * sampleTime;
            
            % Calculate angular velocities BEFORE storing pose
            if current_time >= 0.5  % Start calculating after 0.5 seconds
                current_angle1 = x(3, 1);
                current_angle2 = x(3, 2);
                
                % Calculate angular velocities (MbRobot style)
                angular_change1 = wrapToPi(current_angle1 - prev_angle1);
                angular_change2 = wrapToPi(current_angle2 - prev_angle2);
                angular_velocity1 = angular_change1 / sampleTime;
                angular_velocity2 = angular_change2 / sampleTime;
                
                % Update previous angles for next iteration
                prev_angle1 = current_angle1;
                prev_angle2 = current_angle2;
            end
            
            % Store pose history with angular velocity (MbRobot style)
            pose_history(1:3, :, pose_idx) = x;
            pose_history(4, 1, pose_idx) = angular_velocity1;
            pose_history(4, 2, pose_idx) = angular_velocity2;
            
            % Turning detection using angular velocity thresholds
            if current_time >= 0.5  % Start checking after 0.5 seconds
                % Check if either robot is turning significantly
                is_turning = abs(angular_velocity1) > angular_velocity_threshold || ...
                            abs(angular_velocity2) > angular_velocity_threshold;
                
                % Start recording when turning begins
                if ~recording && is_turning
                    recording = true;
                    fprintf('Trial %d: Turning detection started at t=%.2fs (ω1=%.2f, ω2=%.2f)\n', ...
                            n+1, current_time, angular_velocity1, angular_velocity2);
                end
                
                % Stop recording when turning ends
                if recording && ~is_turning
                    recording = false;
                    fprintf('Trial %d: Turning detection stopped at t=%.2fs (ω1=%.2f, ω2=%.2f)\n', ...
                            n+1, current_time, angular_velocity1, angular_velocity2);
                end
                
                % Record poses only when turning is happening
                if recording
                    poses_robot1_apf = [poses_robot1_apf, [x(:, 1); angular_velocity1]];
                    poses_robot2_apf = [poses_robot2_apf, [x(:, 2); angular_velocity2]];
                end
            end
            
            % Track distance traveled
            for i = 1:N
                movement = norm(x(1:2, i) - last_positions(:, i));
                if movement > 0.001
                    total_distance_traveled(i) = total_distance_traveled(i) + movement;
                end
                last_positions(:, i) = x(1:2, i);
            end
            
            % === Apply APF Algorithm for Each Robot ===
            dxu = zeros(2, N); % [v; omega] for each robot
            
            for i = 1:N
                % Current state
                current_pos = x(1:2, i);
                current_heading = x(3, i);
                
                % Note: Angular velocity already calculated above before control loop
                
                % Goal and parameters for this robot
                if i == 1
                    goal = goal1;
                    attraction_factor = attraction_factor_robot1;
                    repulsion_factor = repulsion_factor_robot1;
                else
                    goal = goal2;
                    attraction_factor = attraction_factor_robot2;
                    repulsion_factor = repulsion_factor_robot2;
                end
                
                % Find other robots' positions (obstacles)
                obstacle_positions = x(1:2, setdiff(1:N, i));
                
                % Check if at goal
                distance_to_goal = norm(goal - current_pos);
                
                if distance_to_goal > goal_radius
                    % Calculate APF forces using MbRobot algorithm
                    [F_att, F_rep, F_combined] = calculate_apf_forces_mbstyle(current_pos, goal, ...
                        obstacle_positions, detection_radius, attraction_factor, repulsion_factor);
                    
                    % Store forces for analysis
                    force_history(i).attraction(:, pose_idx) = F_att;
                    force_history(i).repulsion(:, pose_idx) = F_rep;
                    force_history(i).combined(:, pose_idx) = F_combined;
                    
                    % === CONSTANT VELOCITY CONTROL (MbRobot style) ===
                    % Normalize combined force to get direction only
                    if norm(F_combined) > 0.01
                        % Desired heading from combined force
                        desired_heading = atan2(F_combined(2), F_combined(1));
                    else
                        % If no significant force, move toward goal
                        goal_direction = goal - current_pos;
                        desired_heading = atan2(goal_direction(2), goal_direction(1));
                    end
                    
                    % Compute angular velocity based on heading error
                    heading_error = wrapToPi(desired_heading - current_heading);
                    omega = 2.0 * heading_error; % P control for heading
                    
                    % Apply angular velocity limit
                    omega = max(-max_angular_velocity, min(max_angular_velocity, omega));
                    
                    % === MAINTAIN CONSTANT LINEAR VELOCITY ===
                    v = constant_linear_velocity;
                else
                    % At goal - stop
                    v = 0;
                    omega = 0;
                end
                
                % Set control inputs
                dxu(:, i) = [v; omega];
                velocity_history(:, i, pose_idx) = [v; omega];
            end
            
            % Update simple visualization (simulation_APF.m style) if enabled
            if animation
                updateSimpleVisualizationAPF(r.figure_handle, x, pose_history, pose_idx, ...
                    [start1, start2], [goal1, goal2]);
            end
            
            % Set velocities and step
            r.set_velocities(1:N, dxu);
            r.step();
            
            pose_idx = pose_idx + 1;
            
            % Check if robots reached goals and record times
            dist1 = norm(x(1:2, 1) - goal1);
            dist2 = norm(x(1:2, 2) - goal2);
            
            % Record goal reaching times (only print for first few trials or when animation is on)
            if dist1 < goal_radius && path_end_time(1) == 0
                path_end_time(1) = current_time;
                if animation || n < 5
                    fprintf('Trial %d: Robot 1 reached goal at %.2f seconds\n', n+1, current_time);
                end
            end
            if dist2 < goal_radius && path_end_time(2) == 0
                path_end_time(2) = current_time;
                if animation || n < 5
                    fprintf('Trial %d: Robot 2 reached goal at %.2f seconds\n', n+1, current_time);
                end
            end
            
            if dist1 < goal_radius && dist2 < goal_radius
                if animation || n < 5
                    fprintf('Trial %d: Both robots reached their goals!\n', n+1);
                end
                break;
            end
            
            % Safety check - break if simulation runs too long
            if current_time > simulationTime
                if animation || n < 5
                    fprintf('Trial %d: Simulation timeout reached\n', n+1);
                end
                break;
            end
        end
        
        %% STORE TRIAL DATA
        % Store comprehensive data for later analysis
        trial_data = struct();
        trial_data.poses_robot1_turning = poses_robot1_apf;  % Robot 1 poses during turning
        trial_data.poses_robot2_turning = poses_robot2_apf;  % Robot 2 poses during turning
        trial_data.start_positions = [start1, start2];      % Start positions
        trial_data.goal_positions = [goal1, goal2];         % Goal positions
        trial_data.pose_history = pose_history(:,:,1:pose_idx-1);  % Full pose history [x;y;theta;omega]
        trial_data.velocity_history = velocity_history(:,:,1:pose_idx-1); % Full velocity history
        trial_data.force_history = force_history;            % APF forces
        trial_data.simulation_time = (pose_idx-1) * sampleTime; % Actual simulation time
        trial_data.constant_velocity = constant_linear_velocity; % Target velocity
        
        % Store in cell array (maintain compatibility with original format)
        apf_constant_vel_data{n+1, 1} = poses_robot1_apf;  % For backward compatibility
        apf_constant_vel_data{n+1, 2} = poses_robot2_apf;  % For backward compatibility
        
        % Store full data in separate structure
        if n == 0
            apf_full_data = cell(samples, 1);
        end
        apf_full_data{n+1} = trial_data;
        
        % Increment trial counter
        n = n + 1;
        
        % Progress report every 1000 trials
        if mod(n, 1000) == 0
            fprintf('Completed Trials: %d\n', n);
        end
    end
end

%% POST-SIMULATION DATA HANDLING

% Save data if requested
if record_data
    save("apf_constant_velocity_turn_only.mat", "apf_constant_vel_data", "apf_full_data");
    fprintf('Data saved to apf_constant_velocity_turn_only.mat\n');
    fprintf('  - apf_constant_vel_data: turning data only (backward compatibility)\n');
    fprintf('  - apf_full_data: comprehensive trial data\n');
end

% Plot results from last trial (simplified like simulation_APF.m)
if exist('apf_full_data', 'var') && n > 0
    % Use last trial for plotting
    sample_trial = n;
    data = apf_full_data{sample_trial};
    
    if ~isempty(data) && ~isempty(data.poses_robot1_turning)
        % Create simple visualization
        figure('Name', sprintf('APF Constant Velocity Results (Trial %d)', sample_trial));
        hold on;
        
        % Plot full trajectories
        if size(data.pose_history, 3) > 1
            trajectory1 = squeeze(data.pose_history(1:2, 1, :));
            trajectory2 = squeeze(data.pose_history(1:2, 2, :));
            plot(trajectory1(1,:), trajectory1(2,:), '-', 'Color', [51,102,0]/255, 'LineWidth', 1);
            plot(trajectory2(1,:), trajectory2(2,:), '-', 'Color', [0,0,153]/255, 'LineWidth', 1);
            
            % Highlight turning trajectories (recorded segments)
            if ~isempty(data.poses_robot1_turning)
                turning_traj1 = data.poses_robot1_turning(1:2, :);  % Only x,y positions
                plot(turning_traj1(1,:), turning_traj1(2,:), '-', 'Color', [0,0.7,0], 'LineWidth', 3);
            end
            
            if ~isempty(data.poses_robot2_turning)
                turning_traj2 = data.poses_robot2_turning(1:2, :);  % Only x,y positions
                plot(turning_traj2(1,:), turning_traj2(2,:), '-', 'Color', [0,0,0.7], 'LineWidth', 3);
            end
            
            % Plot safety circles along trajectories to check for violations
            safety_radius = 0.3;
            theta_circle = linspace(0, 2*pi, 30);
            colors_light = {[0.7,1,0.7], [0.7,0.7,1]}; % Light green and light blue
            
            % Sample every 10th point to avoid cluttering
            sample_interval = 10;
            num_points = size(trajectory1, 2);
            
            for k = 1:sample_interval:num_points
                % Robot 1 safety circle
                pos1 = trajectory1(:, k);
                safety_circle1 = pos1 + safety_radius * [cos(theta_circle); sin(theta_circle)];
                plot(safety_circle1(1,:), safety_circle1(2,:), '-', ...
                    'Color', colors_light{1}, 'LineWidth', 0.5);
                
                % Robot 2 safety circle
                pos2 = trajectory2(:, k);
                safety_circle2 = pos2 + safety_radius * [cos(theta_circle); sin(theta_circle)];
                plot(safety_circle2(1,:), safety_circle2(2,:), '-', ...
                    'Color', colors_light{2}, 'LineWidth', 0.5);
            end
            
            % Calculate minimum distance between robots
            distances = zeros(1, num_points);
            for k = 1:num_points
                distances(k) = norm(trajectory1(:, k) - trajectory2(:, k));
            end
            min_distance = min(distances);
            
            % Check for safety violations
            if min_distance < safety_radius
                fprintf('WARNING: Safety violation detected! Minimum distance: %.3f m (< %.3f m)\n', ...
                        min_distance, safety_radius);
                
                % Highlight the closest approach point
                [~, min_idx] = min(distances);
                pos1_min = trajectory1(:, min_idx);
                pos2_min = trajectory2(:, min_idx);
                
                % Draw red circles at closest approach
                safety_circle1_red = pos1_min + safety_radius * [cos(theta_circle); sin(theta_circle)];
                safety_circle2_red = pos2_min + safety_radius * [cos(theta_circle); sin(theta_circle)];
                plot(safety_circle1_red(1,:), safety_circle1_red(2,:), 'r-', 'LineWidth', 2);
                plot(safety_circle2_red(1,:), safety_circle2_red(2,:), 'r-', 'LineWidth', 2);
                
                % Mark the exact closest points
                plot(pos1_min(1), pos1_min(2), 'rx', 'MarkerSize', 10, 'LineWidth', 3);
                plot(pos2_min(1), pos2_min(2), 'rx', 'MarkerSize', 10, 'LineWidth', 3);
            else
                fprintf('Safety maintained: Minimum distance: %.3f m (>= %.3f m)\n', ...
                        min_distance, safety_radius);
            end
        end
        
        % Plot start and goal positions
        plot(data.start_positions(1,1), data.start_positions(2,1), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
        plot(data.goal_positions(1,1), data.goal_positions(2,1), 'kp', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
        plot(data.start_positions(1,2), data.start_positions(2,2), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
        plot(data.goal_positions(1,2), data.goal_positions(2,2), 'kp', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
        
        axis equal;
        xlim([-2, 2]);
        ylim([-1.5, 1.5]);
        xlabel('X (m)'); ylabel('Y (m)');
        title(sprintf('APF Trajectories - Constant Velocity %.2f m/s (Turning Detection)', data.constant_velocity));
        
        % Add legend to explain trajectory segments
        legend_entries = {'Robot 1 Full Path', 'Robot 2 Full Path'};
        if ~isempty(data.poses_robot1_turning)
            legend_entries{end+1} = 'Robot 1 Turning (Recorded)';
        end
        if ~isempty(data.poses_robot2_turning)
            legend_entries{end+1} = 'Robot 2 Turning (Recorded)';
        end
        legend(legend_entries, 'Location', 'best');
        grid on;
        
        % Print statistics
        fprintf('\n=== Trial %d Statistics ===\n', sample_trial);
        fprintf('Simulation time: %.2f seconds\n', data.simulation_time);
        fprintf('Minimum inter-robot distance: %.3f m\n', min_distance);
        fprintf('Safety radius: %.3f m\n', safety_radius);
        fprintf('Turning data points: Robot1=%d, Robot2=%d\n', ...
                size(data.poses_robot1_turning,2), size(data.poses_robot2_turning,2));
    end
end

% Clean up Robotarium
r.debug();

fprintf('\nAPF Constant Velocity Multi-Trial Simulation Complete\n');
fprintf('Total trials completed: %d\n', n);
if record_data
    fprintf('Data saved to: apf_constant_velocity_turn_only.mat\n');
end

%% HELPER FUNCTIONS

function [F_att, F_rep, F_combined] = calculate_apf_forces_mbstyle(current_pos, goal, obstacles, ...
    detection_radius, attraction_factor, repulsion_factor)
    % Calculate APF forces using MbRobot artificial_potential_field algorithm
    % Adapted from MbRobot.m lines 112-143
    
    % Calculate attraction to goal (MbRobot style)
    goal_vector = goal - current_pos;
    if norm(goal_vector) > 0
        F_att = attraction_factor * (goal_vector / norm(goal_vector));
    else
        F_att = [0; 0];
    end
    
    % Calculate repulsion from obstacles
    F_rep = [0; 0];
    if ~isempty(obstacles)
        % Calculate distances to all obstacles
        obstacle_distances = zeros(1, size(obstacles, 2));
        for i = 1:size(obstacles, 2)
            obstacle_distances(i) = norm(obstacles(:, i) - current_pos);
        end
        
        % Find nearest obstacle
        [min_distance, nearest_obstacle_index] = min(obstacle_distances);
        
        % Apply repulsion if within detection range
        if min_distance <= detection_radius && min_distance > 0
            obstacle_vector = current_pos - obstacles(:, nearest_obstacle_index);
            F_rep = (obstacle_vector / norm(obstacle_vector)) * ...
                   (detection_radius - min_distance) / detection_radius;
            F_rep = repulsion_factor * F_rep;
        end
    end
    
    % Combine forces
    F_combined = F_att + F_rep;
    
    % Normalize combined force (MbRobot style)
    if norm(F_combined) > 0
        F_combined = F_combined / norm(F_combined);
    end
end

function [newStart, newGoal, interceptPoint] = generateInterceptingPathSimple(refStart, refGoal, ...
    x_bound, y_bound, min_path_length, safeRadius)
    % Generate intercepting path using simple approach from cbf_constant_velocity.m
    
    max_attempts = 100;
    attempt = 0;
    
    while attempt < max_attempts
        attempt = attempt + 1;
        
        % Direction from start to goal for robot 1
        dir = refGoal - refStart;
        
        % Simple intercept point selection (40-60% along path)
        t = 0.4 + rand() * 0.2;
        interceptPoint = refStart + t * dir;
        
        % Check if intercept point is within bounds
        if interceptPoint(1) >= -x_bound && interceptPoint(1) <= x_bound && ...
           interceptPoint(2) >= -y_bound && interceptPoint(2) <= y_bound
            
            % Distance from intercept point to robot 1 start (this is our radius)
            radius = norm(interceptPoint - refStart);
            
            % Generate robot 2 start position on circle around intercept point
            newStart = generateRandomPointOnCircleBounded(interceptPoint, radius, x_bound, y_bound);
            
            % Simple separation check
            if norm(newStart - refStart) > safeRadius
                % Goal on opposite side of intercept point
                dir_to_goal = interceptPoint - newStart;
                newGoal = interceptPoint + dir_to_goal;
                
                % Ensure goal is within bounds
                newGoal(1) = max(-x_bound, min(x_bound, newGoal(1)));
                newGoal(2) = max(-y_bound, min(y_bound, newGoal(2)));
                
                % Simple success check - just verify minimum path length
                if norm(newGoal - newStart) >= min_path_length
                    return;  % Success!
                end
            end
        end
    end
    
    % Fallback: simple crossing paths
    center = [0; 0];
    angle1 = atan2(refGoal(2) - refStart(2), refGoal(1) - refStart(1));
    angle2 = angle1 + pi/2;  % Perpendicular
    
    fallback_radius = max(min_path_length/2, safeRadius/2);
    newStart = center + fallback_radius * [cos(angle2 + pi); sin(angle2 + pi)];
    newGoal = center + fallback_radius * [cos(angle2); sin(angle2)];
    
    % Ensure within bounds
    newStart(1) = max(-x_bound, min(x_bound, newStart(1)));
    newStart(2) = max(-y_bound, min(y_bound, newStart(2)));
    newGoal(1) = max(-x_bound, min(x_bound, newGoal(1)));
    newGoal(2) = max(-y_bound, min(y_bound, newGoal(2)));
    
    interceptPoint = center;
end

function point = generateRandomPointOnCircleBounded(center, radius, x_bound, y_bound)
    % Generate random point on circle within bounds
    
    max_attempts = 50;
    attempt = 0;
    
    while attempt < max_attempts
        attempt = attempt + 1;
        
        % Generate random angle
        theta = 2 * pi * rand();
        
        % Calculate point on circle
        x = center(1) + radius * cos(theta);
        y = center(2) + radius * sin(theta);
        
        % Check if point is within bounds
        if x >= -x_bound && x <= x_bound && y >= -y_bound && y <= y_bound
            point = [x; y];
            return;
        end
    end
    
    % Fallback: return a point that's within bounds (reduced radius)
    fallback_radius = min(radius, min(x_bound, y_bound) * 0.8);
    angle = rand() * 2 * pi;
    point = center + fallback_radius * [cos(angle); sin(angle)];
end

function updateSimpleVisualizationAPF(fig_handle, current_poses, pose_history, pose_idx, starts, goals)
    % Simple visualization matching simulation_APF.m style
    % Don't use clf to avoid deleting Robotarium objects
    
    figure(fig_handle);
    
    % Clear only our custom plot elements (not Robotarium objects)
    delete(findobj(gca, 'Tag', 'APFTrajectory'));
    delete(findobj(gca, 'Tag', 'APFStartGoal'));
    delete(findobj(gca, 'Tag', 'APFBoundary'));
    delete(findobj(gca, 'Tag', 'APFSafetyCircle'));
    
    hold on;
    
    % Plot trajectories up to current point
    if pose_idx > 1
        colors = {[51,102,0]/255, [0,0,153]/255}; % Green and blue
        for i = 1:2
            trajectory = squeeze(pose_history(1:2, i, 1:pose_idx));
            plot(trajectory(1,:), trajectory(2,:), '-', 'Color', colors{i}, 'LineWidth', 1, 'Tag', 'APFTrajectory');
        end
    end
    
    % Plot safety circles around robots (0.3m radius)
    safety_radius = 0.3;
    theta_circle = linspace(0, 2*pi, 50);
    colors = {[0,1,0], [0,0,1]}; % Green and blue
    
    for i = 1:2
        pos = current_poses(1:2, i);
        safety_circle = pos + safety_radius * [cos(theta_circle); sin(theta_circle)];
        plot(safety_circle(1,:), safety_circle(2,:), '-', ...
            'Color', colors{i}, 'LineWidth', 2, 'Tag', 'APFSafetyCircle');
    end
    
    % Plot start positions (circles)
    plot(starts(1,1), starts(2,1), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'Tag', 'APFStartGoal');
    plot(starts(1,2), starts(2,2), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'b', 'Tag', 'APFStartGoal');
    
    % Plot goal positions (stars)
    plot(goals(1,1), goals(2,1), 'kp', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'Tag', 'APFStartGoal');
    plot(goals(1,2), goals(2,2), 'kp', 'MarkerSize', 10, 'MarkerFaceColor', 'b', 'Tag', 'APFStartGoal');
    
    % Arena boundaries
    rectangle('Position', [-1.6, -1, 3.2, 2], 'EdgeColor', 'k', 'LineWidth', 2, 'Tag', 'APFBoundary');
    
    axis equal;
    xlim([-1.8, 1.8]);
    ylim([-1.2, 1.2]);
    xlabel('X (m)');
    ylabel('Y (m)');
    title('APF Constant Velocity Navigation');
    drawnow;
end

function angle = wrapToPi(angle)
    % Wrap angle to [-pi, pi]
    angle = atan2(sin(angle), cos(angle));
end