% APF Trajectory Matching Simulation
% This script loads trajectory data from a real-world experiment and runs a
% simulation with identical parameters to compare the results.

clc; clear; close all;

%% Load Real Trajectory Data
% Load the specified .mat file containing the experimental data.
data_file = 'trajectories_7.mat';
fprintf('Loading real trajectory data from: %s\n', data_file);
load(data_file, 'r1_traj_pts', 'r2_traj_pts', 'r1_dt_data', 'r2_dt_data');

%% Coordinate System Transformation
% The real-world data is 3D [X, Y, Z]. The simulation is 2D.
% We map the real-world XZ plane to the simulation's XY plane.
% sim_x = real_z, sim_y = real_x
real_r1_traj_2d = [r1_traj_pts(3, :); r1_traj_pts(1, :)];
real_r2_traj_2d = [r2_traj_pts(3, :); r2_traj_pts(1, :)];

% Offset all coordinates to be positive for visualization
all_points = [real_r1_traj_2d, real_r2_traj_2d];
min_coords = min(all_points, [], 2);
shift = -min_coords + 500; % Add a 500mm buffer
shift = 3000;
real_r1_traj_2d = real_r1_traj_2d + shift;
real_r2_traj_2d = real_r2_traj_2d + shift;

%% Set Initial and Goal Conditions from Loaded Data
% The simulation must start and end at the same points as the real experiment.
r1_pos = real_r1_traj_2d(:, 1);
r1_goal = real_r1_traj_2d(:, end);
r2_pos = real_r2_traj_2d(:, 1);
r2_goal = real_r2_traj_2d(:, end);

fprintf('R1 Start: [%.1f, %.1f], Goal: [%.1f, %.1f]\n', r1_pos(1), r1_pos(2), r1_goal(1), r1_goal(2));
fprintf('R2 Start: [%.1f, %.1f], Goal: [%.1f, %.1f]\n', r2_pos(1), r2_pos(2), r2_goal(1), r2_goal(2));

%% Parameters (from APF_Dual_Robot_Avoidance.m)
attraction_factor = 1.2;
repulsion_factor = 1.2;
detection_radius = 2000;  % mm
desired_motor_speed = 50;
robot_speed = 3.67 * desired_motor_speed - 31.45; % mm/s

% PID gains for heading control
Kp = 0.8; Ki = 0.2; Kd = 0.15;

%% Initialize Simulation States
r1_traj_sim = r1_pos;
r2_traj_sim = r2_pos;

% Hysteresis states for APF force calculation
r1_prev_obstacle_dist = norm(r1_pos - r2_pos);
r2_prev_obstacle_dist = norm(r1_pos - r2_pos);
r1_min_dist_reached = false;
r2_min_dist_reached = false;

% PID states for heading control
r1_heading = atan2(r1_goal(2) - r1_pos(2), r1_goal(1) - r1_pos(1));
r2_heading = atan2(r2_goal(2) - r2_pos(2), r2_goal(1) - r2_pos(1));
r1_integral = 0; r1_prev_error = 0;
r2_integral = 0; r2_prev_error = 0;

%% Setup Visualization
figure('Name', 'APF Simulation vs. Real Trajectory', 'units', 'normalized', 'outerposition', [0 0 1 1]);
hold on; grid on; axis equal;
title('APF Simulation vs. Real Trajectory');
xlabel('X (mm)'); ylabel('Y (mm)');

% Plot real trajectories
plot(real_r1_traj_2d(1,:), real_r1_traj_2d(2,:), 'b-', 'LineWidth', 2, 'DisplayName', 'Robot 1 (Real)');
plot(real_r2_traj_2d(1,:), real_r2_traj_2d(2,:), 'g-', 'LineWidth', 2, 'DisplayName', 'Robot 2 (Real)');

% Plot simulated trajectories (handles will be updated in the loop)
h_sim_r1 = plot(r1_pos(1), r1_pos(2), 'b--', 'LineWidth', 2, 'DisplayName', 'Robot 1 (Sim)');
h_sim_r2 = plot(r2_pos(1), r2_pos(2), 'g--', 'LineWidth', 2, 'DisplayName', 'Robot 2 (Sim)');

% Plot start and goal markers
plot(r1_pos(1), r1_pos(2), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
plot(r1_goal(1), r1_goal(2), 'b*', 'MarkerSize', 15, 'LineWidth', 2);
plot(r2_pos(1), r2_pos(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot(r2_goal(1), r2_goal(2), 'g*', 'MarkerSize', 15, 'LineWidth', 2);

legend('Location', 'best');
xlim([-500, 4000]); ylim([-500, 4000]);

%% Main Simulation Loop
num_steps = min([size(r1_traj_pts, 2), size(r2_traj_pts, 2), numel(r1_dt_data), numel(r2_dt_data)]);
fprintf('Running simulation for %d steps.\n', num_steps);

for step = 1:num_steps
    % Use the variable time step from the recorded data
    dt1 = r1_dt_data(step);
    dt2 = r2_dt_data(step);
    
    % Store current positions for simultaneous update
    r1_current = r1_pos;
    r2_current = r2_pos;
    
    % Calculate APF forces for Robot 1
    [~, ~, F_combined_1, r1_prev_obstacle_dist, r1_min_dist_reached] = calculate_apf_forces(r1_current, r1_goal, r2_current, detection_radius,  attraction_factor, repulsion_factor, r1_prev_obstacle_dist, r1_min_dist_reached);
    
    % Calculate APF forces for Robot 2
    [~, ~, F_combined_2, r2_prev_obstacle_dist, r2_min_dist_reached] = ...
        calculate_apf_forces(r2_current, r2_goal, r1_current, detection_radius, ...
        attraction_factor, repulsion_factor, r2_prev_obstacle_dist, r2_min_dist_reached);
    
    % PID Heading Control for Robot 1
    desired_heading_1 = atan2(F_combined_1(2), F_combined_1(1));
    error_1 = atan2(sin(desired_heading_1 - r1_heading), cos(desired_heading_1 - r1_heading));
    r1_integral = r1_integral + error_1 * dt1;
    r1_integral = max(min(r1_integral, 2), -2); % Anti-windup
    if dt1 > 0, derivative_1 = (error_1 - r1_prev_error) / dt1; else, derivative_1 = 0; end
    heading_change_1 = Kp * error_1 + Ki * r1_integral + Kd * derivative_1;
    r1_heading = r1_heading + heading_change_1 * dt1;
    r1_prev_error = error_1;
    
    % PID Heading Control for Robot 2
    desired_heading_2 = atan2(F_combined_2(2), F_combined_2(1));
    error_2 = atan2(sin(desired_heading_2 - r2_heading), cos(desired_heading_2 - r2_heading));
    r2_integral = r2_integral + error_2 * dt2;
    r2_integral = max(min(r2_integral, 2), -2); % Anti-windup
    if dt2 > 0, derivative_2 = (error_2 - r2_prev_error) / dt2; else, derivative_2 = 0; end
    heading_change_2 = Kp * error_2 + Ki * r2_integral + Kd * derivative_2;
    r2_heading = r2_heading + heading_change_2 * dt2;
    r2_prev_error = error_2;
    
    % Update positions using actual headings (not desired)
    r1_pos = r1_pos + robot_speed * dt1 * [cos(r1_heading); sin(r1_heading)];
    r2_pos = r2_pos + robot_speed * dt2 * [cos(r2_heading); sin(r2_heading)];
    
    % Store trajectories
    r1_traj_sim(:,end+1) = r1_pos;
    r2_traj_sim(:,end+1) = r2_pos;
    
    % Update visualization every 10 steps
    if mod(step, 10) == 0
        set(h_sim_r1, 'XData', r1_traj_sim(1,:), 'YData', r1_traj_sim(2,:));
        set(h_sim_r2, 'XData', r2_traj_sim(1,:), 'YData', r2_traj_sim(2,:));
        drawnow;
    end
    
    % Check goal reached
    if norm(r1_pos - r1_goal) < 200 && norm(r2_pos - r2_goal) < 200
        fprintf('Goals reached at t = %.2f s\n', sum(r1_dt_data(1:step)));
        break;
    end
end

% Final plot update
set(h_sim_r1, 'XData', r1_traj_sim(1,:), 'YData', r1_traj_sim(2,:));
set(h_sim_r2, 'XData', r2_traj_sim(1,:), 'YData', r2_traj_sim(2,:));
drawnow;
fprintf('Simulation finished.\n');


%% Original APF Function from Simulation Script
function [F_att, F_rep, F_combined, new_prev_dist, new_min_reached] =...
    calculate_apf_forces(current_pos, goal, obstacle, detection_radius,...
    attraction_factor, repulsion_factor, prev_dist, min_reached)
    
    % 1. Calculate attractive force as a unit vector
    goal_vector = goal - current_pos;
    if norm(goal_vector) > 0
        F_att_unit = goal_vector / norm(goal_vector);
    else
        F_att_unit = [0; 0];
    end

    % 2. Calculate repulsive force with conditional boost
    F_rep_scaled = [0; 0];
    obstacle_distance = norm(obstacle - current_pos);
    new_prev_dist = obstacle_distance;
    new_min_reached = min_reached;
    
    if obstacle_distance <= detection_radius && obstacle_distance > 0
        obstacle_vector = current_pos - obstacle;
        
        rep_magnitude_normalized = (detection_radius - obstacle_distance) / detection_radius;
        
        % Hysteresis logic is handled inside this function
        if obstacle_distance > prev_dist
            new_min_reached = true;  % Set flag permanently once robots start moving apart
        end
        
        use_boost = (obstacle_distance < prev_dist) && ~new_min_reached;
        
        if use_boost
            % Force is increasing: use boosted sqrt logic
            rep_magnitude = sqrt(rep_magnitude_normalized);
        else
            % Force is decreasing or min distance has been passed: use normal linear logic
            rep_magnitude = rep_magnitude_normalized;
        end
        
        F_rep_scaled = (obstacle_vector / norm(obstacle_vector)) * rep_magnitude;
    end

    % For compatibility with plotting
    F_att = attraction_factor * F_att_unit;
    F_rep = repulsion_factor * F_rep_scaled;

    % 3. Combine forces
    F_combined = F_att + F_rep;
    
    % 4. Normalize
    if norm(F_combined) > 0
        F_combined = F_combined / norm(F_combined);
    end
end
