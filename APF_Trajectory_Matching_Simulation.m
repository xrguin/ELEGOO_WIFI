% APF Trajectory Matching using exact method from APF_Dual_Robot_Avoidance.m
clc; clear; close all;

%% Load Real Trajectory Data
load("8.21_experiment_apf/trajectories_2.mat", "r1_traj_pts", "r2_traj_pts")
real_r1_x = r1_traj_pts(3,:) + 2500;
real_r1_y = r1_traj_pts(1,:) + 2500;
real_r2_x = r2_traj_pts(3,:) + 2500;
real_r2_y = r2_traj_pts(1,:) + 2500;

% Debug info about real trajectory data
fprintf('Real trajectory data points: %d\n', length(real_r1_x));
fprintf('Real R1 start: [%.1f, %.1f], R1 goal: [%.1f, %.1f]\n', ...
    real_r1_x(1), real_r1_y(1), real_r1_x(end), real_r1_y(end));
fprintf('Real R2 start: [%.1f, %.1f], R2 goal: [%.1f, %.1f]\n', ...
    real_r2_x(1), real_r2_y(1), real_r2_x(end), real_r2_y(end));

%% Initial Conditions
r1_pos = [3518.52; 1386.13];
r1_goal = [357.643; 3252.14];
r2_pos = [357.643; 3252.14];
r2_goal = [3518.52; 1386.13];

%% Parameters (from APF_Dual_Robot_Avoidance.m)
attraction_factor = 1;
repulsion_factor = 1.4;
detection_radius = 2000;  % mm
desired_motor_speed = 50;
robot_speed = 3.67 * desired_motor_speed - 31.45;  % Convert to mm/s
dt = 0.05;  % 20 Hz control

%% Initialize States
r1_traj = r1_pos;
r2_traj = r2_pos;

% Hysteresis states (critical for matching real behavior)
r1_prev_obstacle_dist = norm(r1_pos - r2_pos);
r2_prev_obstacle_dist = r1_prev_obstacle_dist;
r1_min_dist_reached = false;
r2_min_dist_reached = false;

% PID states for heading control (creates realistic wiggling)
% PID gains from real robot
Kp = 0.8; Ki = 0.2; Kd = 0.15;
r1_heading = atan2(r1_goal(2) - r1_pos(2), r1_goal(1) - r1_pos(1));
r2_heading = atan2(r2_goal(2) - r2_pos(2), r2_goal(1) - r2_pos(1));
r1_integral = 0; r1_prev_error = 0;
r2_integral = 0; r2_prev_error = 0;

%% Setup Visualization
figure('Position', [100, 100, 1200, 600]);

hold on; grid on; axis equal;
plot(real_r1_x, real_r1_y, 'b-', 'LineWidth', 2, 'DisplayName', 'Robot 1 (Real)');
plot(real_r2_x, real_r2_y, 'g-', 'LineWidth', 2, 'DisplayName', 'Robot 2 (Real)');
h_sim_r1 = plot(r1_pos(1), r1_pos(2), 'b--', 'LineWidth', 2, 'DisplayName', 'Robot 1 (Sim)');
h_sim_r2 = plot(r2_pos(1), r2_pos(2), 'g--', 'LineWidth', 2, 'DisplayName', 'Robot 2 (Sim)');
plot(r1_pos(1), r1_pos(2), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
plot(r1_goal(1), r1_goal(2), 'b*', 'MarkerSize', 15);
plot(r2_pos(1), r2_pos(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot(r2_goal(1), r2_goal(2), 'g*', 'MarkerSize', 15);
xlabel('X (mm)'); ylabel('Y (mm)');
title('Trajectory Comparison');
legend('Location', 'best');
xlim([0, 4000]); ylim([0, 4000]);


%% Main Simulation Loop
time_vec = [];

for step = 1:600  % Max 30 seconds
    t = (step-1) * dt;
    
    % Store current positions for simultaneous update
    r1_current = r1_pos;
    r2_current = r2_pos;
    
    % Calculate APF forces for Robot 1
    [F_att_1, F_rep_1, F_combined_1, r1_prev_obstacle_dist, r1_min_dist_reached] = ...
        calculate_apf_forces(r1_current, r1_goal, r2_current, detection_radius, ...
        attraction_factor, repulsion_factor, r1_prev_obstacle_dist, r1_min_dist_reached);
    
    % Calculate APF forces for Robot 2
    [F_att_2, F_rep_2, F_combined_2, r2_prev_obstacle_dist, r2_min_dist_reached] = ...
        calculate_apf_forces(r2_current, r2_goal, r1_current, detection_radius, ...
        attraction_factor, repulsion_factor, r2_prev_obstacle_dist, r2_min_dist_reached);
    
    % PID Heading Control for Robot 1
    desired_heading_1 = atan2(F_combined_1(2), F_combined_1(1));
    error_1 = atan2(sin(desired_heading_1 - r1_heading), cos(desired_heading_1 - r1_heading));
    r1_integral = r1_integral + error_1 * dt;
    r1_integral = max(min(r1_integral, 2), -2); % Anti-windup
    derivative_1 = (error_1 - r1_prev_error) / dt;
    heading_change_1 = Kp * error_1 + Ki * r1_integral + Kd * derivative_1;
    r1_heading = r1_heading + heading_change_1 * dt;
    r1_prev_error = error_1;
    
    % PID Heading Control for Robot 2
    desired_heading_2 = atan2(F_combined_2(2), F_combined_2(1));
    error_2 = atan2(sin(desired_heading_2 - r2_heading), cos(desired_heading_2 - r2_heading));
    r2_integral = r2_integral + error_2 * dt;
    r2_integral = max(min(r2_integral, 2), -2); % Anti-windup
    derivative_2 = (error_2 - r2_prev_error) / dt;
    heading_change_2 = Kp * error_2 + Ki * r2_integral + Kd * derivative_2;
    r2_heading = r2_heading + heading_change_2 * dt;
    r2_prev_error = error_2;
    
    % Update positions using actual headings (not desired)
    r1_pos = r1_pos + robot_speed * dt * [cos(r1_heading); sin(r1_heading)];
    r2_pos = r2_pos + robot_speed * dt * [cos(r2_heading); sin(r2_heading)];
    
    % Store trajectories
    r1_traj(:,end+1) = r1_pos;
    r2_traj(:,end+1) = r2_pos;
    
    
    
    % Update visualization every 10 steps
    if mod(step, 10) == 0
        set(h_sim_r1, 'XData', r1_traj(1,:), 'YData', r1_traj(2,:));
        set(h_sim_r2, 'XData', r2_traj(1,:), 'YData', r2_traj(2,:));

        drawnow;
    end
    

    % Check goal reached
    if norm(r1_pos - r1_goal) < 200 && norm(r2_pos - r2_goal) < 200
        fprintf('Goals reached at t = %.2f s\n', t);
        break;
    end
end

%% Final Statistics
fprintf('\n=== Results ===\n');
if ~isempty(errors_r1)
    fprintf('Mean error R1: %.1f mm\n', mean(errors_r1));
    fprintf('Mean error R2: %.1f mm\n', mean(errors_r2));
    fprintf('Max error R1: %.1f mm\n', max(errors_r1));
    fprintf('Max error R2: %.1f mm\n', max(errors_r2));
    fprintf('Std error R1: %.1f mm\n', std(errors_r1));
    fprintf('Std error R2: %.1f mm\n', std(errors_r2));
else
    fprintf('No error data collected\n');
end

%% EXACT APF Function from APF_Dual_Robot_Avoidance.m (lines 472-516)
function [F_att, F_rep, F_combined, new_prev_dist, new_min_reached] = ...
    calculate_apf_forces(current_pos, goal, obstacle, detection_radius, ...
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
        
        % Hysteresis logic
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