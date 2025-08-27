% APF Trajectory Matching with Dynamic Collision Scenario Generation
clc; clear; close all;

%% Scenario & Boundary Configuration (from APF_Dual_Robot_Avoidance.m)
% -- Boundary points from the real-world setup
boundary_pts_3d = [ 2467, 50, 148; 1814, 65, -2659; -1878, 37, -2732; -1101, 6.8, 1769];

% -- Convert to 2D simulation coordinates (sim_x = raw_z, sim_y = raw_x)
% We add a buffer to keep paths away from the absolute edge.
buffer = 200; 
x_bound = (max(boundary_pts_3d(:,3)) - min(boundary_pts_3d(:,3))) / 2 - buffer;
y_bound = (max(boundary_pts_3d(:,1)) - min(boundary_pts_3d(:,1))) / 2 - buffer;
center_offset = [
    (max(boundary_pts_3d(:,3)) + min(boundary_pts_3d(:,3))) / 2;
    (max(boundary_pts_3d(:,1)) + min(boundary_pts_3d(:,1))) / 2
];

% -- Path generation parameters
min_path_length = 2500; % Minimum travel distance for robots
safeRadius = 500;       % Minimum starting distance between robots

%% Generate Dynamic Start and Goal Positions
% Generate a valid, non-colliding start path for Robot 1
refStart = [(2*rand-1) * x_bound; (2*rand-1) * y_bound];
refGoal = [(2*rand-1) * x_bound; (2*rand-1) * y_bound];
while norm(refGoal - refStart) < min_path_length
    refGoal = [(2*rand-1) * x_bound; (2*rand-1) * y_bound];
end

% Generate an intercepting path for Robot 2
[r2_pos, r2_goal, interceptPoint] = generateInterceptingPathSimple(refStart, refGoal, ...
    x_bound, y_bound, min_path_length, safeRadius);

% Assign Robot 1 paths
r1_pos = refStart;
r1_goal = refGoal;

% Offset all coordinates to be positive for plotting
all_points = [r1_pos, r1_goal, r2_pos, r2_goal];
min_coords = min(all_points, [], 2);
shift = -min_coords + buffer;
r1_pos = r1_pos + shift;
r1_goal = r1_goal + shift;
r2_pos = r2_pos + shift;
r2_goal = r2_goal + shift;
interceptPoint = interceptPoint + shift;
plot_bounds = [max(all_points(1,:) + shift(1)), max(all_points(2,:) + shift(2))] + buffer;

fprintf('R1 Start: [%.1f, %.1f], Goal: [%.1f, %.1f]\n', r1_pos(1), r1_pos(2), r1_goal(1), r1_goal(2));
fprintf('R2 Start: [%.1f, %.1f], Goal: [%.1f, %.1f]\n', r2_pos(1), r2_pos(2), r2_goal(1), r2_goal(2));
fprintf('Est. Intercept: [%.1f, %.1f]\n', interceptPoint(1), interceptPoint(2));


%% Parameters (from APF_Dual_Robot_Avoidance.m)
attraction_factor = 1;
repulsion_factor = 1.4;
detection_radius = 2000;  % mm
desired_motor_speed = 50;
robot_speed = 3.67 * desired_motor_speed - 31.45;  % Convert to mm/s
dt = 0.1; % Fixed time step for simulation

%% Initialize States
r1_traj = r1_pos;
r2_traj = r2_pos;

% Hysteresis states (critical for matching real behavior)
r1_prev_obstacle_dist = norm(r1_pos - r2_pos);
r2_prev_obstacle_dist = r1_prev_obstacle_dist;
r1_min_dist_reached = false;
r2_min_dist_reached = false;

% PID states for heading control (creates realistic wiggling)
Kp = 0.8; Ki = 0.2; Kd = 0.15;
r1_heading = atan2(r1_goal(2) - r1_pos(2), r1_goal(1) - r1_pos(1));
r2_heading = atan2(r2_goal(2) - r2_pos(2), r2_goal(1) - r2_pos(1));
r1_integral = 0; r1_prev_error = 0;
r2_integral = 0; r2_prev_error = 0;

%% Setup Visualization
figure('Position', [100, 100, 1000, 800]);

hold on; grid on; axis equal;
h_sim_r1 = plot(r1_pos(1), r1_pos(2), 'b--', 'LineWidth', 2, 'DisplayName', 'Robot 1 (Sim)');
h_sim_r2 = plot(r2_pos(1), r2_pos(2), 'g--', 'LineWidth', 2, 'DisplayName', 'Robot 2 (Sim)');
plot(r1_pos(1), r1_pos(2), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b', 'DisplayName', 'R1 Start');
plot(r1_goal(1), r1_goal(2), 'b*', 'MarkerSize', 15, 'DisplayName', 'R1 Goal');
plot(r2_pos(1), r2_pos(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'R2 Start');
plot(r2_goal(1), r2_goal(2), 'g*', 'MarkerSize', 15, 'DisplayName', 'R2 Goal');
plot(interceptPoint(1), interceptPoint(2), 'rx', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Intercept');
xlabel('X (mm)'); ylabel('Y (mm)');
title('Dynamic APF Simulation');
legend('Location', 'best');
xlim([0, plot_bounds(1)]); ylim([0, plot_bounds(2)]);


%% Main Simulation Loop
num_steps = 2000; % Max simulation steps
total_time = 0;

for step = 1:num_steps
    total_time = total_time + dt;
    
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
    
    % Update positions using actual headings
    r1_pos = r1_pos + robot_speed * dt * [cos(r1_heading); sin(r1_heading)];
    r2_pos = r2_pos + robot_speed * dt * [cos(r2_heading); sin(r2_heading)];
    
    % Store trajectories
    r1_traj(:,end+1) = r1_pos;
    r2_traj(:,end+1) = r2_pos;
    
    % Update visualization
    if mod(step, 5) == 0
        set(h_sim_r1, 'XData', r1_traj(1,:), 'YData', r1_traj(2,:));
        set(h_sim_r2, 'XData', r2_traj(1,:), 'YData', r2_traj(2,:));
        drawnow;
    end
    
    % Check goal reached
    if norm(r1_pos - r1_goal) < 200 || norm(r2_pos - r2_goal) < 200
        disp('A robot has reached its goal.');
        if norm(r1_pos - r1_goal) < 200 && norm(r2_pos - r2_goal) < 200
            fprintf('Both goals reached at t = %.2f s\n', total_time);
        end
        break;
    end
end

% Final plot update
set(h_sim_r1, 'XData', r1_traj(1,:), 'YData', r1_traj(2,:));
set(h_sim_r2, 'XData', r2_traj(1,:), 'YData', r2_traj(2,:));
drawnow;
fprintf('Simulation finished after %d steps.\n', step);


%% APF and Path Generation Functions

function [F_att, F_rep, F_combined, new_prev_dist, new_min_reached] = ...
    calculate_apf_forces(current_pos, goal, obstacle, detection_radius, ...
    attraction_factor, repulsion_factor, prev_dist, min_reached)
    
    goal_vector = goal - current_pos;
    if norm(goal_vector) > 0, F_att_unit = goal_vector / norm(goal_vector);
    else, F_att_unit = [0; 0]; end

    F_rep_scaled = [0; 0];
    obstacle_distance = norm(obstacle - current_pos);
    new_prev_dist = obstacle_distance;
    new_min_reached = min_reached;
    
    if obstacle_distance <= detection_radius && obstacle_distance > 0
        obstacle_vector = current_pos - obstacle;
        rep_magnitude_normalized = (detection_radius - obstacle_distance) / detection_radius;
        if obstacle_distance > prev_dist, new_min_reached = true; end
        use_boost = (obstacle_distance < prev_dist) && ~new_min_reached;
        if use_boost, rep_magnitude = sqrt(rep_magnitude_normalized);
        else, rep_magnitude = rep_magnitude_normalized; end
        F_rep_scaled = (obstacle_vector / norm(obstacle_vector)) * rep_magnitude;
    end

    F_att = attraction_factor * F_att_unit;
    F_rep = repulsion_factor * F_rep_scaled;
    F_combined = F_att + F_rep;
    
    if norm(F_combined) > 0, F_combined = F_combined / norm(F_combined); end
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
