% =======================================================================
% Velocity_Calibration.m
%
% Calibration script to measure the relationship between motor commands
% and actual velocity measured by OptiTrack
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
% 1. CONFIGURATION
% =======================================================================

% -- Robot Configuration
robot_ip = "192.168.1.84";

% -- NatNet Server Configuration
natnet_client_ip = '192.168.1.70';
natnet_server_ip = '192.168.1.209';

% -- Waypoints from PID_Velocity_Profile_Control.m
way_points = [1450, 23, 707;
    -972.4, 42, -2017.3];  % Just using first and last waypoints

way_points = way_points - [100, 0, 100];  % Same offset as in PID control
start_waypoint = way_points(1, :);
goal_waypoint = way_points(end, :);

% -- Test Parameters
motor_speeds_to_test = [40, 60, 80, 100];  % Motor PWM values to test
test_duration = 5;  % Duration to run each speed test (seconds)
alignment_tolerance = 30;  % Degrees tolerance for initial alignment
robot_id = 1;

% -- Data Collection
velocity_data = struct();
velocity_data.motor_speeds = motor_speeds_to_test;
velocity_data.measured_velocities = [];
velocity_data.std_deviations = [];

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

% -- Initialize Plot for Real-time Velocity Display
figure('Name', 'Velocity Calibration', 'Position', [100, 100, 1200, 600]);

% Subplot 1: Real-time velocity
subplot(1, 2, 1);
h_velocity_plot = plot(0, 0, 'b-', 'LineWidth', 2);
hold on;
h_target_line = yline(0, 'r--', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Velocity (mm/s)');
title('Real-time Velocity Measurement');
grid on;
xlim([0, test_duration]);
ylim([0, 2000]);
legend('Measured Velocity', 'Target Speed', 'Location', 'best');

% Subplot 2: Calibration curve
subplot(1, 2, 2);
h_calibration = plot(0, 0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
hold on;
h_fit_line = plot(0, 0, 'r-', 'LineWidth', 2);
xlabel('Motor Speed (PWM)');
ylabel('Measured Velocity (mm/s)');
title('Motor Speed vs Actual Velocity');
grid on;
xlim([0, max(motor_speeds_to_test) + 20]);
ylim([0, 2000]);

% =======================================================================
% 3. INITIAL POSITIONING AND ALIGNMENT PHASE
% =======================================================================
disp('=== INITIAL POSITIONING AND ALIGNMENT PHASE ===');
disp(['Moving to start position: X=' num2str(start_waypoint(1)) ', Z=' num2str(start_waypoint(3)) ' mm']);

% First, move to the start waypoint
positioning_complete = false;
positioning_tolerance = 100;  % mm
last_loop_time = tic;

while ~positioning_complete
    dt = toc(last_loop_time);
    last_loop_time = tic;
    
    data = natnetclient.getFrame;
    if isempty(data.RigidBodies)
        disp('No rigid bodies in frame...');
        pause(0.01);
        continue;
    end
    
    rb_data = data.RigidBodies(2);  % Adjust index as needed
    current_pos_3d = [rb_data.x, rb_data.y, rb_data.z] * 1000;
    
    % Calculate distance to start waypoint
    error_vec = [start_waypoint(1) - current_pos_3d(1), start_waypoint(3) - current_pos_3d(3)];
    distance_to_start = norm(error_vec);
    
    if distance_to_start < positioning_tolerance
        disp('Reached start position!');
        send_robot_command(robot_ip, 'S');
        positioning_complete = true;
        pause(1);
    else
        % Get current orientation
        q = quaternion(rb_data.qw, rb_data.qx, rb_data.qy, rb_data.qz);
        angles = q.EulerAngles('YZX');
        current_angle = rad2deg(angles(1)) + 230;
        if current_angle >= 180
            current_angle = -360 + current_angle;
        end
        
        % Calculate heading to start waypoint
        target_angle = rad2deg(atan2(error_vec(1), error_vec(2)));
        heading_error = target_angle - current_angle;
        heading_error = atan2(sin(deg2rad(heading_error)), cos(deg2rad(heading_error)));
        heading_error = rad2deg(heading_error);
        
        % Simple proportional control
        forward_speed = min(0.3 * distance_to_start, 80);
        turn_speed = 1.0 * heading_error;
        turn_speed = min(max(turn_speed, -60), 60);
        
        left_speed = round(forward_speed + turn_speed);
        right_speed = round(forward_speed - turn_speed);
        
        left_speed = min(max(left_speed, -150), 150);
        right_speed = min(max(right_speed, -150), 150);
        
        json_command = sprintf('{"N":4,"D1":%d,"D2":%d,"H":"cal"}', left_speed, right_speed);
        send_robot_command(robot_ip, json_command);
        
        fprintf('Moving to start: Distance=%.1f mm, Heading error=%.1f deg\n', ...
                distance_to_start, heading_error);
    end
    
    pause(0.05);
end

% Now align to face the goal waypoint
disp(['Aligning to face goal position: X=' num2str(goal_waypoint(1)) ', Z=' num2str(goal_waypoint(3)) ' mm']);
alignment_complete = false;

while ~alignment_complete
    dt = toc(last_loop_time);
    last_loop_time = tic;
    
    data = natnetclient.getFrame;
    if isempty(data.RigidBodies)
        pause(0.01);
        continue;
    end
    
    rb_data = data.RigidBodies(2);
    current_pos_3d = [rb_data.x, rb_data.y, rb_data.z] * 1000;
    
    % Get current orientation
    q = quaternion(rb_data.qw, rb_data.qx, rb_data.qy, rb_data.qz);
    angles = q.EulerAngles('YZX');
    current_angle = rad2deg(angles(1)) + 230;
    if current_angle >= 180
        current_angle = -360 + current_angle;
    end
    
    % Calculate heading to goal waypoint
    goal_vec = [goal_waypoint(1) - current_pos_3d(1), goal_waypoint(3) - current_pos_3d(3)];
    target_angle = rad2deg(atan2(goal_vec(1), goal_vec(2)));
    heading_error = target_angle - current_angle;
    heading_error = atan2(sin(deg2rad(heading_error)), cos(deg2rad(heading_error)));
    heading_error = rad2deg(heading_error);
    
    fprintf('Aligning to goal: Current angle: %.1f | Target: %.1f | Error: %.1f\n', ...
            current_angle, target_angle, heading_error);
    
    if abs(heading_error) < alignment_tolerance
        disp('Alignment to goal complete!');
        send_robot_command(robot_ip, 'S');
        alignment_complete = true;
        pause(1);  % Brief pause before starting tests
    else
        % Rotate to align
        turn_speed = 1.0 * heading_error;
        turn_speed = min(max(turn_speed, -75), 75);
        json_command = sprintf('{"N":3,"D1":1,"D2":%d,"H":"cal"}', round(abs(turn_speed)));
        send_robot_command(robot_ip, json_command);
    end
    
    pause(0.05);
end

% =======================================================================
% 4. VELOCITY CALIBRATION TESTS
% =======================================================================
disp('=== STARTING VELOCITY CALIBRATION TESTS ===');

% The starting position is now the start_waypoint we moved to
start_position = start_waypoint;
fprintf('Tests will start from position: X=%.1f, Z=%.1f mm\n', start_position(1), start_position(3));
fprintf('Robot is aligned toward goal: X=%.1f, Z=%.1f mm\n', goal_waypoint(1), goal_waypoint(3));

for speed_idx = 1:length(motor_speeds_to_test)
    current_motor_speed = motor_speeds_to_test(speed_idx);
    
    fprintf('\n--- Test %d/%d: Motor Speed = %d ---\n', ...
            speed_idx, length(motor_speeds_to_test), current_motor_speed);
    
    % Initialize data collection for this speed
    velocity_samples = [];
    time_stamps = [];
    prev_position = [];
    position_initialized = false;
    
    % Start the robot moving forward
    json_command = sprintf('{"N":4,"D1":%d,"D2":%d,"H":"cal"}', ...
                          current_motor_speed, current_motor_speed);
    send_robot_command(robot_ip, json_command);
    
    test_start_time = tic;
    last_sample_time = tic;
    
    % Collect velocity data for test_duration seconds
    while toc(test_start_time) < test_duration
        dt = toc(last_sample_time);
        if dt < 0.05  % Sample at ~20Hz
            continue;
        end
        last_sample_time = tic;
        
        data = natnetclient.getFrame;
        if isempty(data.RigidBodies)
            continue;
        end
        
        rb_data = data.RigidBodies(2);
        current_pos_3d = [rb_data.x, rb_data.y, rb_data.z] * 1000;
        
        % Calculate velocity
        if position_initialized && dt > 0
            % Calculate 2D velocity in XZ plane
            position_delta = sqrt((current_pos_3d(1) - prev_position(1))^2 + ...
                                (current_pos_3d(3) - prev_position(3))^2);
            instant_velocity = position_delta / dt;  % mm/s
            
            % Store sample
            velocity_samples(end+1) = instant_velocity;
            time_stamps(end+1) = toc(test_start_time);
            
            % Update real-time plot
            set(h_velocity_plot, 'XData', time_stamps, 'YData', velocity_samples);
            set(h_target_line, 'Value', current_motor_speed * 10);  % Rough estimate for visualization
            drawnow;
            
            fprintf('Time: %.2f s | Velocity: %.1f mm/s\n', ...
                   toc(test_start_time), instant_velocity);
        else
            position_initialized = true;
        end
        
        prev_position = current_pos_3d;
    end
    
    % Stop the robot
    send_robot_command(robot_ip, 'S');
    
    % Calculate statistics for this speed
    % Remove first 1 second (acceleration) and last 0.5 seconds (deceleration)
    % Assuming ~20Hz sampling rate
    start_trim = min(20, floor(length(velocity_samples) * 0.2));  % Remove first 1s or 20% of samples
    end_trim = min(10, floor(length(velocity_samples) * 0.1));    % Remove last 0.5s or 10% of samples
    
    if length(velocity_samples) > (start_trim + end_trim + 10)
        steady_state_samples = velocity_samples(start_trim+1:end-end_trim);
        fprintf('Using samples %d to %d (out of %d total) for steady-state analysis\n', ...
                start_trim+1, length(velocity_samples)-end_trim, length(velocity_samples));
    else
        steady_state_samples = velocity_samples;
        fprintf('Warning: Not enough samples for trimming. Using all %d samples.\n', ...
                length(velocity_samples));
    end
    
    mean_velocity = mean(steady_state_samples);
    std_velocity = std(steady_state_samples);
    
    % Store results
    velocity_data.measured_velocities(speed_idx) = mean_velocity;
    velocity_data.std_deviations(speed_idx) = std_velocity;
    
    fprintf('Results: Mean velocity = %.1f mm/s, Std = %.1f mm/s\n', ...
           mean_velocity, std_velocity);
    fprintf('Steady-state velocity range: [%.1f, %.1f] mm/s\n', ...
           min(steady_state_samples), max(steady_state_samples));
    
    % Update calibration plot
    set(h_calibration, 'XData', motor_speeds_to_test(1:speed_idx), ...
                      'YData', velocity_data.measured_velocities);
    
    % Fit linear model if we have at least 2 points
    if speed_idx >= 2
        p = polyfit(motor_speeds_to_test(1:speed_idx), ...
                   velocity_data.measured_velocities, 1);
        fit_x = [0, max(motor_speeds_to_test)];
        fit_y = polyval(p, fit_x);
        set(h_fit_line, 'XData', fit_x, 'YData', fit_y);
        
        % Display conversion factor
        conversion_factor = p(1);  % mm/s per PWM unit
        fprintf('Current conversion factor: %.2f mm/s per PWM unit\n', conversion_factor);
    end
    
    drawnow;
    
    % Return to starting position between tests
    if speed_idx < length(motor_speeds_to_test)
        disp('Returning to starting position...');
        pause(1);  % Brief pause after stopping
        
        % Get current position and calculate return path
        return_complete = false;
        return_tolerance = 100;  % mm tolerance for returning to start
        
        while ~return_complete
            data = natnetclient.getFrame;
            if isempty(data.RigidBodies)
                pause(0.01);
                continue;
            end
            
            rb_data = data.RigidBodies(2);
            current_pos_3d = [rb_data.x, rb_data.y, rb_data.z] * 1000;
            
            % Calculate distance to starting position
            error_vec = [start_position(1) - current_pos_3d(1), 
                        start_position(3) - current_pos_3d(3)];
            distance_to_start = norm(error_vec);
            
            if distance_to_start < return_tolerance
                disp('Returned to starting position.');
                send_robot_command(robot_ip, 'S');
                return_complete = true;
                pause(2);  % Pause before next test
            else
                % Get current orientation
                q = quaternion(rb_data.qw, rb_data.qx, rb_data.qy, rb_data.qz);
                angles = q.EulerAngles('YZX');
                current_angle = rad2deg(angles(1)) + 230;
                if current_angle >= 180
                    current_angle = -360 + current_angle;
                end
                
                % Calculate heading to starting position
                target_angle = rad2deg(atan2(error_vec(1), error_vec(2)));
                heading_error = target_angle - current_angle;
                heading_error = atan2(sin(deg2rad(heading_error)), cos(deg2rad(heading_error)));
                heading_error = rad2deg(heading_error);
                
                % Simple proportional control to return
                forward_speed = min(0.2 * distance_to_start, 60);
                turn_speed = 0.8 * heading_error;
                turn_speed = min(max(turn_speed, -60), 60);
                
                left_speed = round(forward_speed + turn_speed);
                right_speed = round(forward_speed - turn_speed);
                
                % Ensure speeds are within bounds
                left_speed = min(max(left_speed, -100), 100);
                right_speed = min(max(right_speed, -100), 100);
                
                json_command = sprintf('{"N":4,"D1":%d,"D2":%d,"H":"cal"}', ...
                                     left_speed, right_speed);
                send_robot_command(robot_ip, json_command);
                
                fprintf('Returning: Distance=%.1f mm, Heading error=%.1f deg\n', ...
                       distance_to_start, heading_error);
            end
            
            pause(0.05);
        end
    end
end

% =======================================================================
% 5. FINAL RESULTS AND ANALYSIS
% =======================================================================
disp('');
disp('=== CALIBRATION COMPLETE ===');
disp('Motor Speed (PWM) | Measured Velocity (mm/s) | Std Dev (mm/s)');
disp('--------------------------------------------------------------');
for i = 1:length(motor_speeds_to_test)
    fprintf('%16d | %24.1f | %14.1f\n', ...
           motor_speeds_to_test(i), ...
           velocity_data.measured_velocities(i), ...
           velocity_data.std_deviations(i));
end

% Perform linear regression
p = polyfit(motor_speeds_to_test, velocity_data.measured_velocities, 1);
conversion_factor = p(1);
offset = p(2);

disp('');
fprintf('Linear Fit: Velocity (mm/s) = %.2f * Motor_Speed + %.2f\n', ...
       conversion_factor, offset);
fprintf('Conversion Factor: %.2f mm/s per PWM unit\n', conversion_factor);

% Calculate R-squared
y_fit = polyval(p, motor_speeds_to_test);
ss_res = sum((velocity_data.measured_velocities - y_fit).^2);
ss_tot = sum((velocity_data.measured_velocities - mean(velocity_data.measured_velocities)).^2);
r_squared = 1 - (ss_res / ss_tot);
fprintf('R-squared: %.4f\n', r_squared);

% Save calibration data
save('velocity_calibration_data.mat', 'velocity_data', 'conversion_factor', 'offset', 'p');
disp('Calibration data saved to velocity_calibration_data.mat');

% Add text to plot with results
subplot(1, 2, 2);
text_str = sprintf('y = %.2fx + %.2f\nR² = %.4f', conversion_factor, offset, r_squared);
text(motor_speeds_to_test(1), max(velocity_data.measured_velocities)*0.9, ...
     text_str, 'FontSize', 12, 'BackgroundColor', 'w');

% =======================================================================
% 6. CLEANUP
% =======================================================================
disp('Disconnecting NatNet client...');
send_robot_command(robot_ip, 'S');
natnetclient.disconnect;
clear natnetclient;
disp('Calibration complete!');

% =======================================================================
% 7. HELPER FUNCTION
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