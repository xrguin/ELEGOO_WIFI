% =========================================================================
% simple_motor_test.m
% 
% Simple interactive test for ELEGOO robot motor commands with plotting
% Select mode, test for 2 seconds, then auto-stop
% =========================================================================

clear; clc; close all;

% =========================================================================
% CONFIGURATION - CHANGE THIS TO YOUR ROBOT IP
% =========================================================================
robot_ip = "192.168.1.84";  % <-- Change to your robot's IP
test_duration = 2;  % seconds
plot_update_rate = 0.1;  % Update plot every 0.1 seconds

fprintf('====================================\n');
fprintf('  ELEGOO Motor Test with Plotting\n');
fprintf('====================================\n');
fprintf('Robot IP: %s\n', robot_ip);
fprintf('Test Duration: %d seconds\n\n', test_duration);

% =========================================================================
% INITIALIZE PLOT
% =========================================================================
fig = figure('Position', [100, 100, 1200, 400], 'Name', 'Motor Speed Monitor');

% Left subplot for motor speeds over time
ax1 = subplot(1, 2, 1);
hold on; grid on;
title('Motor Speeds Over Time');
xlabel('Time (s)'); ylabel('Speed (PWM)');
ylim([-260, 260]);
h_left = plot(0, 0, 'b-', 'LineWidth', 2, 'DisplayName', 'Left Motor');
h_right = plot(0, 0, 'r-', 'LineWidth', 2, 'DisplayName', 'Right Motor');
yline(0, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
legend('Location', 'best');

% Right subplot for current speeds (bar chart)
ax2 = subplot(1, 2, 2);
hold on; grid on;
title('Current Motor Speeds');
xlabel('Motor'); ylabel('Speed (PWM)');
ylim([-260, 260]);
h_bars = bar(categorical({'Left', 'Right'}), [0, 0], 'FaceColor', 'flat');
h_bars.CData = [0 0 1; 1 0 0];  % Blue for left, red for right

% Initialize data storage
time_data = [];
left_speed_data = [];
right_speed_data = [];

% =========================================================================
% MAIN TEST LOOP
% =========================================================================
while true
    fprintf('\n--- Select Test Mode ---\n');
    fprintf('1. Forward\n');
    fprintf('2. Backward\n');
    fprintf('3. Turn Left\n');
    fprintf('4. Turn Right\n');
    fprintf('5. Spin Left\n');
    fprintf('6. Spin Right\n');
    fprintf('7. Custom Speed\n');
    fprintf('8. Clear Plot\n');
    fprintf('0. Exit\n');
    
    choice = input('Enter choice (0-8): ');
    
    if choice == 0
        fprintf('\nExiting...\n');
        break;
    end
    
    if choice == 8
        % Clear plot data
        time_data = [];
        left_speed_data = [];
        right_speed_data = [];
        set(h_left, 'XData', [], 'YData', []);
        set(h_right, 'XData', [], 'YData', []);
        h_bars.YData = [0, 0];
        fprintf('Plot cleared!\n');
        continue;
    end
    
    % Default speed and motor values
    speed = 60;
    left_motor = 0;
    right_motor = 0;
    
    % Get speed for movements
    if choice >= 1 && choice <= 6
        fprintf('\nSelect Speed:\n');
        fprintf('1. Slow (30)\n');
        fprintf('2. Medium (60)\n');
        fprintf('3. Fast (100)\n');
        speed_choice = input('Enter speed (1-3): ');
        
        switch speed_choice
            case 1
                speed = 30;
            case 2
                speed = 60;
            case 3
                speed = 100;
            otherwise
                speed = 60;
        end
    end
    
    % Execute command based on choice
    switch choice
        case 1  % Forward
            fprintf('\n>> FORWARD at speed %d\n', speed);
            left_motor = speed;
            right_motor = speed;
            json_cmd = sprintf('{"N":1,"D1":0,"D2":%d, "D3":1}', speed);
            
        case 2  % Backward (using N:1 command)
            fprintf('\n>> BACKWARD at speed %d\n', speed);
            % For visualization, show as negative
            left_motor = -speed;
            right_motor = -speed;
            % N:1 with D1=0 (both motors), D2=speed, D3=2 (backward)
            json_cmd = sprintf('{"N":1,"D1":0,"D2":%d,"D3":2}', speed);
            
        case 3  % Turn Left (arc - left slower)
            fprintf('\n>> TURN LEFT at speed %d (CORRECTED)\n', speed);
            left_motor = max(0, speed - 30);  % Left wheel slower
            right_motor = speed;
            % CORRECTED: D1=right motor, D2=left motor
            json_cmd = sprintf('{"N":4,"D1":%d,"D2":%d}', right_motor, left_motor);
            
        case 4  % Turn Right (arc - right slower)
            fprintf('\n>> TURN RIGHT at speed %d (CORRECTED)\n', speed);
            left_motor = speed;
            right_motor = max(0, speed - 30);  % Right wheel slower
            % CORRECTED: D1=right motor, D2=left motor
            json_cmd = sprintf('{"N":4,"D1":%d,"D2":%d}', right_motor, left_motor);
            
        case 5  % Spin Left (true spin - left backward, right forward)
            fprintf('\n>> SPIN LEFT at speed %d (using N:1 commands)\n', speed);
            left_motor = -speed;  % For visualization
            right_motor = speed;  % For visualization
            % We'll send two N:1 commands - need special handling
            json_cmd = 'SPIN_LEFT';  % Special flag
            
        case 6  % Spin Right (true spin - left forward, right backward)
            fprintf('\n>> SPIN RIGHT at speed %d (using N:1 commands)\n', speed);
            left_motor = speed;   % For visualization
            right_motor = -speed; % For visualization
            % We'll send two N:1 commands - need special handling
            json_cmd = 'SPIN_RIGHT';  % Special flag
            
        case 7  % Custom Speed
            fprintf('\nEnter custom speeds:\n');
            left_motor = input('Left motor (0-255): ');
            right_motor = input('Right motor (0-255): ');
            
            % Clamp to valid range
            left_motor = max(0, min(255, left_motor));
            right_motor = max(0, min(255, right_motor));
            
            fprintf('\n>> CUSTOM: Left=%d, Right=%d\n', left_motor, right_motor);
            json_cmd = sprintf('{"N":4,"D1":%d,"D2":%d}', left_motor, right_motor);
            
        otherwise
            fprintf('Invalid choice!\n');
            continue;
    end
    
    % Send command(s)
    if strcmp(json_cmd, 'SPIN_LEFT')

        json_cmd = sprintf('{"N":3,"D1":1,"D2":%d}', speed);
        response = send_robot_command(robot_ip, json_cmd);
        fprintf('  Responses: %s\n', response);

    elseif strcmp(json_cmd, 'SPIN_RIGHT')

        json_cmd = sprintf('{"N":3,"D1":2,"D2":%d}', speed);
        response = send_robot_command(robot_ip, json_cmd);
        fprintf('  Responses: %s\n', response);
    else
        % Normal single command
        fprintf('Sending: %s\n', json_cmd);
        response = send_robot_command(robot_ip, json_cmd);
        fprintf('Response: %s\n', response);
    end
    
    % Get start time for this test
    if isempty(time_data)
        test_start_time = 0;
    else
        test_start_time = time_data(end) + 0.5;  % Add gap between tests
    end
    
    % Run for test duration with live plotting
    fprintf('Running for %d seconds...\n', test_duration);
    test_timer = tic;
    
    while toc(test_timer) < test_duration
        current_time = test_start_time + toc(test_timer);
        
        % Add data points
        time_data(end+1) = current_time;
        left_speed_data(end+1) = left_motor;
        right_speed_data(end+1) = right_motor;
        
        % Update plots
        set(h_left, 'XData', time_data, 'YData', left_speed_data);
        set(h_right, 'XData', time_data, 'YData', right_speed_data);
        h_bars.YData = [left_motor, right_motor];
        
        % Update x-axis limits to show last 10 seconds
        if current_time > 10
            xlim(ax1, [current_time-10, current_time+1]);
        else
            xlim(ax1, [0, max(10, current_time+1)]);
        end
        
        drawnow;
        
        % Show progress
        elapsed = toc(test_timer);
        if mod(floor(elapsed), 1) == 0 && elapsed < test_duration
            fprintf('  %.0f...', elapsed);
        end
        
        pause(plot_update_rate);
    end
    fprintf(' Done!\n');
    
    % Auto stop
    fprintf('Stopping...\n');
    stop_cmd = '{"N":4,"D1":0,"D2":0}';
    send_robot_command(robot_ip, stop_cmd);
    
    % Add stop state to plot
    current_time = test_start_time + test_duration;
    time_data(end+1) = current_time;
    left_speed_data(end+1) = 0;
    right_speed_data(end+1) = 0;
    
    % Update plots with stop state
    set(h_left, 'XData', time_data, 'YData', left_speed_data);
    set(h_right, 'XData', time_data, 'YData', right_speed_data);
    h_bars.YData = [0, 0];
    drawnow;
    
    fprintf('Test complete!\n');
end

fprintf('\nFinal stop command sent.\n');
send_robot_command(robot_ip, '{"N":4,"D1":0,"D2":0}');
close(fig);
fprintf('Goodbye!\n');

% =========================================================================
% HELPER FUNCTION
% =========================================================================
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