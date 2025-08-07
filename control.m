clear;clc;
% --- Main script ---
% Set the robot's IP address
robot_ip = "192.168.1.140"; % Using string for IP is fine



disp('Sending command: Forward-Curving Left (JSON)');
json_command_left = '{"N":102,"D1":5,"D2":150,"H":"matlab"}';
json_command_right =   '{"N":102,"D1":7,"D2":150,"H":"matlab"}';
response = send_command(robot_ip, json_command_right);
disp(response);
pause(2);

% 5. Stop the robot again
disp('Sending command: Stop');
response = send_command(robot_ip, 'S');
disp(response);

disp('Command sequence finished.');




function response = send_command(ip, command)
    % Create a TCP client object
    t = tcpclient(ip, 80);

    % Convert the command to a character array and append the newline character.
    % This creates a single character vector (e.g., 'F\n') to send.
    data_to_send = [char(command), newline];
    write(t, data_to_send);

    % Read the response from the server. It arrives as a vector of uint8 values.
    response_uint8 = read(t);
    
    % Convert the uint8 vector to a readable character string.
    % The transpose (') is used to make it a row vector.
    if ~isempty(response_uint8)
        response = char(response_uint8');
    else
        response = '(No response from server)';
    end

    % The tcpclient object is automatically closed when it goes out of scope
    % or by using clear t;
end

