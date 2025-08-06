import socket
    
def send_command(ip, command):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((ip, 80))
    sock.send((command + '\n').encode())
    response = sock.recv(1024).decode()
    sock.close()
    return response
   

robot_ip = "192.168.1.140"
   
# Send commands
send_command(robot_ip, "S")
#send_command(robot_ip, "STOP")