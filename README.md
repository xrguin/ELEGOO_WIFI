# ELEGOO_WIFI

# Arduino UNO R4 WiFi Robot Setup Guide

This guide shows how to set up the ELEGOO Smart Robot Car on Arduino UNO R4 WiFi on a new laptop.

## Prerequisites

1. **Arduino IDE 2.x** installed
2. **Arduino UNO R4 WiFi** board
3. **USB-C cable** for programming
4. **Battery pack** for robot power

## Step 1: Install Arduino IDE and Board Support

### 1.1 Install Arduino IDE
- Download from: https://www.arduino.cc/en/software
- Install Arduino IDE 2.x (not 1.x)

### 1.2 Install Arduino UNO R4 Board Package
1. Open Arduino IDE
2. Go to **Tools → Board → Boards Manager**
3. Search for "Arduino UNO R4"
4. Install **"Arduino UNO R4 Boards"** package

## Step 2: Install Required Libraries

Install these libraries via **Tools → Manage Libraries**:

### Required Libraries:
- **FastLED** by Daniel Garcia (latest version)
- **MPU6050** by Electronic Cats (contains I2Cdev)
- **ArduinoJson** by Benoit Blanchon (version 6.x or 7.x)

### Libraries NOT needed:
- ~~NewPing~~ (disabled in code)
- ~~IRremote~~ (disabled for UNO R4 compatibility)
- ~~Servo~~ (disabled for UNO R4 compatibility)

## Step 3: Copy Project Files

Copy these files to your new laptop:

### Main Project Files:
```
SmartRobotCarV4.0_V1_20230201/
├── SmartRobotCarV4.0_V1_20230201.ino    (Main sketch - MODIFIED)
├── ApplicationFunctionSet_xxx0.h         (Header file)
├── ApplicationFunctionSet_xxx0.cpp       (Main functions - MODIFIED)
├── DeviceDriverSet_xxx0.h                (Drivers header - MODIFIED)
├── DeviceDriverSet_xxx0.cpp              (Drivers code - MODIFIED)
├── ArduinoJson-v6.11.1.h                 (JSON library)
├── MPU6050_getdata.h                      (MPU6050 interface)
├── MPU6050_getdata.cpp                    (MPU6050 code - MODIFIED)
└── old_libraries_backup/                 (Backup folder - optional)
```

### Backup Folder (optional):
```
old_libraries_backup/
├── IRremote.cpp
├── IRremote.h  
├── IRremoteInt.h
├── MPU6050.cpp
├── MPU6050.h
└── I2Cdev.cpp
└── I2Cdev.h
```

## Step 4: Key Modifications Made

The following files have been **modified** for UNO R4 WiFi compatibility:

### 4.1 Main Sketch (`SmartRobotCarV4.0_V1_20230201.ino`)
**Added:**
- WiFi support with `#include <WiFiS3.h>`
- WiFi configuration variables
- WiFi connection setup
- TCP server for remote control
- Enhanced serial debugging

**WiFi Configuration (lines 24-25):**
```cpp
const char* ssid = "YOUR_WIFI_SSID";          // CHANGE THIS
const char* password = "YOUR_WIFI_PASSWORD";   // CHANGE THIS
```

### 4.2 Application Functions (`ApplicationFunctionSet_xxx0.cpp`)
**Modifications:**
- IR functionality completely disabled
- Static function declarations fixed
- Watchdog timer compatibility added
- JSON parsing compatibility fixed

### 4.3 Device Drivers (`DeviceDriverSet_xxx0.cpp`)
**Modifications:**
- Servo functionality disabled for UNO R4
- IR functionality disabled
- Watchdog timer compatibility added

### 4.4 Headers (`DeviceDriverSet_xxx0.h`)
**Modifications:**
- Servo library conditionally compiled
- IR library include commented out

### 4.5 MPU6050 Interface (`MPU6050_getdata.cpp`)
**Modifications:**
- Updated library includes to use system libraries

## Step 5: Library Compatibility Fix

If you encounter `BUFFER_LENGTH` errors, add this fix to:
`~/Documents/Arduino/libraries/MPU6050/src/I2Cdev.cpp`

**Add after line 49:**
```cpp
// Compatibility fix for Arduino UNO R4 and newer cores
#ifndef BUFFER_LENGTH
    #ifdef I2C_BUFFER_LENGTH
        #define BUFFER_LENGTH I2C_BUFFER_LENGTH
    #else
        #define BUFFER_LENGTH 32  // Default buffer size
    #endif
#endif
```

## Step 6: Upload and Configure

### 6.1 Board Settings
- **Board**: Arduino UNO R4 WiFi
- **Port**: Select your UNO R4 port
- **Programmer**: Default

### 6.2 WiFi Configuration
1. Edit lines 24-25 in the main `.ino` file:
   ```cpp
   const char* ssid = "YourWiFiName";
   const char* password = "YourWiFiPassword";
   ```

### 6.3 Upload Code
1. Connect Arduino via USB-C
2. Select correct board and port
3. Click **Upload**

## Step 7: Testing

### 7.1 Serial Monitor Test
1. Open **Tools → Serial Monitor**
2. Set baud rate to **9600**
3. Set line ending to **"Both NL & CR"**

**Expected output:**
```
MPU6050_chip_id: 52
Connecting to WiFi network: YourWiFiName
...........
WiFi connected successfully!
IP address: 192.168.1.XXX
Robot ready for TCP commands on port 80
```

### 7.2 Motor Control Test
**Forward:**
```json
{"N":3,"D1":1,"D2":150,"H":"test"}
```

**Stop:**
```json
{"N":3,"D1":0,"D2":0,"H":"test"}
```

## Step 8: Remote Control from Laptop

### Option 1: Simple TCP Commands
```bash
echo "FORWARD" | nc 192.168.1.XXX 80
echo "STOP" | nc 192.168.1.XXX 80
```

### Option 2: Python Script
```python
import socket

def send_command(ip, command):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((ip, 80))
    sock.send((command + '\n').encode())
    response = sock.recv(1024).decode()
    sock.close()
    return response

# Usage
robot_ip = "192.168.1.XXX"  # Use actual IP from Serial Monitor
send_command(robot_ip, "FORWARD")
send_command(robot_ip, "STOP")
```

## Available Commands

### WiFi Commands (simple):
- `FORWARD` or `F` - Move forward
- `BACKWARD` or `B` - Move backward
- `LEFT` or `L` - Turn left  
- `RIGHT` or `R` - Turn right
- `STOP` or `S` - Stop

### Serial/JSON Commands:
- `{"N":3,"D1":1,"D2":150,"H":"test"}` - Forward
- `{"N":3,"D1":2,"D2":150,"H":"test"}` - Backward
- `{"N":3,"D1":3,"D2":150,"H":"test"}` - Left
- `{"N":3,"D1":4,"D2":150,"H":"test"}` - Right
- `{"N":3,"D1":0,"D2":0,"H":"test"}` - Stop

## Features Status

### ✅ Working Features:
- Basic motor control (forward, backward, left, right, stop)
- MPU6050 gyroscope integration
- WiFi TCP server
- Serial communication
- JSON command parsing

### ❌ Disabled Features:
- IR remote control (not compatible with UNO R4)
- Servo motor control (library not compatible yet)
- Ultrasonic sensor (can be re-enabled if needed)

## Troubleshooting

### WiFi IP shows 0.0.0.0
- Check WiFi credentials
- Router may have DHCP issues
- Try restarting router

### Motors don't move
- Ensure battery pack is connected and charged
- USB power is insufficient for motors
- Check motor wiring connections

### Compilation errors
- Verify all required libraries are installed
- Check that Arduino UNO R4 board package is installed
- Make sure I2Cdev compatibility fix is applied

### Serial Monitor shows nothing
- Check baud rate is set to 9600
- Try pressing reset button on Arduino
- Verify USB connection

## File Checksums (Optional)

For verification, these are the key modified files:
- `SmartRobotCarV4.0_V1_20230201.ino` - Contains WiFi setup
- `ApplicationFunctionSet_xxx0.cpp` - IR disabled, compatibility fixes
- `DeviceDriverSet_xxx0.cpp` - Servo/IR disabled
- `DeviceDriverSet_xxx0.h` - Library includes modified

---

**Last Updated**: August 2025  
**Compatible with**: Arduino UNO R4 WiFi, Arduino IDE 2.x
