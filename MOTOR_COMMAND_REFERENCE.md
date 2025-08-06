# Motor Command Reference Guide

## JSON Command Format
```json
{"N":1,"D1":1,"D2":150,"D3":1,"H":"test"}
```

## Parameter Definitions

### N - Command Mode
- `N:1` = Individual motor control
- `N:2` = Car control (time-limited)
- `N:3` = Car control (no time limit)
- `N:4` = Motor speed control (both motors)

### D1 - Motor Selection (for N:1)
- `D1:0` = Both motors
- `D1:1` = Left motor only (Motor A)
- `D1:2` = Right motor only (Motor B)

### D2 - Speed
- Range: `0-255`
- `0` = Stop
- `150` = Medium speed
- `255` = Maximum speed

### D3 - Direction (for N:1)
- `D3:0` = Stop
- `D3:1` = Forward
- `D3:2` = Backward

### H - Command Header
- Any string for command identification
- Returns `{H_ok}` in response

## Motor Mapping

| Motor | Code | Physical Location |
|-------|------|------------------|
| L1 | Motor A (Left) | Left front wheel |
| R1 | Motor B (Right) | Right front wheel |
| L2 | Motor A (Left) | Left rear wheel |
| R2 | Motor B (Right) | Right rear wheel |

## Example Commands

### Individual Motor Control (N:1)

**Both motors forward:**
```json
{"N":1,"D1":0,"D2":150,"D3":1,"H":"test"}
```

**Left motor only forward:**
```json
{"N":1,"D1":1,"D2":150,"D3":1,"H":"test"}
```

**Right motor only forward:**
```json
{"N":1,"D1":2,"D2":150,"D3":1,"H":"test"}
```

**Stop all motors:**
```json
{"N":1,"D1":0,"D2":0,"D3":0,"H":"test"}
```

### Car Movement Control (N:3)

**Forward:**
```json
{"N":3,"D1":1,"D2":150,"H":"test"}
```

**Backward:**
```json
{"N":3,"D1":2,"D2":150,"H":"test"}
```

**Turn Left:**
```json
{"N":3,"D1":3,"D2":150,"H":"test"}
```

**Turn Right:**
```json
{"N":3,"D1":4,"D2":150,"H":"test"}
```

**Stop:**
```json
{"N":3,"D1":0,"D2":0,"H":"test"}
```

## Code Locations

### Command Parsing
- **File:** `ApplicationFunctionSet_xxx0.cpp`
- **Lines:** 1826-1835
- **Function:** Serial command parser in `ApplicationFunctionSet_SerialPortDataAnalysis()`

### Motor Control Implementation
- **File:** `ApplicationFunctionSet_xxx0.cpp`
- **Lines:** 1037-1115
- **Function:** `CMD_MotorControl_xxx0()`

### Key Code Snippet
```cpp
case 1: /*<Command：N 1> motor control mode */
  Application_SmartRobotCarxxx0.Functional_Mode = CMD_MotorControl;
  CMD_is_MotorSelection = doc["D1"];    // Which motor
  CMD_is_MotorSpeed = doc["D2"];        // Speed (0-255)
  CMD_is_MotorDirection = doc["D3"];    // Direction
```

## Motor Behavior Table

| Command | L1 | R1 | L2 | R2 | Result |
|---------|----|----|----|----|--------|
| `{"N":1,"D1":1,"D2":150,"D3":1,"H":"test"}` | F | S | F | S | Left motors forward |
| `{"N":1,"D1":2,"D2":150,"D3":1,"H":"test"}` | S | F | S | F | Right motors forward |
| `{"N":1,"D1":0,"D2":150,"D3":1,"H":"test"}` | F | F | F | F | All forward (should move straight) |
| `{"N":1,"D1":0,"D2":0,"H":"test"}` | S | S | S | S | Stop all |

Where: F = Forward, S = Stop

## Troubleshooting

### Robot Spins Instead of Moving Forward
- One motor may be wired backwards
- Motor A and Motor B may be swapped
- Check physical motor connections

### Commands Not Working
- Verify battery pack is connected
- Check serial monitor shows `{test_ok}` response
- Ensure correct JSON format with all quotes

### WiFi Commands
Use the same JSON format but send via TCP connection on port 80 to the robot's IP address.