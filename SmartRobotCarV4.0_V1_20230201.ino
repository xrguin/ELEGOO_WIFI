// #include <NewPing.h> // Not actually used - removed for UNO R4 compatibility

/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2020-12-18 14:14:35
 * @LastEditors: Changhua
 * @Description: Smart Robot Car V4.0 - Modified for Arduino UNO R4 WiFi
 * @FilePath: 
 */

// WiFi support for Arduino UNO R4 WiFi
#include <WiFiS3.h>

// Conditional compilation for watchdog timer support
#if defined(__AVR__)
  #include <avr/wdt.h>
  #define WDT_SUPPORTED
#endif

#include "ApplicationFunctionSet_xxx0.h"

// WiFi configuration - CHANGE THESE TO YOUR NETWORK
const char* ssid = "discover";          // Replace with your WiFi name
const char* password = "00000000";   // Replace with your WiFi password

WiFiServer server(80);  // TCP server on port 80
bool wifiConnected = false;

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);  // Initialize Serial first
  delay(1000);         // Wait for Serial to initialize
  
  Application_FunctionSet.ApplicationFunctionSet_Init();
  
  #ifdef WDT_SUPPORTED
    wdt_enable(WDTO_2S);
  #endif
  
  // Initialize WiFi connection
  Serial.print("Connecting to WiFi network: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  
  // Wait for connection
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
    
    // Print status for debugging
    if (attempts % 10 == 0) {
      Serial.print(" Status: ");
      Serial.println(WiFi.status());
    }
  }
  
  Serial.println();
  Serial.print("Final WiFi Status: ");
  Serial.println(WiFi.status());
  
  if (WiFi.status() == WL_CONNECTED) {
    // Wait a bit more for DHCP to assign IP
    delay(2000);
    
    IPAddress ip = WiFi.localIP();
    if (ip == IPAddress(0, 0, 0, 0)) {
      Serial.println("DHCP failed - no IP assigned");
      wifiConnected = false;
    } else {
      wifiConnected = true;
      Serial.println("WiFi connected successfully!");
      Serial.print("IP address: ");
      Serial.println(ip);
      Serial.print("Signal strength (RSSI): ");
      Serial.println(WiFi.RSSI());
      Serial.println("Robot ready for TCP commands on port 80");
      
      // Start the server
      server.begin();
    }
  } else {
    Serial.print("WiFi connection failed. Status code: ");
    Serial.println(WiFi.status());
    Serial.println("Using serial commands only.");
    wifiConnected = false;
  }
}

void loop()
{
  //put your main code here, to run repeatedly :
  #ifdef WDT_SUPPORTED
    wdt_reset();
  #endif
  
  // Handle WiFi clients
  handleWiFiClients();
  
  Application_FunctionSet.ApplicationFunctionSet_SensorDataUpdate();
  Application_FunctionSet.ApplicationFunctionSet_KeyCommand();
  Application_FunctionSet.ApplicationFunctionSet_RGB();
  Application_FunctionSet.ApplicationFunctionSet_Follow();
  Application_FunctionSet.ApplicationFunctionSet_Obstacle();
  Application_FunctionSet.ApplicationFunctionSet_Tracking();
  Application_FunctionSet.ApplicationFunctionSet_Rocker();
  Application_FunctionSet.ApplicationFunctionSet_Standby();
  // Application_FunctionSet.ApplicationFunctionSet_IRrecv(); // IR disabled for UNO R4
  Application_FunctionSet.ApplicationFunctionSet_SerialPortDataAnalysis();

  Application_FunctionSet.CMD_ServoControl_xxx0();
  Application_FunctionSet.CMD_MotorControl_xxx0();
  Application_FunctionSet.CMD_CarControlTimeLimit_xxx0();
  Application_FunctionSet.CMD_CarControlNoTimeLimit_xxx0();
  Application_FunctionSet.CMD_MotorControlSpeed_xxx0();
  Application_FunctionSet.CMD_LightingControlTimeLimit_xxx0();
  Application_FunctionSet.CMD_LightingControlNoTimeLimit_xxx0();
  Application_FunctionSet.CMD_ClearAllFunctions_xxx0();
}

// Handle incoming WiFi client connections
void handleWiFiClients() {
  if (!wifiConnected) return;
  
  WiFiClient client = server.available();
  if (client) {
    String currentLine = "";
    String command = "";
    
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        
        if (c == '\n') {
          // Process the command
          if (currentLine.length() > 0) {
            command = currentLine;
            processWiFiCommand(command);
            client.println("OK"); // Send acknowledgment
          }
          break;
        } else if (c != '\r') {
          currentLine += c;
        }
      }
    }
    client.stop();
  }
}

// Process WiFi commands and convert to serial format
void processWiFiCommand(String command) {
  command.trim();
  Serial.print("WiFi Command: ");
  Serial.println(command);
  
  // Convert simple commands to JSON format for existing parser
  String jsonCommand = "";
  
  if (command == "FORWARD" || command == "F") {
    jsonCommand = "{\"N\":1,\"D1\":1,\"D2\":150}";  // Forward at speed 150
  } else if (command == "BACKWARD" || command == "B") {
    jsonCommand = "{\"N\":1,\"D1\":2,\"D2\":150}";  // Backward at speed 150
  } else if (command == "LEFT" || command == "L") {
    jsonCommand = "{\"N\":1,\"D1\":3,\"D2\":150}";  // Left at speed 150
  } else if (command == "RIGHT" || command == "R") {
    jsonCommand = "{\"N\":1,\"D1\":4,\"D2\":150}";  // Right at speed 150
  } else if (command == "STOP" || command == "S") {
    jsonCommand = "{\"N\":1,\"D1\":0,\"D2\":0}";    // Stop
  } else if (command.startsWith("{")) {
    // Already JSON format, use as is
    jsonCommand = command + "}";
  }
  
  // Feed the JSON command to the existing serial parser by simulating serial input
  if (jsonCommand.length() > 0) {
    // Simulate sending the command through serial
    for (int i = 0; i < jsonCommand.length(); i++) {
      // This would need to be fed to the serial parser
      // For now, we'll just print it
      Serial.print("Processing: ");
      Serial.println(jsonCommand);
    }
  }
}
