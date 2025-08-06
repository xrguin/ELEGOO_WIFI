/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2020-12-29 16:07:48
 * @LastEditors: Changhua
 * @Description: Smart Robot Car V4.0
 * @FilePath: 
 */
#include "DeviceDriverSet_xxx0.h"
//#include "PinChangeInt.h"

// Conditional compilation for watchdog timer support
#if defined(__AVR__)
  #include <avr/wdt.h>
  #define WDT_SUPPORTED
#endif
static void
delay_xxx(uint16_t _ms)
{
  #ifdef WDT_SUPPORTED
    wdt_reset();
  #endif
  for (unsigned long i = 0; i < _ms; i++)
  {
    delay(1);
  }
}
/*RBG LED*/
static uint32_t Color(uint8_t r, uint8_t g, uint8_t b)
{
  return (((uint32_t)r << 16) | ((uint32_t)g << 8) | b);
}
void DeviceDriverSet_RBGLED::DeviceDriverSet_RBGLED_xxx(uint16_t Duration, uint8_t Traversal_Number, CRGB colour)
{
  if (NUM_LEDS < Traversal_Number)
  {
    Traversal_Number = NUM_LEDS;
  }
  for (int Number = 0; Number < Traversal_Number; Number++)
  {
    leds[Number] = colour;
    FastLED.show();
    delay_xxx(Duration);
  }
}
void DeviceDriverSet_RBGLED::DeviceDriverSet_RBGLED_Init(uint8_t set_Brightness)
{
  FastLED.addLeds<NEOPIXEL, PIN_RBGLED>(leds, NUM_LEDS);
  FastLED.setBrightness(set_Brightness);
}
#if _Test_DeviceDriverSet
void DeviceDriverSet_RBGLED::DeviceDriverSet_RBGLED_Test(void)
{
  leds[0] = CRGB::White;
  FastLED.show();
  delay_xxx(50);
  leds[1] = CRGB::Red;
  FastLED.show();
  delay_xxx(50);
  DeviceDriverSet_RBGLED_xxx(50 /*Duration*/, 5 /*Traversal_Number*/, CRGB::Black);
}
#endif

void DeviceDriverSet_RBGLED::DeviceDriverSet_RBGLED_Color(uint8_t LED_s, uint8_t r, uint8_t g, uint8_t b)
{
  if (LED_s > NUM_LEDS)
    return;
  if (LED_s == NUM_LEDS)
  {
    FastLED.showColor(Color(r, g, b));
  }
  else
  {
    leds[LED_s] = Color(r, g, b);
  }
  FastLED.show();
}

/*Key*/
uint8_t DeviceDriverSet_Key::keyValue = 0;

static void attachPinChangeInterrupt_GetKeyValue(void)
{
  DeviceDriverSet_Key Key;
  static uint32_t keyValue_time = 0;
  static uint8_t keyValue_temp = 0;
  if ((millis() - keyValue_time) > 500)
  {
    keyValue_temp++;
    keyValue_time = millis();
    if (keyValue_temp > keyValue_Max)
    {
      keyValue_temp = 0;
    }
    Key.keyValue = keyValue_temp;
  }
}
void DeviceDriverSet_Key::DeviceDriverSet_Key_Init(void)
{
  pinMode(PIN_Key, INPUT_PULLUP);
  //attachPinChangeInterrupt(PIN_Key, attachPinChangeInterrupt_GetKeyValue, FALLING);
  attachInterrupt(0, attachPinChangeInterrupt_GetKeyValue, FALLING);
}

#if _Test_DeviceDriverSet
void DeviceDriverSet_Key::DeviceDriverSet_Key_Test(void)
{
  Serial.println(DeviceDriverSet_Key::keyValue);
}
#endif

void DeviceDriverSet_Key::DeviceDriverSet_key_Get(uint8_t *get_keyValue)
{
  *get_keyValue = keyValue;
}

/*ITR20001 Detection*/
bool DeviceDriverSet_ITR20001::DeviceDriverSet_ITR20001_Init(void)
{
  pinMode(PIN_ITR20001xxxL, INPUT);
  pinMode(PIN_ITR20001xxxM, INPUT);
  pinMode(PIN_ITR20001xxxR, INPUT);
  return false;
}
int DeviceDriverSet_ITR20001::DeviceDriverSet_ITR20001_getAnaloguexxx_L(void)
{
  return analogRead(PIN_ITR20001xxxL);
}
int DeviceDriverSet_ITR20001::DeviceDriverSet_ITR20001_getAnaloguexxx_M(void)
{
  return analogRead(PIN_ITR20001xxxM);
}
int DeviceDriverSet_ITR20001::DeviceDriverSet_ITR20001_getAnaloguexxx_R(void)
{
  return analogRead(PIN_ITR20001xxxR);
}
#if _Test_DeviceDriverSet
void DeviceDriverSet_ITR20001::DeviceDriverSet_ITR20001_Test(void)
{
  Serial.print("\tL=");
  Serial.print(analogRead(PIN_ITR20001xxxL));

  Serial.print("\tM=");
  Serial.print(analogRead(PIN_ITR20001xxxM));

  Serial.print("\tR=");
  Serial.println(analogRead(PIN_ITR20001xxxR));
}
#endif

/*Voltage Detection*/
void DeviceDriverSet_Voltage::DeviceDriverSet_Voltage_Init(void)
{
  pinMode(PIN_Voltage, INPUT);
  //analogReference(INTERNAL);
}
float DeviceDriverSet_Voltage::DeviceDriverSet_Voltage_getAnalogue(void)
{
  //float Voltage = ((analogRead(PIN_Voltage) * 5.00 / 1024) * 7.67); //7.66666=((10 + 1.50) / 1.50)
  float Voltage = (analogRead(PIN_Voltage) * 0.0375);
  Voltage = Voltage + (Voltage * 0.08); //Compensation 8%
  //return (analogRead(PIN_Voltage) * 5.00 / 1024) * ((10 + 1.50) / 1.50); //Read voltage value
  return Voltage;
}

#if _Test_DeviceDriverSet
void DeviceDriverSet_Voltage::DeviceDriverSet_Voltage_Test(void)
{
  //float Voltage = ((analogRead(PIN_Voltage) * 5.00 / 1024) * 7.67); //7.66666=((10 + 1.50) / 1.50)
  float Voltage = (analogRead(PIN_Voltage) * 0.0375); //7.66666=((10 + 1.50) / 1.50)
  Voltage = Voltage + (Voltage * 0.08);               //Compensation 8%
  //Serial.println(analogRead(PIN_Voltage) * 4.97 / 1024);
  Serial.println(Voltage);
}
#endif
/*Motor control*/
void DeviceDriverSet_Motor::DeviceDriverSet_Motor_Init(void)
{
  pinMode(PIN_Motor_PWMA, OUTPUT);
  pinMode(PIN_Motor_PWMB, OUTPUT);
  pinMode(PIN_Motor_AIN_1, OUTPUT);
  pinMode(PIN_Motor_BIN_1, OUTPUT);
  pinMode(PIN_Motor_STBY, OUTPUT);
}

#if _Test_DeviceDriverSet
void DeviceDriverSet_Motor::DeviceDriverSet_Motor_Test(void)
{
  //A...Right
  //B...Left
  digitalWrite(PIN_Motor_STBY, HIGH);

  digitalWrite(PIN_Motor_AIN_1, HIGH);
  analogWrite(PIN_Motor_PWMA, 100);
  digitalWrite(PIN_Motor_BIN_1, HIGH);
  analogWrite(PIN_Motor_PWMB, 100);
  delay_xxx(1000);

  digitalWrite(PIN_Motor_STBY, LOW);
  delay_xxx(1000);
  digitalWrite(PIN_Motor_STBY, HIGH);
  digitalWrite(PIN_Motor_AIN_1, LOW);
  analogWrite(PIN_Motor_PWMA, 100);
  digitalWrite(PIN_Motor_BIN_1, LOW);
  analogWrite(PIN_Motor_PWMB, 100);

  delay_xxx(1000);
}
#endif

/*
 Motor_control：AB / movement direction and speed
*/
void DeviceDriverSet_Motor::DeviceDriverSet_Motor_control(boolean direction_A, uint8_t speed_A, //Group A motor parameters
                                                          boolean direction_B, uint8_t speed_B, //Group B motor parameters
                                                          boolean controlED                     //AB enable setting (true)
                                                          )                                     //Motor control
{

  if (controlED == control_enable) //Enable motot control？
  {
    digitalWrite(PIN_Motor_STBY, HIGH);
    { //A...Right

      switch (direction_A) //movement direction control
      {
      case direction_just:
        digitalWrite(PIN_Motor_AIN_1, HIGH);
        analogWrite(PIN_Motor_PWMA, speed_A);
        break;
      case direction_back:

        digitalWrite(PIN_Motor_AIN_1, LOW);
        analogWrite(PIN_Motor_PWMA, speed_A);
        break;
      case direction_void:
        analogWrite(PIN_Motor_PWMA, 0);
        digitalWrite(PIN_Motor_STBY, LOW);
        break;
      default:
        analogWrite(PIN_Motor_PWMA, 0);
        digitalWrite(PIN_Motor_STBY, LOW);
        break;
      }
    }

    { //B...Left
      switch (direction_B)
      {
      case direction_just:
        digitalWrite(PIN_Motor_BIN_1, HIGH);

        analogWrite(PIN_Motor_PWMB, speed_B);
        break;
      case direction_back:
        digitalWrite(PIN_Motor_BIN_1, LOW);
        analogWrite(PIN_Motor_PWMB, speed_B);
        break;
      case direction_void:
        analogWrite(PIN_Motor_PWMB, 0);
        digitalWrite(PIN_Motor_STBY, LOW);
        break;
      default:
        analogWrite(PIN_Motor_PWMB, 0);
        digitalWrite(PIN_Motor_STBY, LOW);
        break;
      }
    }
  }
  else
  {
    digitalWrite(PIN_Motor_STBY, LOW);
    return;
  }
}

/*ULTRASONIC*/
//#include <NewPing.h>
// NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
void DeviceDriverSet_ULTRASONIC::DeviceDriverSet_ULTRASONIC_Init(void)
{
  pinMode(ECHO_PIN, INPUT); //Ultrasonic module initialization
  pinMode(TRIG_PIN, OUTPUT);
}
void DeviceDriverSet_ULTRASONIC::DeviceDriverSet_ULTRASONIC_Get(uint16_t *ULTRASONIC_Get /*out*/)
{
  unsigned int tempda_x = 0;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  tempda_x = ((unsigned int)pulseIn(ECHO_PIN, HIGH) / 58);
  // *ULTRASONIC_Get = tempda_x;

  if (tempda_x > 150)
  {
    *ULTRASONIC_Get = 150;
  }
  else
  {
    *ULTRASONIC_Get = tempda_x;
  }
  // sonar.ping() / US_ROUNDTRIP_CM; // Send ping, get ping time in microseconds (uS).
}

#if _Test_DeviceDriverSet
void DeviceDriverSet_ULTRASONIC::DeviceDriverSet_ULTRASONIC_Test(void)
{

  unsigned int tempda = 0;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  tempda = ((unsigned int)pulseIn(ECHO_PIN, HIGH) / 58);

  // if (tempda_x > 50)
  // {
  //   tempda_x = 50;
  // }

  // // return tempda;
  // return tempda_x;

  Serial.print("ULTRASONIC=");
  Serial.print(tempda); // Convert ping time to distance and print result (0 = outside set distance range, no ping echo)
  Serial.println("cm");
}

#endif

/*Servo*/
#ifndef SERVO_LIB_DISABLED
Servo myservo; // create servo object to control a servo
#endif
void DeviceDriverSet_Servo::DeviceDriverSet_Servo_Init(unsigned int Position_angle)
{
#ifndef SERVO_LIB_DISABLED
  myservo.attach(PIN_Servo_z, 500, 2400); //500: 0 degree  2400: 180 degree
  myservo.attach(PIN_Servo_z);
  myservo.write(Position_angle); //sets the servo position according to the 90（middle）
  delay_xxx(500);

  myservo.attach(PIN_Servo_y, 500, 2400); //500: 0 degree  2400: 180 degree
  myservo.attach(PIN_Servo_y);
  myservo.write(Position_angle); //sets the servo position according to the 90（middle）
  delay_xxx(500);
  myservo.detach();
#else
  // Servo not supported on Arduino UNO R4 yet
#endif
}
#if _Test_DeviceDriverSet
void DeviceDriverSet_Servo::DeviceDriverSet_Servo_Test(void)
{
  for (;;)
  {
    myservo.attach(PIN_Servo_z);
    myservo.write(180);
    delay_xxx(500);
    myservo.write(0);
    delay_xxx(500);
  }

  // for (uint8_t i = 0; i < 6; i++)
  // {
  //   myservo.write(30 * i);
  //   delay(500);
  // }
  // for (uint8_t i = 6; i > 0; i--)
  // {
  //   myservo.write(30 * i);
  //   delay(500);
  // }

  // myservo.attach(PIN_Servo_y);

  // for (uint8_t i = 0; i < 6; i++)
  // {
  //   myservo.write(30 * i);
  //   delay(500);
  // }
  // for (uint8_t i = 6; i > 0; i--)
  // {
  //   myservo.write(30 * i);
  //   delay(500);
  // }
}
#endif

/*0.17sec/60degree(4.8v)*/
void DeviceDriverSet_Servo::DeviceDriverSet_Servo_control(unsigned int Position_angle)
{
#ifndef SERVO_LIB_DISABLED
  myservo.attach(PIN_Servo_z);
  myservo.write(Position_angle);
  delay_xxx(450);
  myservo.detach();
#else
  // Servo not supported on Arduino UNO R4 yet
#endif
}
//Servo motor control:Servo motor number and position angle
void DeviceDriverSet_Servo::DeviceDriverSet_Servo_controls(uint8_t Servo, unsigned int Position_angle)
{
#ifndef SERVO_LIB_DISABLED
  if (Servo == 1 || Servo == 3) //Servo_z
  {
    if (Position_angle <= 1) //minimum angle control
    {
      Position_angle = 1;
    }
    if (Position_angle >= 17) //maximum angle control
    {
      Position_angle = 17;
    }
    myservo.attach(PIN_Servo_z);
    myservo.write(10 * Position_angle);
    delay_xxx(500);
  }
  if (Servo == 2 || Servo == 3) //Servo_y
  {

    if (Position_angle <= 3) //minimum angle control
    {
      Position_angle = 3;
    }
    if (Position_angle >= 11) //maximum angle control
    {
      Position_angle = 11;
    }
    myservo.attach(PIN_Servo_y);
    myservo.write(10 * Position_angle);
    delay_xxx(500);
  }
  myservo.detach();
#else
  // Servo not supported on Arduino UNO R4 yet
#endif
}

/*IRrecv - DISABLED FOR ARDUINO UNO R4 COMPATIBILITY*/
// IRrecv irrecv(RECV_PIN); //  Create an infrared receive drive object
// decode_results results;  //  Create decoding object
void DeviceDriverSet_IRrecv::DeviceDriverSet_IRrecv_Init(void)
{
  // irrecv.enableIRIn(); //Enable infrared communication NEC
  // IR functionality disabled for Arduino UNO R4 WiFi
}
bool DeviceDriverSet_IRrecv::DeviceDriverSet_IRrecv_Get(uint8_t *IRrecv_Get /*out*/)
{
  // IR functionality disabled for Arduino UNO R4 WiFi
  return false;
}

#if _Test_DeviceDriverSet
void DeviceDriverSet_IRrecv::DeviceDriverSet_IRrecv_Test(void)
{
  // IR functionality disabled for Arduino UNO R4 WiFi
  /*
  if (irrecv.decode(&results))
  {
    Serial.print("IRrecv_Test:");
    Serial.println(results.value);
    irrecv.resume();
  }
  */
}
#endif
