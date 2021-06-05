/******************************************************************************
 * @file main.cpp
 *
 * @mainpage the Aging Apprentice calibrating servo motors 
 * 
 * @section intro_sec Introduction
 *
 * This is an Arduino sketch that allows you to send MQTT messages to the 
 * MCU in order to calaibrate your servo motors.
 *
 * @section dependencies Dependencies
 * 
 * This sketch depends on the following libraries:
 * - [Arduino.h](https://github.com/espressif/arduino-esp32). This is the 
 * Arduino core library that comes bundled with PlatformIO.
 * - [Adafruit_PWMServoDriver.h](https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library).
 * This is the library used to control the servo motors via the PCA9865 servo 
 * motor driver.   
 * - [aaMqtt.h](https://github.com/theAgingApprentice/aaMqtt).
 * - [aaNetwork](https://github.com/theAgingApprentice/aaNetwork).
 * - [aaFlash.h](https://github.com/theAgingApprentice/aaFlash).
 *
 * @section author Author(s)
 *
 * Written by Old Squire for the Aging Apprentice.
 *
 * @section license license
 *
 * Copyright 2021 the Aging Apprentice 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to 
 * deal in the Software without restriction, including without limitation the 
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is 
 * furnished to do so, subject to the following conditions: The above copyright
 * notice and this permission notice shall be included in all copies or 
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.  
 *****************************************************************************/
#include <Arduino.h> // Arduino Core for ESP32. Comes with Platform.io
#include <Wire.h> // I2C communication.
#include <Adafruit_PWMServoDriver.h> // https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library.
#include <aaMqtt.h> // https://github.com/theAgingApprentice/aaMqtt. Store values that persist past reboot.
#include <aaFormat.h> // 

/**
 * Define global objects.
 * =================================================================================*/
aaMqtt mqtt; // Communicating with the MQTT broker. 
aaNetwork wifi("calServo"); // Explain what this object reference is for. 
aaFlash flash; // Non-volatile memory management. 
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire); // I2C address 0x40 (default) on Wire1.
aaFormat format;

#define SERVO_FREQ 50 // Analog servos run at a freq of 50Hz creating a period of (1/50*1000 = 20ms). 
                      // Can be between 40Hz and 1600Hz. Servo motors tyically use 50Hz.
#define SERVO_START_TICK 0 // setPWM tick count for start of pulse width
#define PIN_SERVO_FEEDBACK 0 // Connect orange PWM pin, 0 = first on first block. Monitor PWM from servo driver. 
#define SERVO_START_NUM 1 // First servo cnnected to pin 1
#define SERVO_CNT 4 // Number of servos connected
// Structure for servo motors.
typedef struct
{
   String role; // The role this servo fills for the robot.
   String label; // The role this servo fills for the robot.
   int driverPort; // The driver port (0-15) that the servo is connected to.
   long stand; // Position when standing to attention. 
   long step; // Position for full step formward.
   long min; // Minimal value servo can move to.
   long max; // Maximum value servo can move to.
   long east; // Servo is at the 0 position of a 180 degree arch.
   long north; // Servo is at the 90 degree position of a 180 degree arch.
   long west; // Servo is at the 180 degree position of a 180 degree arch.
} servoMotorStruct;
servoMotorStruct servoMotor[SERVO_CNT + 1];

const byte interruptPin = 14; // GPIO14 is physical pin 11 on 30 pin Devkit V1 board.
volatile int interruptCounter = 0; // Number of signals from servo driver not processed.
int numberOfInterrupts = 0; // Total number of servo driver signals in total.
volatile long last, now; // Track freq of servo motorcontroller PWM signal.

// State machine for walking cadence.
enum stepCadenceStates 
{
  STOP = 0, // No movement. 
  CRAWL = 1, // Slow gate.
  WALK = 2, // Normal gate.
  RUN = 3 // Fast gate.
};
enum stepCadenceStates style;

// State machine for step phases.
enum stepPhaseStates 
{
  NEUTRAL = 0, // Leg joint start and end step position.
  LIFT = 1, // 
  STEP = 2, // blink enable
  LOWER = 3 // we want the led to be on for interval
};
 
// Structure for walking.
typedef struct
{
   bool walkFlag = false; // Walking or standing still?
   enum stepCadenceStates cadence; // Walking cadence.
   enum stepPhaseStates phase; // Walking phase.
   int32_t cadencePeriod; // How long between phases in millis.
   unsigned long timer; // Milli count for next action.
} walkingStruct;
walkingStruct walkingState; // Track walking phases
 
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED; // Mutex to prevent ISR and main conflicts

/**
 * @brief ISR for PWM signal from PCA9685.
 * =================================================================================*/
void IRAM_ATTR handleInterrupt() 
{
  portENTER_CRITICAL_ISR(&mux);
  interruptCounter++;
  last = now;
  now = millis();
  portEXIT_CRITICAL_ISR(&mux);
} // handleInterrupt()

/**
 * @brief Initialize the serial output with the specified baud rate measured in bits 
 * per second.
 * =================================================================================*/
void setupSerial()
{
   unsigned long serialBaudRate = 115200; // Speed we want for serial output (bps).
   Serial.begin(serialBaudRate); // Open a serial connection at specified baud rate. 
   while (!Serial); // Wait for Serial port to be ready.
} //setupSerial()

/**
 * @brief Process the incoming command.
 * =================================================================================*/
bool processCmd(String payload)
{
   String ucPayload = format.stringToUpper(payload);
   int firstComma = ucPayload.indexOf(",");
   int secondComma = ucPayload.indexOf(",", firstComma + 1);
   int lenCmd = firstComma;
   int lenArg = secondComma - firstComma - 1;
   int lenVal = ucPayload.length() - secondComma - 1;
   int cmdStart =  0;
   int argStart = firstComma + 1;
   int valStart = secondComma + 1;
   String cmd = ucPayload.substring(cmdStart, cmdStart + lenCmd);
   String arg = ucPayload.substring(argStart, argStart + lenArg);
   String val = ucPayload.substring(valStart, valStart + lenVal);

   Serial.print("<processCmd> Payload length = "); Serial.println(ucPayload.length());
   Serial.print("<processCmd> Received paylod = "); Serial.println(ucPayload);

   Serial.print("<processCmd> First comma = "); Serial.println(firstComma);
   Serial.print("<processCmd> Second comma = "); Serial.println(secondComma);

   Serial.print("<processCmd> cmdStart = "); Serial.println(cmdStart);
   Serial.print("<processCmd> lenCmd = "); Serial.println(lenCmd);
   Serial.print("<processCmd> cmd = "); Serial.println(cmd);

   Serial.print("<processCmd> argStart = "); Serial.println(argStart);
   Serial.print("<processCmd> lenArg = "); Serial.println(lenArg);
   Serial.print("<processCmd> arg = "); Serial.println(arg);

   Serial.print("<processCmd> valStart = "); Serial.println(valStart);
   Serial.print("<processCmd> lenVal = "); Serial.println(lenVal);
   Serial.print("<processCmd> val = "); Serial.println(val);

   // If user wants to move one of the servo motors.
   if(cmd == "SERVO_POS") 
   {
      uint8_t servoNumber = arg.toInt();
      uint16_t servoPosition = val.toInt();
      Serial.print("<processCmd> Move servo number "); 
      Serial.print(servoNumber);
      Serial.print(" to position ");
      Serial.println(servoPosition);
      pwm.setPWM(servoNumber, SERVO_START_TICK, servoPosition);      
      return true;
   } // if

   // Since the servo motors we are using do not move in a linear fasion around their north (centre of range)
   // posiiton we cannot use the Arduino MAP() function as it requires that the servos move linearally. So do not
   // try to issue angle commands using the mapping function but instead set values up for key positions and move
   // using setPWM method.
   if(cmd == "SERVO_ANGLE") 
   {
      uint8_t servoNumber = arg.toInt();
      uint16_t servoPosition = val.toInt();
      long pulseLength = map(servoPosition, 0, 180, servoMotor[servoNumber].west, servoMotor[servoNumber].east);
      Serial.print("<processCmd> Requested degrees = "); 
      Serial.println(servoPosition);
      Serial.print("<processCmd> Servo west mapped to 0 = "); 
      Serial.println(servoMotor[servoNumber].west);
      Serial.print("<processCmd> Servo east mapped to 180 = "); 
      Serial.println(servoMotor[servoNumber].east);
      Serial.print("<processCmd> Move servo number "); 
      Serial.print(servoNumber);
      Serial.print(" to angle ");
      Serial.print(servoPosition);
      Serial.print(" which maps to pulse length ");
      Serial.println(pulseLength);
      pwm.setPWM(servoMotor[servoNumber].driverPort, SERVO_START_TICK, pulseLength);      
      return true;
   } // if

   // If user wants to know the current frequency of the PWM signal from the PCA9685.
   if(cmd == "SERVO_GET_FREQ")
   {
      Serial.print("<processCmd> Interrupt count = ");
      Serial.print(numberOfInterrupts);
      Serial.print(", period = ");
      Serial.print(now - last);
      Serial.print(" ms, freq = ");
      Serial.print(1000 / (now - last));
      Serial.println("Hz.");
      return true;
   } // if

  // If user wants the leg to start walking.
   if(cmd == "WALK")
   {
      uint8_t walkActive = arg.toInt(); // 0 = stand still, 1 = walk 
      uint16_t walkStyle = val.toInt(); // 0 = basic cadence, 1 = inverted kinematics
      Serial.print("<processCmd> Walking activation = ");
      Serial.println(walkActive);
      Serial.print("<processCmd> Walking style = ");
      Serial.println(walkStyle);

      if(walkActive == 1)
      {
         Serial.print("<processCmd> Start walking using style ");
         if(walkStyle == 0)
         {
            Serial.println("BASIC CADENCE.");
            walkingState.phase = NEUTRAL; // Start in neutral position.   
            walkingState.cadencePeriod = WALK; // Set cadence to a walking gate.
            walkingState.walkFlag = true; // Flag us to start walking.
            walkingState.cadencePeriod = 1000; // Millis between cadence phases.
            walkingState.timer = millis(); // Set next phase timer.
         } // if
         else
         {
            Serial.println("INVERTED KINEMETICS.");
         } // else
      } // if
      else
      {
         Serial.println("<processCmd> Stop walking.");
         walkingState.cadencePeriod = STOP; // Set cadence to a walking gate.
         walkingState.walkFlag = false;
      } // else
      return true;
   } // if

   // If the command sent is unrecognized.
   Serial.println("<processCmd> Warning - unrecognized command."); 
   return false;
} // processCmd()

/**
 * @brief Followed this tutorial: https://diyi0t.com/servo-motor-tutorial-for-arduino-and-esp8266/
 * =================================================================================*/
void setup() 
{
   char uniqueName[HOST_NAME_SIZE]; // Character array that holds unique name. 
   char *uniqueNamePtr = uniqueName; // Pointer to unique name character array.
   uint32_t oscFreq; // Oscillator freq of the PCA9865 servo motor driver. Board specific.
   uint8_t brokerIpOctet0; // First octet of broker IP address.
   uint8_t brokerIpOctet1; // Second octet of broker IP address.
   uint8_t brokerIpOctet2; // Third octet of broker IP address.
   uint8_t brokerIpOctet3; // Fourth octet of broker IP address.
   setupSerial(); // Set serial baud rate. 
   Serial.println("<setup> Start of setup");
   wifi.connect(); // Connect to a known WiFi network.
   wifi.getUniqueName(uniqueNamePtr);
   Serial.print("<setup> Unique name = ");
   Serial.println(uniqueName);
   // Stepper motor calibration settings
   // ==================================
   // About MIN/MAX settings
   // Depending on your servo make, the pulse width min and max may vary, you 
   // want these to be as small/large as possible without hitting the hard stop
   // for max range. You'll have to tweak them as necessary to match the servos you
   // have!
   // 
   // About oscillator freq setting
   // =============================
   // In theory the internal oscillator (clock) is 25MHz but it really isn't
   // that precise. You can 'calibrate' this by tweaking this number until
   // you get the PWM update frequency you're expecting!
   // The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
   // is used for calculating things like writeMicroseconds()
   // Analog servos run at ~50 Hz updates, It is importaint to use an
   // oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
   // 1) Attach the oscilloscope to one of the PWM signal pins and ground on
   //    the I2C PCA9685 chip you are setting the value for.
   // 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
   //    expected value (50Hz for most ESCs)
   // Setting the value here is specific to each individual I2C PCA9685 chip and
   // affects the calculations for the PWM update frequency. 
   // Failure to correctly set the int.osc value will cause unexpected PWM results
   if(strcmp(uniqueName, "calServoCC50E394F048") == 0) // Andrew's MCU
   {
      brokerIpOctet0 = 192; // First octet of broker IP address.
      brokerIpOctet1 = 168; // Second octet of broker IP address.
      brokerIpOctet2 = 2; // Third octet of broker IP address.
      brokerIpOctet3 = 21; // Fourth octet of broker IP address.
      Serial.println("<setup> Andrews MCU specific settings");
      oscFreq = 25700500; // PWM output via PWM0 on PCA9685. Make function to set automatically. 
      // LEG #1 ===============================================================
      // Servo #1 - Hip Joint
      servoMotor[1].role = "Hip joint"; // The role this servo fills for the robot.
      servoMotor[1].label = "2"; // The label marked on the servo.
      servoMotor[1].driverPort = 13; // The driver port (0-15) that the servo is connected to.
      servoMotor[1].stand = 300; // Position when standing to attention. 
      servoMotor[1].step = 400; // Position for full step formward.
      servoMotor[1].min = 72; // Minimal value servo can move to.
      servoMotor[1].max = 538; // Maximum value servo can move to.
      servoMotor[1].east = 95; // Servo is at the 0 position of a 180 degree arch.
      servoMotor[1].north = 290; // Servo is at the 90 degree position of a 180 degree arch.
      servoMotor[1].west = 485; // Servo is at the 180 degree position of a 180 degree arch.
      // Servo #2 - Knee Joint
      servoMotor[2].role = "Knee joint"; // The role this servo fills for the robot.
      servoMotor[2].label = "3"; // The label marked on the servo.
      servoMotor[2].driverPort = 14; // The driver port (0-15) that the servo is connected to.
      servoMotor[2].stand = 300; // Position when standing to attention. 
      servoMotor[2].step = 200; // Position for full step formward.
      servoMotor[2].min = 72; // Minimal value servo can move to.
      servoMotor[2].max = 540; // Maximum value servo can move to.
      servoMotor[2].east = 100; // Servo is at the 0 position of a 180 degree arch.
      servoMotor[2].north = 300; // Servo is at the 90 degree position of a 180 degree arch.
      servoMotor[2].west = 500; // Servo is at the 180 degree position of a 180 degree arch.
      // Servo #3 - Ankle Joint
      servoMotor[3].role = "Ankle joint"; // The role this servo fills for the robot.
      servoMotor[3].label = "4"; // The label marked on the servo.
      servoMotor[3].driverPort = 15; // The driver port (0-15) that the servo is connected to.
      servoMotor[3].stand = 300; // Position when standing to attention. 
      servoMotor[3].step = 200; // Position for full step formward.
      servoMotor[3].min = 71; // Minimal value servo can move to.
      servoMotor[3].max = 538; // Maximum value servo can move to.
      servoMotor[3].east = 90; // Servo is at the 0 position of a 180 degree arch.
      servoMotor[3].north = 290; // Servo is at the 90 degree position of a 180 degree arch.
      servoMotor[3].west = 480; // Servo is at the 180 degree position of a 180 degree arch.
      // Servo #4 - Unused
      servoMotor[4].role = "Unused"; // The role this servo fills for the robot.
      servoMotor[4].label = "1"; // The label marked on the servo.
      servoMotor[4].driverPort = 12; // The driver port (0-15) that the servo is connected to.
      servoMotor[4].stand = 270; // Position when standing to attention. 
      servoMotor[4].step = 200; // Position for full step formward.
      servoMotor[4].min = 71; // Minimal value servo can move to.
      servoMotor[4].max = 538; // Maximum value servo can move to.
      servoMotor[4].east = 90; // Servo is at the 0 position of a 180 degree arch.
      servoMotor[4].north = 290; // Servo is at the 90 degree position of a 180 degree arch.
      servoMotor[4].west = 480; // Servo is at the 180 degree position of a 180 degree arch.
   } // if
   else // Doug's MCU. NOTE: Stand and Step values not set yet! Using Andrew's values. 
   {
      brokerIpOctet0 = 192; // First octet of broker IP address.
      brokerIpOctet1 = 168; // Second octet of broker IP address.
      brokerIpOctet2 = 0; // Third octet of broker IP address.
      brokerIpOctet3 = 99; // Fourth octet of broker IP address.
      Serial.print("<setup> Dougs MCU specific settings");
      oscFreq = 25700500; // PWM output via PWM0 on PCA9685. Make function to set automatically. 
      // LEG #1 ===============================================================
      // Servo #1 - Hip Joint
      servoMotor[1].role = "Hip joint"; // The role this servo fills for the robot.
      servoMotor[1].label = "D1"; // The label marked on the servo.
      servoMotor[1].driverPort = 1; // The driver port (0-15) that the servo is connected to.
      servoMotor[1].stand = 300; // Position when standing to attention. 
      servoMotor[1].step = 200; // Position for full step formward.
      servoMotor[1].min = 72; // Minimal value servo can move to.
      servoMotor[1].max = 540; // Maximum value servo can move to.
      servoMotor[1].east = 120; // Servo is at the 0 position of a 180 degree arch.
      servoMotor[1].north = 300; // Servo is at the 90 degree position of a 180 degree arch.
      servoMotor[1].west = 505; // Servo is at the 180 degree position of a 180 degree arch.
      // Servo #2 - Knee Joint
      servoMotor[2].role = "Knee joint"; // The role this servo fills for the robot.
      servoMotor[2].label = "D2"; // The label marked on the servo.
      servoMotor[2].driverPort = 2; // The driver port (0-15) that the servo is connected to.
      servoMotor[2].stand = 510; // Position when standing to attention. 
      servoMotor[2].step = 450; // Position for full step formward.
      servoMotor[2].min = 71; // Minimal value servo can move to.
      servoMotor[2].max = 530; // Maximum value servo can move to.
      servoMotor[2].east = 117; // Servo is at the 0 position of a 180 degree arch.
      servoMotor[2].north = 300; // Servo is at the 90 degree position of a 180 degree arch.
      servoMotor[2].west = 510; // Servo is at the 180 degree position of a 180 degree arch.
      // Servo #3 - Ankle Joint
      servoMotor[3].role = "Ankle joint"; // The role this servo fills for the robot.
      servoMotor[3].label = "D3"; // The label marked on the servo.
      servoMotor[3].driverPort = 3; // The driver port (0-15) that the servo is connected to.
      servoMotor[3].stand = 270; // Position when standing to attention. 
      servoMotor[3].step = 200; // Position for full step formward.
      servoMotor[3].min = 72; // Minimal value servo can move to.
      servoMotor[3].max = 539; // Maximum value servo can move to.
      servoMotor[3].east = 110; // Servo is at the 0 position of a 180 degree arch.
      servoMotor[3].north = 300; // Servo is at the 90 degree position of a 180 degree arch.
      servoMotor[3].west = 500; // Servo is at the 180 degree position of a 180 degree arch.
      // Servo #4 - Unused
      servoMotor[4].role = "Unused"; // The role this servo fills for the robot.
      servoMotor[4].label = "D4"; // The label marked on the servo.
      servoMotor[4].driverPort = 4; // The driver port (0-15) that the servo is connected to.
      servoMotor[4].stand = 270; // Position when standing to attention. 
      servoMotor[4].step = 200; // Position for full step formward.
      servoMotor[4].min = 72; // Minimal value servo can move to.
      servoMotor[4].max = 539; // Maximum value servo can move to.
      servoMotor[4].east = 107; // Servo is at the 0 position of a 180 degree arch.
      servoMotor[4].north = 300; // Servo is at the 90 degree position of a 180 degree arch.
      servoMotor[4].west = 505; // Servo is at the 180 degree position of a 180 degree arch.
   } //else

   for (int i = SERVO_START_NUM; i < SERVO_CNT; i++)
   {
      Serial.print("<setup> servo"); Serial.print(i);
      Serial.print(" Min = "); Serial.print(servoMotor[i].min);
      Serial.print(" Max = "); Serial.println(servoMotor[i].max);
   }  // for 

   if(oscFreq == 25700500) // Should use library call here to get actual value.
   {
      Serial.println("<setup> Oscillator freq matches.");
   } // if
   else
   {
      Serial.println("<setup> Oscillator freq DOES NOT match.");
   } // else
   wifi.cfgToConsole();
   IPAddress brokerIP(brokerIpOctet0, brokerIpOctet1, brokerIpOctet2, brokerIpOctet3); // IP address of the MQTT broker.
   mqtt.connect(brokerIP, uniqueName);
   bool x = false;

   while(x == false)
   {
      x = mqtt.publishMQTT(HEALTH_MQTT_TOPIC, "This is a test message");
      delay(10);
   } //while     
   pwm.begin();
   pwm.setOscillatorFrequency(oscFreq); // Make function to adjust this until the PWM 
                                        // signal on pin 11 to hit as close to 50Hz as 
                                        // possible. Using Saleae Logic 8 unit outut 
                                        // ranges from 49.72Hz to 50.51Hz with most 
                                        // readings around 50.1Hz. 
   pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates.  
   pwm.setPWM(PIN_SERVO_FEEDBACK,0,2048); // half of time high, half of time low
   delay(10);
   now = millis();
   pinMode(interruptPin, INPUT);
   attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, RISING);
   Serial.println("<setup> End of setup");
} // setup()

//uint8_t servonum = 1; // Which servo to control (0-15)

/**
 * @brief Main loop.
 * =================================================================================*/
void loop() 
{
   // Check for incoming MQTT commands and process as needed.
   String cmd = mqtt.getCmd();
   if(cmd != "")
   {
      Serial.print("<loop> cmd = ");
      Serial.println(cmd);
      bool allIsWell = processCmd(cmd);
      if(allIsWell)
      {
         Serial.println("<loop> All went well.");
      } // if 
      else
      {
         Serial.println("<loop> Something went wrong.");
      } // if 
   } // if
  
  // Check if external PWM interrupt has occured and track with counters.
  if(interruptCounter > 0)
  {
      portENTER_CRITICAL(&mux);
      interruptCounter--;
      portEXIT_CRITICAL(&mux);
      numberOfInterrupts++;
  } // if

  // Walking logic. This is a very basic and non-useful gate used to prove out the jig for 1 leg.
  if(walkingState.walkFlag == true) //  Are we supposed to be walking?
  {
      if(walkingState.timer <= millis()) // Time to act?
      {
         switch(walkingState.phase) 
         {
            case NEUTRAL:
               Serial.println("<loop> Neutral position for all 3 motors.");         
               pwm.setPWM(servoMotor[1].driverPort, SERVO_START_TICK, servoMotor[1].stand); // Hip
               pwm.setPWM(servoMotor[2].driverPort, SERVO_START_TICK, servoMotor[2].stand); // Knee
               pwm.setPWM(servoMotor[3].driverPort, SERVO_START_TICK, servoMotor[3].stand); // Ankle 
               walkingState.phase = LIFT; 
               break;
            case LIFT:
               Serial.println("<loop> Lift position for all 3 motors.");         
               pwm.setPWM(servoMotor[1].driverPort, SERVO_START_TICK, servoMotor[1].stand); // Hip
               pwm.setPWM(servoMotor[2].driverPort, SERVO_START_TICK, servoMotor[2].step); // Knee
               pwm.setPWM(servoMotor[3].driverPort, SERVO_START_TICK, servoMotor[3].step); // Ankle 
               walkingState.phase = STEP; 
               break;
            case STEP:
               Serial.println("<loop> Step position for all 3 motors.");         
               pwm.setPWM(servoMotor[1].driverPort, SERVO_START_TICK, servoMotor[1].step); // Hip
               pwm.setPWM(servoMotor[2].driverPort, SERVO_START_TICK, servoMotor[2].step); // Knee
               pwm.setPWM(servoMotor[3].driverPort, SERVO_START_TICK, servoMotor[3].step); // Ankle
               walkingState.phase = LOWER; 
               break;
            case LOWER:
               Serial.println("<loop> Lower position for all 3 motors.");
               pwm.setPWM(servoMotor[3].driverPort, SERVO_START_TICK, servoMotor[3].stand); // Ankle                      
               pwm.setPWM(servoMotor[1].driverPort, SERVO_START_TICK, servoMotor[1].step); // Hip
               pwm.setPWM(servoMotor[2].driverPort, SERVO_START_TICK, servoMotor[2].stand); // Knee
               walkingState.phase = NEUTRAL; 
               break;
            default:
               Serial.println("<loop> Error - invalid step state");
               walkingState.phase = NEUTRAL; 
               break;
         } // switch   
         walkingState.timer = millis() + walkingState.cadencePeriod;      
      } // if
  } // if
} // loop()