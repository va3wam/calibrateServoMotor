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
#include <Wire.h> // I2C communication.
#include <Adafruit_PWMServoDriver.h> // https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library.
#include <aaMqtt.h> // https://github.com/theAgingApprentice/aaMqtt. Store values that persist past reboot.

/**
 * Define global objects.
 * =================================================================================*/
aaMqtt mqtt; // Communicating with the MQTT broker. 
aaNetwork wifi("calServo"); // Explain what this object reference is for. 
aaFlash flash; // Non-volatile memory management. 
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire); // I2C address 0x40 (default) on Wire1.

#define SERVO_FREQ 50 // Analog servos run at a freq of 50Hz creating a period of (1/50*1000 = 20ms). 
                      // Can be between 40Hz and 1600Hz. Servo motors tyically use 50Hz.
#define SERVO_START_TICK 0 // setPWM tick count for start of pulse width
#define PIN_SERVO_FEEDBACK 0 // Connect orange PWM pin, 0 = first on first block. Monitor PWM from servo driver. 

// Structure for servo motors
typedef struct
{
   int16_t min;
   int16_t mid;
   int16_t max;
} servoMotorStruct;
servoMotorStruct servoMotor[4];

const byte interruptPin = 14; // GPIO14 is physical pin 11 on 30 pin Devkit V1 board.
volatile int interruptCounter = 0;
int numberOfInterrupts = 0;
volatile long last, now; 
 
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

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
 * @brief Followed this tutorial: https://diyi0t.com/servo-motor-tutorial-for-arduino-and-esp8266/
 * =================================================================================*/
void setup() 
{
   char uniqueName[HOST_NAME_SIZE]; // Character array that holds unique name. 
   char *uniqueNamePtr = uniqueName; // Pointer to unique name character array.
   uint32_t oscFreq; // Oscillator freq of the PCA9865 servo motor driver. Board specific.
   IPAddress brokerIP(192, 168, 2, 21); // IP address of the MQTT broker.
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
      Serial.println("<setup> Andrews MCU specific settings");
      servoMotor[0].max = 110; // Servo #1
      servoMotor[0].mid = 310;
      servoMotor[0].min = 510;
      servoMotor[1].max = 110; // Servo #2
      servoMotor[1].mid = 300;
      servoMotor[1].min = 495;
      servoMotor[2].max = 125; // Servo #3
      servoMotor[2].mid = 330;
      servoMotor[2].min = 525;
      servoMotor[3].max = 120; // Servo #4
      servoMotor[3].mid = 315;
      servoMotor[3].min = 510;
      oscFreq = 25700500; // Oscillator. Make function to set automatically.        
   } // if
   else // Doug's MCU
   {
      Serial.print("<setup> Dougs MCU specific settings");
      servoMotor[0].max = 110; // Servo #1
      servoMotor[0].mid = 310;
      servoMotor[0].min = 510;
      servoMotor[1].max = 110; // Servo #2
      servoMotor[1].mid = 300;
      servoMotor[1].min = 495;
      servoMotor[2].max = 125; // Servo #3
      servoMotor[2].mid = 330;
      servoMotor[2].min = 525;
      servoMotor[3].max = 120; // Servo #4
      servoMotor[3].mid = 315;
      servoMotor[3].min = 510;
      oscFreq = 25700500; // Oscillator. Make function to set automatically.       
   } //else
   if(oscFreq == 25700500)
   {
      Serial.println("<setup> Oscillator freq matches.");
   } // if
   else
   {
      Serial.print("<setup> Oscillator freq DOES NOT match.");
   } // else
   wifi.cfgToConsole();
   mqtt.connect(brokerIP, uniqueName);
   bool x = false;
   while(x == false)
   {
      x = mqtt.publishMQTT(HEALTH_MQTT_TOPIC, "This is a test message");
      delay(10);
   } //while     
   Serial.println("<setup> Calibrating servo min/max values via MQTT commands");
   pwm.begin();
   pwm.setOscillatorFrequency(oscFreq); // Make function to adjust thiis until the PWM 
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

uint8_t servonum = 1; // Which servo to control (0-15)

/**
 * @brief Main loop.
 * =================================================================================*/
void loop() 
{
   String cmd = mqtt.getCmd();
   if(cmd != "")
   {
      Serial.print("<loop> Process command: ");
      Serial.println(cmd.toInt());
      pwm.setPWM(servonum, SERVO_START_TICK, cmd.toInt());
   } //if
  if(interruptCounter > 0)
  {
      portENTER_CRITICAL(&mux);
      interruptCounter--;
      portEXIT_CRITICAL(&mux);
      numberOfInterrupts++;
      Serial.print("Interrupt count = ");
      Serial.print(numberOfInterrupts);
      Serial.print(" Last diff = ");
      Serial.print(now-last);
      Serial.print(" ms or freq of ");
      Serial.print(1000/(now-last));
      Serial.println("Hz.");
  } // if
} // loop()