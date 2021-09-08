
#include <Arduino.h> // Arduino Core for ESP32. Comes with Platform.io
#include <Wire.h> // I2C communication.
#include <Adafruit_PWMServoDriver.h> // https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library.
#include <aaFormat.h> // 
#include <main.h>     // routine call sequence definitions to allow putting them anywhere in source file

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire); // I2C address 0x40 (default) on Wire1.
aaFormat format;

#define SERVO_FREQ 50 // Analog servos run at a freq of 50Hz creating a period of (1/50*1000 = 20ms). 
                      // Can be between 40Hz and 1600Hz. Servo motors tyically use 50Hz.
#define SERVO_START_TICK 0 // setPWM tick count for start of pulse width
#define PIN_SERVO_FEEDBACK 0 // Connect orange PWM pin, 0 = first on first block. Monitor PWM from servo driver. 
#define SERVO_START_NUM 1 // First servo cnnected to pin 1
#define SERVO_CNT 4 // Number of servos connected
#define LED 2   // gpio for onboad red led nxt to power led

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
   setupSerial(); // Set serial baud rate. 
   Serial.println("<setup> Start of setup");
 
   pwm.begin();
   pwm.setOscillatorFrequency(25700500); 
   pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates.  
   pwm.setPWM(PIN_SERVO_FEEDBACK,0,2048); // half of time high, half of time low
   delay(10);
 
   Serial.println("<setup> End of setup");
} // setup()

void loop() 
{
      pwm.setPWM(13, 0, 300); // Hip
      pwm.setPWM(14, 0, 300); // Knee
      pwm.setPWM(15, 0, 300); // Ankle 
      delay(5000);      // paus 5 sconds before moving to (0,0,0) again

} // loop()