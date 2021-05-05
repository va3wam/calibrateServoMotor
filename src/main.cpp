/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 8 servos, one after the other on the
  first 8 pins of the PCA9685

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815
  
  These drivers use I2C to communicate, 2 pins are required to  
  interface.

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h> // https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library
#include <aaMqtt.h> // https://github.com/theAgingApprentice/aaMqtt. Store values that persist past reboot.

/**
 * Define global objects.
 * =================================================================================*/
aaMqtt mqtt; // Explain what this object reference is for. 
aaNetwork wifi("calServo"); // Explain what this object reference is for. 
aaFlash flash; // Non-volatile memory management. 
// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
// Pulse width = SERVOMAX - SERVOMIN. This is the time (ms) that the signal is high during one period. 
#define SERVOMIN  205 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  410 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at a freq of 50Hz creating a period of (1/50*1000 = 20ms). 
                      // Can be between 40Hz and 1600Hz. Servo motors tyically use 50Hz.
#define SERVO_POS_90 410 // setPWM tick count for pulse width that causes servo to move to +90 degrees (~2.0ms).
#define SERVO_NEG_90 205 // setPWM tick count for pulse width that causes servo to move to -90 degrees (~1.0ms).
#define SERVO_CENTRE 307 // setPWM tick count for pulse width that causes servo to move to 0 degrees (~1.5ms).
#define SERVO_START_TICK 0 // setPWM tick count for start of pulse width

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

// Followed this tutorial: https://diyi0t.com/servo-motor-tutorial-for-arduino-and-esp8266/
void setup() 
{
   char uniqueName[HOST_NAME_SIZE]; // Character array that holds unique name. 
   char *uniqueNamePtr = uniqueName; // Pointer to unique name character array.
   IPAddress brokerIP(192, 168, 2, 21); // IP address of the MQTT broker.
   setupSerial(); // Set serial baud rate. 
   Serial.println("<setup> Start of setup");
   wifi.connect(); // Connect to a known WiFi network.
   wifi.getUniqueName(uniqueNamePtr);
   Serial.print("<setup> Unique name = ");
   Serial.println(uniqueName);
   wifi.cfgToConsole();
   mqtt.connect(brokerIP, uniqueName);
   bool x = false;
   while(x == false)
   {
      x = mqtt.publishMQTT(HEALTH_MQTT_TOPIC, "This is a test message");
      delay(10);
   } //while     

//   setupSerial(); // Set serial baud rate. 
//   Serial.begin(115200);
//  Serial.println("<setup> Start of setup");
  Serial.println("<setup> Calibrating servo min/max values via MQTT commands");
//  pwm.begin();
  /*
   * In theory the internal oscillator (clock) is 25MHz but it really isn't
   * that precise. You can 'calibrate' this by tweaking this number until
   * you get the PWM update frequency you're expecting!
   * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
   * is used for calculating things like writeMicroseconds()
   * Analog servos run at ~50 Hz updates, It is importaint to use an
   * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
   * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
   *    the I2C PCA9685 chip you are setting the value for.
   * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
   *    expected value (50Hz for most ESCs)
   * Setting the value here is specific to each individual I2C PCA9685 chip and
   * affects the calculations for the PWM update frequency. 
   * Failure to correctly set the int.osc value will cause unexpected PWM results
   */
//   pwm.setOscillatorFrequency(25700500); // Adjusting to hit as close to 50Hz as possible. 
                                         // Using Saleae Logic 8 unit outut ranges from
                                         // 49.72Hz to 50.51Hz with most readings around 50.1Hz. 
                                         // Think of tjs as the fine adjust setting. 
//   pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates. THis is the course adjust.
//   delay(10);
   Serial.println("<setup> End of setup");
}

char input;

uint8_t servonum = 1; // Which servo to control (0-15)
void loop() 
{
/*
   Serial.println("<loop> +90");
   pwm.setPWM(servonum, SERVO_START_TICK, SERVO_POS_90);
   delay(1000);
   Serial.println("<loop> 0");
   pwm.setPWM(servonum, SERVO_START_TICK, SERVO_CENTRE);
   delay(1000);
   Serial.println("<loop> -90");
   pwm.setPWM(servonum, SERVO_START_TICK, SERVO_NEG_90);
   delay(1000);
   Serial.println("<loop> 0");
   pwm.setPWM(servonum, SERVO_START_TICK, SERVO_CENTRE);
   delay(1000);
*/
} // loop()