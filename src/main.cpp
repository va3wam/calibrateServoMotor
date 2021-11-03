/* command structure for 8 dips switches
 sw1 sw2 - selection of joint combination
 --- ---
 off off do nothing, and ignore other switches
 off on  /move hip to selected angle on selected legs
 on  off /move hip and knee to selected angle on selected legs
 on  on  /move hip, knee and ankle to selected angle on selected legs

 sw3 sw4 - selection of angle, applied to all slected servos
 --- ---
 off off /  0  degrees (center position)
 off on  / +45 degrees
 on  off / -45 degrees
 on  on  / -80 degrees

 sw5 through sw8 select which legs are included in the movement
 --------------
 sw5 on  /means include leg 1
 sw6 on  /means include leg 2
 sw7 on  /means include leg 3
 sw8 on  /means include leg 4

*/
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

#define sw_1 13   // gpio # for wire going to dip switch 1
#define sw_2 12   
//#define sw_3 14   // GP14 deprecated in https://diyprojects.io/esp32-how-to-use-gpio-digital-io-arduino-code/
// suspected this to be cause of download problems iff dip switch powrd up
#define sw_3 35
#define sw_4 27
#define sw_5 26
#define sw_6 25
#define sw_7 33
#define sw_8 32

int st1,st2,st3,st4,st5,st6,st7,st8;    // state of the switches. on=1 off=0

void setupSerial()
{
   unsigned long serialBaudRate = 115200; // Speed we want for serial output (bps).
   Serial.begin(serialBaudRate); // Open a serial connection at specified baud rate. 
   while (!Serial); // Wait for Serial port to be ready.
   pinMode(sw_1, INPUT);
   pinMode(sw_2, INPUT);
   pinMode(sw_3, INPUT);
   pinMode(sw_4, INPUT);
   pinMode(sw_5, INPUT);
   pinMode(sw_6, INPUT);
   pinMode(sw_7, INPUT);
   pinMode(sw_8, INPUT);

} //setupSerial()

/**
 * @brief Followed this tutorial: https://diyi0t.com/servo-motor-tutorial-for-arduino-and-esp8266/
 * =================================================================================*/
void readsw()        // read current state of switches
{
   st1 = 0;
   if(digitalRead(sw_1) == HIGH) st1 = 1;
   st2 = 0;
   if(digitalRead(sw_2) == HIGH) st2 = 1;
   st3 = 0;
   if(digitalRead(sw_3) == HIGH) st3 = 1;
   st4 = 0;
   if(digitalRead(sw_4) == HIGH) st4 = 1;
   st5 = 0;
   if(digitalRead(sw_5) == HIGH) st5 = 1;
   st6 = 0;
   if(digitalRead(sw_6) == HIGH) st6 = 1;
   st7 = 0;
   if(digitalRead(sw_7) == HIGH) st7 = 1;
   st8 = 0;
   if(digitalRead(sw_8) == HIGH) st8 = 1;
}

int32_t mapDegToPWM(float degrees, float centerDeg)
{
   const int8_t offset = 90; // Angle offset of motor in neutral position?
   // TODO #26 create constants for all magic numbers in this function. 
   if(degrees < centerDeg - offset) // range check the desired degrees value
   {
      degrees = centerDeg - offset;
   } // if
   if(degrees > centerDeg + offset) // range check the desired degrees value
   {
      degrees = centerDeg + offset;
   } // if
   // formula fits a line to two measured data points (x=degrees, y=PWM) with 
   // the points selected to minimize overall errors: (24,160) (166,460)
   // formula is based on y = M * x + b where M is the slope, 
   // (delta Y)/(delta X) for 2 selected points b is the y intercept, derived 
   // by substituting a selected point into above formula after slope is known.
   return (degrees - (centerDeg - offset)) * 300 / 142 + 109.3;
} // mapDegToPWM()

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
//      pwm.setPWM(13, 0, 300); // Hip
//      pwm.setPWM(14, 0, 300); // Knee
//      pwm.setPWM(15, 0, 300); // Ankle 
   readsw();             // read current vlaue of switches into stat variabls st1,...
 
   Serial.print(st1);      // show switch readings on serial
   Serial.print(st2);
   Serial.print(st3);
   Serial.print(st4);
   Serial.print(st5);
   Serial.print(st6);
   Serial.print(st7);
   Serial.println(st8);

  // act only if command is something other than binary 0
  if(st1 + st2 > 0)
      {  int angle_code = st3 * 2 + st4;   // convert sw3, sw4 from binary to 0 - 3

         float angle;      // decode sw3 and sw4 into desired angle
         if(angle_code == 0) {angle = 0;}   // binary 0 means zero degrees
         if(angle_code == 1) {angle = 45;}  // binary 1 means +45 degrees
         if(angle_code == 2) {angle = -45;} // binary 2 means -45 degrees
         if(angle_code == 3) {angle = -80;} // binary 3 means -80 degrees
         float angle_pwm = mapDegToPWM(angle,0);   // convert to something we can use with servos

   // now move the right combination of hip, knee and ankle,  for each of the selected legs
   // hip is enabled for all joint combos, so do it unconditionally, for selected legs
         Serial.println(angle_pwm);
         if(st5 == 1)  {  pwm.setPWM(13, 0, angle_pwm);}    // if leg 1 was selected,  move Hip
         if(st6 == 1)  {  pwm.setPWM(10, 0, angle_pwm);}    // if leg 2 was selected,  move its Hip
         if(st7 == 1)  {  pwm.setPWM( 7, 0, angle_pwm);}    // if leg 3 was selected,  move its Hip
         if(st8 == 1)  {  pwm.setPWM( 4, 0, angle_pwm);}    // if leg 4 was selected,  move its Hip

         if(st1 == 1)         // if we were told to move knee for selected legs...
         {  if(st5 == 1)  {  pwm.setPWM(14, 0, angle_pwm);}    // if leg 1 was selected,  move its knee
            if(st6 == 1)  {  pwm.setPWM(11, 0, angle_pwm);}    // if leg 2 was selected,  move its knee
            if(st7 == 1)  {  pwm.setPWM( 8, 0, angle_pwm);}    // if leg 3 was selected,  move its knee
            if(st8 == 1)  {  pwm.setPWM( 5, 0, angle_pwm);}    // if leg 4 was selected,  move its knee
         } // if(st1 ==1)
            
         if(st1 + st2 == 3)         // if we were told to move ankle for selected legs...
         {  if(st5 == 1)  {  pwm.setPWM(15, 0, angle_pwm);}    // if leg 1 was selected,  move its ankle
            if(st6 == 1)  {  pwm.setPWM(12, 0, angle_pwm);}    // if leg 2 was selected,  move its ankle
            if(st7 == 1)  {  pwm.setPWM( 9, 0, angle_pwm);}    // if leg 3 was selected,  move its ankle
            if(st8 == 1)  {  pwm.setPWM( 6, 0, angle_pwm);}    // if leg 4 was selected,  move its ankle
         } // if(st1 + st2 ==3)
         
   } // if(st1 + st2 > 0)
   delay (1000);    // wait 1 second between actions to avoid pounding the servos with changes
   // then start top of loop again

} // loop()