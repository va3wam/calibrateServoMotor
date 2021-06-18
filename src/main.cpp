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

// quick and dirty easily typed debug commands
   #define spr(x) Serial.print(x)
   #define sprs(x) Serial.print(x); Serial.print(" ");
   #define spr2(x,y) Serial.print(x); Serial.print(y)
   #define spl(x) Serial.println(x)
   #define sptv(x,y) Serial.print(x);Serial.print(y)
   #define sptvl(x,y) Serial.print(x);Serial.println(y)
   #define sp Serial.print(" ");
   #define nl Serial.println();

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

// global variables and storage

// arrays to hold the definition of a "flow" of leg movements. These movements are accumulated by a series 
// of MQTT flow commands, and then executed by the flow_go command
// Each index value represents one position in the series of movements

int f_count = 0;              // number of entries currently in the flow arrays
float f_hip[100];              // hip angle of this position, in degrees
float f_knee[100];
float f_ankle[100];
float f_msecs[100];            // duration of this movement, in millis. basically, speed
int f_style[100];              // type of motion. initially, just linear angle changes
//  1 basic linear change from before angle to after angle for each joint
// -1 linear increments along the straight line joining the before and after positions

bool f_flowing = false;       // are we executing the flow motion
int f_active = 0;             // which index in flow arrays we're currently executing
int f_nextTime = 0;           // upcoming millis() timestamp when we need to do flow processing
float f_frame = 0;            // frame number, up to f_framesPerPosn, between positions in a flow
float f_deltaHip = 0;         // global to kep the compiler happy
float f_deltaKnee = 0;
float f_deltaAnkle = 0;
float f_framesPerPosn = 40;    // 20 seemed a bit jerky. maybe an MQTT command to control it? part of flow?
float f_cx, f_cy, f_cz;        // coordinates for a converted position. see anglesToCoords()
float f_angH, f_angK, f_angA;  // angles for a converted position. f_ah means angle for hip. see coordsToAngles()
float f_tx, f_ty, f_tz, f_th, f_tk, f_ta;  // temporary variables for (x,y,z) and hip,knee,ankle


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

int32_t mapDegToPWM(float degrees, float centerDeg)
// translate a desired servo position expressed in degrees to a PWM number to feed the servo
// if degrees is on a scale from 0 to 180, the center position is 90 degrees, specified in centerDeg
// if degrees is on a scale from -90 to +90, the center position is 0, which goes in centerDeg
// the equivalent PWM value is returned in integer parameter outPWM
// 
// using a horn with 24 positions allows positioning +/- 15 degrees
// using a 25 poistion horn with reversing allows an accuracy of +/- 7.5 degrees
//
// At this level of accuracy, we can treat the current servo motors as identical,
// all with North = 300, East = 115, and West = 486
//
// The servos aren't perfectly linear, and the PWM value may be off by as much as 3.9,
// which is equivalent to a maximum error of less than 2 degrees
{
   // range check the desired degrees value
   if(degrees < centerDeg - 90) {degrees = centerDeg - 90;};
   if(degrees > centerDeg + 90) {degrees = centerDeg + 90;};

   // formula fits a line to two measured data points (x=degrees, y=PWM) with the points
   //   selected to minimize overall errors: (24,160) (166,460)
   // formula is based on  y = M * x + b
   //    where M is the slope, (delta Y)/(delta X)   for 2 selected points
   //    b is the y intercept, derived by substituting a selected point into above formula after slope is known
   
   return (degrees - (centerDeg-90)) * 300 / 142 + 109.3;

} // mapDegToPWM

//  anglesToCoords converts a position expressed in hip, knee and ankle angles in degrees
//  to coordinates (x, y, z)
// ******** Doug: use main.h to move this routine after do_flow
// due to challenges of scope of returned values in arguments, return is to global variables

void anglesToCoords(float hip, float knee, float ankle)     // values are in degrees
// converted coordinates are returned in global variables f_cx, f_cy and f_cz
{
   f_cx = (2.5 + 7.5*cos(knee/180*PI)+11*sin((ankle-knee+17)/180*PI))*cos(hip/180*PI);
   f_cy = -7.5*sin(knee/180*PI)-11*cos((ankle-knee+17)/180*PI );
   f_cz = sin(hip/180*PI)*(2.5+7.5*cos(knee/180*PI)+11*sin((ankle-knee+17)/180*PI));
} // void anglesToCoords

void coordsToAngles(float x, float y, float z)    
// converted values are returned in global variables f_angH, f_angK, f_angA
{
   f_angH = atan(z/x) / PI * 180;    // the hip angle is the easy one.

   // now reduce to a 2D problem by rotating leg into xy plane (around y axis)
   // resulting in new x coordinate: Ux. ( Uy stays same as original y, and new Uz = 0)
   // using standard formula for rotating a 2D vector with angle opposite to hip angle...
   float f_Ux = (x*cos(-f_angH/180*PI) - z*sin(-f_angH/180*PI));
   // spreadsheet: =(E7*COS(-I7/180*PI()) - G7*SIN(-I7/180*PI()))
spr(" Ux= "); spl(f_Ux);
   // next we deduce where the ankle has rotated to in the xy plane
   // the scary math is explained in another document, but here's a summary:
   // -ankle lies on a circle with radius 7.5 centred on knee servo
   // -ankle also lies on a circle with radius 11, centred on the toe = (x,y,z)
   // this provides 2 equations in 2 unknowns (Ax, Ay), but they're quadratic and have
   // 2 solutions. (this makes sense, because those 2 circles intersect in 2 places)
   // reformulate the equations into a single quadratic equation in one variable,
   // apply the quadratic formula, and be clever in selecting the right solution.
   //
   // A quadratic equation has the form A*x^2 + B*x + C = 0
   // the quadratic formula is: x = ( -B +/- SQRT(B*B - 4*a)) / (2*A)
   //   (the +/- choice is what gives 2 solutions )
   // time to calculate the quadratic coefficients A, B, and C

   float f_QA = 1+(25-20*f_Ux+4*f_Ux*f_Ux)/(4*y*y);
   float f_QB = -5+(10*(-71+f_Ux*f_Ux+y*y) -4*((-71+f_Ux*f_Ux+y*y)*f_Ux))/(4*y*y);
   float f_QC = -50+( (-71.0+f_Ux*f_Ux+y*y)*(-71.0+f_Ux*f_Ux+y*y)) / (4*y*y);
spr("Quad coeffs: "); spr(f_QA); sp; spr(f_QB); sp; spl(f_QC);

   // here comes the quadratic formula, choosing +/- based on the sign of y
   float f_Ax, f_Ay;            // coordinates of ankle, rotated into xy plane
   if(y<0)   {f_Ax = (-1*f_QB + sqrt(f_QB*f_QB-4*f_QA*f_QC)) /(2*f_QA);}
   if(y>=0)  {f_Ax = (-1*f_QB - sqrt(f_QB*f_QB-4*f_QA*f_QC)) /(2*f_QA);}
spr("Ax= "); spl(f_Ax);

   // get y by substituting x into a previous equation, again dependent on sign of y
    if(y>0)   {f_Ay = -1*((-71+f_Ux*f_Ux+y*y) - f_Ax*(-5+2*f_Ux))/(2*y);}
    if(y<=0)  {f_Ay =    ((-71+f_Ux*f_Ux+y*y) - f_Ax*(-5+2*f_Ux))/(2*y);}
spr("Ay= "); spl(f_Ay);

    // now that we know where the ankle is, we can finally work on the angles

    // the math isn't quite right yet, and there are some anomalies when:
    //  y>0  i.e. the toe is higher than the knee
    //  x<0  i.e. the toe is under the bot's body.
    // avoid these cases until I can figure out the math

    f_angK = -1* asin( f_Ay / 7.5) / PI * 180;
    f_angA = asin( (f_Ux - f_Ax)/11) / PI * 180 + f_angK - 17;

   

    


} // void coordsToAngles

/**
 * @brief Process the incoming command.
 * =================================================================================*/
bool processCmd(String payload)
{
   String ucPayload = format.stringToUpper(payload);
   //  generalize command parsing to allow 1 - 20 comma separated strings, including the cmd

   String arg[20];              // arg[0] = cmd, arg[1] = 1st argument, arg[2] = second ...
   int argN = 0;                // argument number that we're working on
   int argStart = 0;            // character number where current argument starts
   int argEnd = ucPayload.indexOf(",",argStart);  // position of comma at end of cmd
   while(argEnd >= 0)           // .indexOf returns -1 if no string found
   {  arg[argN] = ucPayload.substring(argStart,argEnd);  // extract the current argument
      argN ++ ;                 // advance thr argument counter
      argStart = argEnd + 1;    // next arg starts after previous arg's delimiting comma
      argEnd = ucPayload.indexOf(",",argStart);  // find next arg's delimiting comma
   }           
   // last argument had no comma delimiter, so grab it
   arg[argN] = ucPayload.substring(argStart,argEnd);
   // argN ends up as a count of the number of arguments, excluding the command

   String cmd = arg[0];          // first comma separated value in payload is the command

   // If user wants to move one of the servo motors.
   if(cmd == "SERVO_POS") 
   {
      uint8_t servoNumber = arg[1].toInt();       // PWM identifier number for the servo
      uint16_t servoPosition = arg[2].toInt();    // PWM value to feed the servo
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
      uint8_t servoNumber = arg[1].toInt();
      uint16_t servoPosition = arg[2].toInt();
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
      uint8_t walkActive = arg[1].toInt(); // 0 = stand still, 1 = walk 
      uint16_t walkStyle = arg[2].toInt(); // 0 = basic cadence, 1 = inverted kinematics
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
            if(walkStyle == 1)
            {
               Serial.println("INVERTED KINEMATICS.");
            } // if walkstyle = 1
            else
               if(walkStyle == 2)
                  {
                     Serial.println("Controlled Servo Speed.");    
                  } // if walkStyle = 2
      } // if
      else
      {
         Serial.println("<processCmd> Stop walking.");
         walkingState.cadencePeriod = STOP; // Set cadence to a walking gate.
         walkingState.walkFlag = false;
      } // else
      return true;
   } // if cmd = walk

   if(cmd == "GOTO_ANGLES" || cmd == "GA")
   // format: goto_angles,<hip angle>,<knee angle>,<ankle angle>
   //       with all angles in degrees, center (north) = 0
   // example: ga,0,0,0    would put robot in normal neutral stance
   {
      Serial.println("start goto_angles command");
 
      // move the servos in parallel at top speed to desired angles
      f_th = arg[1].toFloat();  // compiler won't allow direct substitution below
      f_tk = arg[2].toFloat();
      f_ta = arg[3].toFloat();
      
      pwm.setPWM(servoMotor[1].driverPort, SERVO_START_TICK, mapDegToPWM(f_th, 0)); // Hip
      pwm.setPWM(servoMotor[2].driverPort, SERVO_START_TICK, mapDegToPWM(f_tk, 0)); // Knee
      pwm.setPWM(servoMotor[3].driverPort, SERVO_START_TICK, mapDegToPWM(f_ta, 0)); // Ankle 

      anglesToCoords(f_th, f_tk, f_ta);
      spr(" coords: "); spr(f_cx); sp; spr(f_cy); sp; spl(f_cz);

      // worst case is moving 90 degrees at .17 sec per 60 degrees, so ...
      delay(510);  // wait for moves to complete
      return true;
   } // if cmd = goto_angles


   if(cmd == "GOTO_COORDS" || cmd == "GC")
   // format: goto_coords,<x value>,<y value>,<z value>
   //     for now, avoid y>0 and x<0 until I can fix some math
   // example: gc, 13.22, -10.52,0    would put robot in normal neutral stance
   {
      Serial.println("start goto_coords command");
 
      // convert coordinates given into angles in degrees
      f_tx = arg[1].toFloat();   // compiler won't allow direct substitution below
      f_ty = arg[2].toFloat();
      f_tz = arg[3].toFloat();

      coordsToAngles(f_tx, f_ty, f_tz);  
spr(" args: "); spr(f_tx); sp; spr(f_ty); sp; spl(f_tz);      
      // angle values in degrees, are returned in globals f_angH, f_angK and f_angA

      // move the servos in parallel at top speed to these angles
      
/* test first - there are still bugs in the math
      pwm.setPWM(servoMotor[1].driverPort, SERVO_START_TICK, mapDegToPWM(f_angH, 0)); // Hip
      pwm.setPWM(servoMotor[2].driverPort, SERVO_START_TICK, mapDegToPWM(f_angK, 0)); // Knee
      pwm.setPWM(servoMotor[3].driverPort, SERVO_START_TICK, mapDegToPWM(f_angA, 0)); // Ankle 
*/
      spr(" angles: "); spr(f_angH); sp; spr(f_angK); sp; spl(f_angA);

      // worst case is moving 90 degrees at .17 sec per 60 degrees, so ...
      delay(510);  // wait for moves to complete
      return true;
   } // if cmd = goto_coords



// FLOW command for storing next position in a smooth motion flow between multiple positions
// the flow command builds arrays describing the desired movement
// the flow_go command starts the movement and controls repetitions, resets, etc

   if(cmd == "FLOW" || cmd == "FL")
   // format: flow,<hip angle>,<knee angle>,<ankle angle>, <duration in msec>, <style>
   //       with all angles in degrees, center (north) = 0
   // action: - save this position in the flow arrays, to be part of the motion initiated by flow_go command
   //         - duration (ignored for first position in a flow) is elapsed time to get to this position, in milliseconds
   //           (it basically controls the speed of movement)
   //  style: - 1 is linear transition from start fgangle to end angle, for each joint
   //         - 2 is linear progress along a line from starting position (equivalent (x,y,z)) to end position
   //         - style is ignored for first position
   // example: fl,0,0,0,99,1           would put robot in normal neutral stance
   //          fl,0,-45,45,1000,1      then move smoothly to "peeing on a tree" stance

   {
      Serial.println("start flow command");
 
      // copy position description from MQTT flow command to flow arrays
      // f_count starts at 0, which we use to store first position
      f_hip[f_count] = arg[1].toFloat();
      f_knee[f_count] = arg[2].toFloat();
      f_ankle[f_count] = arg[3].toFloat();
      f_msecs[f_count] = arg[4].toFloat();
      f_style[f_count] = arg[5].toInt();

      f_count ++;             // advance to next entry in flow arrays

      return true;
   } // if cmd = flow

// FLOW_GO command to start up motion as previously defined in the flow arrays using the FLOW command
// the FLOW command builds arrays describing the desired movement
// the FLOW_GO command starts the movement and controls repetitions, resets, etc
//
// format: FLOW_GO, <action>,<cycles>
// action: 1 - start the flow currently defined in the flow arrays
//         0 - reset the current flow, and empty the flow arrays
// cycles: number of times to repeat the flow, 0 or 1 mean once

   if(cmd == "FLOW_GO" || cmd == "FG")
   {
      int f_action = arg[1].toInt();      // decode the first argument - either start flow, or reset it
      if(f_action == 0)                   // zero means reset
      {  f_flowing = false;
         f_count = 0;
         f_active = 0;
         return true;
      }
      else
      {  if(f_action == 1)                // 1 means start up the currently define flow
         {  if(f_count == 0)              // is there a flow defined to run?
            {  Serial.println("<flow_go>: tried to run flow, but none defined");
               return false;
            }
            else
            {  f_flowing = true;         // enable timer driven movement
               f_active = 0;             // start at beginning of flow arrays
               return true;              // actual movement is in loop
            }
         } // if action = 1
         else
         {   Serial.println("<flow_go>: invalid action in MQTT flow_go command");
             return false;
         }
      } // if action = 0 else
 
   return true;
   } // if cmd = flow_go


   // If the command sent is unrecognized.
   Serial.println("<processCmd> Warning - unrecognized command."); 
   return false;
} // processCmd()


// do_flow is triggered by the MQTT flow_go command, which sets f_flowing to true
// f_active is initially zero, which initiates various setup activities on first do_flow entry
// in general case, do_flow executes a small servo movement on all 3 servos, at a calculated time interval

void do_flow()          // called from loop if there's a flow executing that needs attention
{
   float f_frameHip, f_frameKnee, f_frameAnkle;

   if(f_active > 0)     // we're past initialization, and working thru angle changes, a frame at a time
   /*  Need a diagram to visualize frames between positions

                    f_active  f_frame  flow arrays
   1st position         1        1      [0] +
      (50 ms)           1        1
      frame 1           1       1>2
      (50 ms)           1        2
      frame 2           1       2>3
    ...
      frame 19          1       19>20
      (50 ms)           1        20
    frame 20 = 2nd pos  2       20>1     [1] +

   */
   {
      if(millis() >= f_nextTime )       // if we've waited until next frame time
      {  
         f_frameHip = f_hip[f_active-1] + (f_frame/f_framesPerPosn) * f_deltaHip;   // figure servo positions
         f_frameKnee = f_knee[f_active-1] + (f_frame/f_framesPerPosn) * f_deltaKnee;
         f_frameAnkle = f_ankle[f_active-1] + (f_frame/f_framesPerPosn) * f_deltaAnkle;

         // move servo's a fraction of the way to next position
         pwm.setPWM(servoMotor[1].driverPort, SERVO_START_TICK, mapDegToPWM(f_frameHip, 0)); // Hip
         pwm.setPWM(servoMotor[2].driverPort, SERVO_START_TICK, mapDegToPWM(f_frameKnee, 0)); // Knee
         pwm.setPWM(servoMotor[3].driverPort, SERVO_START_TICK, mapDegToPWM(f_frameAnkle, 0)); // Ankle 
 
//////////// sprs(f_active);sprs(f_frame);  sprs(f_frameHip); sprs(f_frameKnee); spl(f_frameAnkle);

         f_frame = f_frame + 1 ;           // on to next frame within this position
//////////// spr2("f_frame=",f_frame); spr2(",  f_active=",f_active); nl;

         f_nextTime = millis() + int(f_msecs[1] / f_framesPerPosn +.5);
///////////// sprs("times/ac>0: "); sprs(millis()); spl(f_nextTime);
         if(f_frame > f_framesPerPosn)       // did we finish all frame for this position?
         {
            // yup, so we must be sitting at the next position. reorganize for next set of frames
            f_active ++ ;
            if(f_active < f_count )
            {                          // haven't run out of positions yet, so do frames out to the next one
               f_deltaHip = f_hip[f_active] - f_hip[f_active-1];   // figure the angle to be travelled for hip to next position
               f_deltaKnee = f_knee[f_active] - f_knee[f_active-1];
               f_deltaAnkle = f_ankle[f_active] - f_ankle[f_active-1];
////////////////sprs("deltas-2: "); sprs(f_deltaHip); sprs(f_deltaKnee); spl(f_deltaAnkle);               

               f_frame = 1;     // starting a new set of frames
//////////////// spl("just reset f_frame");
               f_nextTime = millis() + int(f_msecs[f_active] / f_framesPerPosn +.5);   //get speed info from next position

            }
            else    // ran out of positions. do we need to do more cycles?
            {
                  // cycles not implemented. stop after first
                  f_nextTime = 0;         // stop any further frame processing from moving servos
                  f_flowing = false;      // exit from flow processing
            }

         } // if f_frame > f_framesPerPosn
      } //if millis > f_nextTime
   } // if f_active > 0

   if(f_active == 0 )         // if this is first call to do_flow after MQTT flow_go command...
   {
      // move the servos in parallel at top speed to angles in first array entry
      pwm.setPWM(servoMotor[1].driverPort, SERVO_START_TICK, mapDegToPWM(f_hip[0], 0)); // Hip
      pwm.setPWM(servoMotor[2].driverPort, SERVO_START_TICK, mapDegToPWM(f_knee[0], 0)); // Knee
      pwm.setPWM(servoMotor[3].driverPort, SERVO_START_TICK, mapDegToPWM(f_ankle[0], 0)); // Ankle 
      delay(510);          // worst case delay to let servos move
///////////////// sptv("f_hip[0] ",f_hip[0]); sp; spr(f_knee[0]); sp; spl(f_ankle[0]);
///////////////// sptv("f_hip[1] ",f_hip[1]); sp; spr(f_knee[1]); sp; spl(f_ankle[1]);
      
      f_deltaHip = f_hip[1] - f_hip[0];   // figure the angle to be travelled for hip to next position
      f_deltaKnee = f_knee[1] - f_knee[0];
      f_deltaAnkle = f_ankle[1] - f_ankle[0];

///////////////// sprs("deltas: "); sprs(f_deltaHip); sprs(f_deltaKnee); spl(f_deltaAnkle);

      // we're setting servos at f_framesPerPosn frames between positions.
      // calculate initial time delay until 1st reposition, in rounded integer milliseconds
      // and schedule next flow processing
      f_nextTime = millis() + int(f_msecs[1] / f_framesPerPosn +.5);
      sprs("times: "); sprs(millis()); spl(f_nextTime);
      f_frame = 1;      // frame number we'll do next
      f_active = 1;     // we're now working towards the position in index 1 of the flow arrays
   } // if f_active = 0

}// void do_flow

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

   #include <servoSettings.cpp>

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

   // set up for flow processing
   f_count = 0;         // start with no entries in the flow arrays
   f_flowing = false;   // we're not currently executing a flow
   f_active = 0;        // the next index in the flow arrays to execute is 0, the first one

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
      } // if timer < millis()
  } // if walkflag = true

  if(f_flowing == true)     // are we executing a predefined flow between positions?
  {
     do_flow();             // yes. caclulate and do next servo commands 
  }
} // loop()