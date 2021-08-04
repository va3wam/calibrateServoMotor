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
#include <main.h>     // routine call sequence definitions to allow putting them anywhere in source file

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

#define LED 2   // gpio for onboad red led nxt to power led

// quick and dirty easily typed debug commands
   #define spr(x) Serial.print(x);
   #define sprs(x) Serial.print(x); Serial.print(" ");
   #define spr2(x,y) Serial.print(x); Serial.print(" "); Serial.print(y);
   #define spr2l(x,y) Serial.print(x); Serial.print(" "); Serial.println(y);
   #define spr3(x,y,z) Serial.print(x); Serial.print(" "); Serial.print(y); Serial.print(" "); Serial.print(z);
   #define spr3l(x,y,z) Serial.print(x); Serial.print(" "); Serial.print(y); Serial.print(" "); Serial.println(z);
   #define spl(x) Serial.println(x);
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
float f_x_hip[100];           // hip angle of this position, in degrees, or x coordinate if style=2
float f_y_knee[100];
float f_z_ankle[100];
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
float f_msecPerFrame_default = 25;      // default time in milliseconds between frames, i.e. servo moves
float f_msecPerFrame = 25;     // time in milliseconds between frames, i.e. servo moves, as set by FG command
float f_framesPerPosn = 0;     // this is now calculated on the fly from position duration and f_msecPerFrame
float f_cx, f_cy, f_cz;        // coordinates for a converted position. see anglesToCoords()
float f_angH, f_angK, f_angA;  // angles for a converted position. f_ah means angle for hip. see coordsToAngles()
float f_tx, f_ty, f_tz, f_th, f_tk, f_ta;  // temporary variables for (x,y,z) and hip,knee,ankle
float f_Ux, f_Uy;              // toe position when rotated into xy plane
float f_lastHa, f_lastKa, f_lastAa;  //remember last position as angles in flow to calculate next deltas
float f_lastHc, f_lastKc, f_lastAc;  //remember last position as coords in flow to calculate next deltas
float f_test;                       // for .h file testing
int f_cycles;                    // number of times to repeat flow. set in flow_go command

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
   f_Ux = (x*cos(-f_angH/180*PI) - z*sin(-f_angH/180*PI));
   f_Uy = y;   // the rotation doesn't change the y value
   // spreadsheet: =(E7*COS(-I7/180*PI()) - G7*SIN(-I7/180*PI()))
// spr("`coordsToAngles args: "); spr(x); sp; spr(y); sp; spr(z); nl;
//spr("`  f_angH = "); spr(f_angH); nl;
//spr("`  f_Ux = "); spl(f_Ux);
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
//spr("`Quad coeffs: "); spr(f_QA); sp; spr(f_QB); sp; spl(f_QC);

   // here comes the quadratic formula, which produces two possible solutions due to the +/-
   //  we'll comput them both, then make a choice based on robot limitations
   //  generally, we want the one where the ankle is the highest, maximizing Ay
   float f_Ax, f_Ay, f_AxPlus, f_AxMinus, f_AyPlus, f_AyMinus;    // coordinates of ankle, rotated into xy plane
   float f_determinant;      // detrminant in quadratic solution - must be >= 0
   f_determinant = round((f_QB*f_QB-4*f_QA*f_QC)*10000)/10000;
//spr("`f_determinant= "); spl(f_determinant);
   if(f_determinant < 0) { spl("`========= negative determinant =======");}
   f_AxPlus  = (-1*f_QB + sqrt(f_determinant)) /(2*f_QA);
   f_AxMinus = (-1*f_QB - sqrt(f_determinant)) /(2*f_QA);

   // substituting back in previous equation to get correcponding Ay valus
   f_AyPlus  = ((-71 +f_Ux*f_Ux +f_Uy*f_Uy) -f_AxPlus *(-5+2*f_Ux)) / (2*y);
   f_AyMinus = ((-71 +f_Ux*f_Ux +f_Uy*f_Uy) -f_AxMinus*(-5+2*f_Ux)) / (2*y);
   
   // initially, guess that we're using the (Ax,Ay) pair with the "+" in quadratic solution
   f_Ax = f_AxPlus;
   f_Ay = f_AyPlus;

   // but swap if the other coordinates have a higher Y value, and the X value is on positive side of knee,
   // .. which means knee can reach it without exceeding 90 degrees  
   if(f_AyMinus > f_AyPlus && f_AxMinus >= 2.5)
   {  f_Ax = f_AxMinus;
      f_Ay = f_AyMinus;
   }
   // however, if that puts x to the -ve side of knee, where ankle can't go, pick the other case
//   if(f_Ax < 2.5)
//   {  f_Ax = f_AxPlus;
//      f_Ay = f_AyPlus;
//   }
   // there are still some unlikely edge cases needing attention, such as h=0, k=-44, a=75

//spr("`AxPlus,AyPlus= "); spr(f_AxPlus); sp; spl(f_AyPlus);
//spr("`AxMinus,AyMinus= "); spr(f_AxMinus); sp; spl(f_AyMinus);
//spr("`Ax,Ay= "); spr(f_Ax); sp; spl(f_Ay);

// think following stuff is obsolete, but keeping the formulas
   // get y by substituting x into a previous equation, again dependent on sign of y
//    if(y>0)   {f_Ay = -1*((-71+f_Ux*f_Ux+y*y) - f_Ax*(-5+2*f_Ux))/(2*y);}
//    if(y<=0)  {f_Ay =    ((-71+f_Ux*f_Ux+y*y) - f_Ax*(-5+2*f_Ux))/(2*y);}


    // now that we know where the ankle is, we can finally work on the angles
    // the knee is easy since we defind the ankle position above

    f_angK = -1* asin( f_Ay / 7.5) / PI * 180;    

// there are 4 possible cases for ankle position, with different calculations for A angle
// 1) Ux>2.5, Uy<0  // the normal case, toe below knee
// 2) Ux>2.5, Uy>=0  // still normal, toe above knee
// 3) Ux<2.5, Uy>0  // unreachable if servo angle limited to 90 degrees
// 4) Ux<2.5, Uy=<0  // toe is underneath robot

// should put in defenses for unreasonable angles being returned

// for cases 1 and 4
   if(f_Uy < 0)
   {
      float f_P = asin((f_Ux - f_Ax)/11) / PI *180;      // can only explain this with a diagram
      f_angA = f_P + f_angK - 17;
   }  // Uy <0

// for cases 2, and impossible 3
   if(f_Uy >= 0)
   {
      float f_R = asin((f_Uy - f_Ay)/11) / PI *180;      // can only explain this with a diagram
      f_angA = f_R + 90 +f_angK - 17;
   }  // Uy <0
   
//spr("`  ang H= "); spl(f_angH);  
//spr("`  ang K= "); spl(f_angK);  
//spr("`  ang A= "); spl(f_angA);  
    


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
      argN ++ ;                 // advance the argument counter
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
      spr("` coords: "); spr(f_cx); sp; spr(f_cy); sp; spl(f_cz);

      // worst case is moving 90 degrees at .17 sec per 60 degrees, so ...
      delay(510);  // wait for moves to complete
      return true;
   } // if cmd = goto_angles


   if(cmd == "GOTO_COORDS" || cmd == "GC")
   // format: goto_coords,<x value>,<y value>,<z value>
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
      
      pwm.setPWM(servoMotor[1].driverPort, SERVO_START_TICK, mapDegToPWM(f_angH, 0)); // Hip
      pwm.setPWM(servoMotor[2].driverPort, SERVO_START_TICK, mapDegToPWM(f_angK, 0)); // Knee
      pwm.setPWM(servoMotor[3].driverPort, SERVO_START_TICK, mapDegToPWM(f_angA, 0)); // Ankle 
      spr("` angles: "); spr(f_angH); sp; spr(f_angK); sp; spl(f_angA);

      // worst case is moving 90 degrees at .17 sec per 60 degrees, so ...
      delay(510);  // wait for moves to complete
      return true;
   } // if cmd = goto_coords



// FLOW command for storing next position in a smooth motion flow between multiple positions
// the flow command builds arrays describing the desired movement
// the flow_go command starts the movement and controls repetitions, resets, etc

   if(cmd == "FLOW" || cmd == "FL")
   // there are 2 command formats, defined by the style parameter:
   // style =1 is based on servo angles...
   //    format: flow,<hip angle>,<knee angle>,<ankle angle>, <duration in msec>, <style=1>
   //            with all angles in degrees, center (north) = 0
   // style = 2 is based on (x,y,z) coordinates
   //    format: flow,<X value>,<y value>,<z value>, <duration in msec>, <style=2>
   //
   // action: - save this position in the flow arrays, to be part of the motion initiated by flow_go command
   //         - duration (ignored for first position in a flow) is elapsed time to get to this position, in milliseconds
   //           (it basically controls the speed of movement)
   //  style: - 1 is linear transition from start angle to end angle, for each joint
   //         - 2 is linear progress along a line from starting position (equivalent (x,y,z)) to end position
   //         - style is ignored for first position
   // example: style 1:  fl,0,0,0,99,1           would put robot in normal neutral stance
   //                    fl,0,-45,45,1000,1      then move smoothly to "peeing on a tree" stance
   //
   //          style 2:  fl,10,-8.5,0,99,2       would put robot in normal neutral stance
   //                    fl,18.8,5.3,0,1000,2    moves to pee on a tree stance

   {
      Serial.println("start flow command");
 
      // copy position description from MQTT flow command to flow arrays
      // f_count starts at 0, which we use to store first position
      f_x_hip[f_count] = arg[1].toFloat();  // depending on style, this is either hip angle or x coordinate
      f_y_knee[f_count] = arg[2].toFloat();
      f_z_ankle[f_count] = arg[3].toFloat();
      f_msecs[f_count] = arg[4].toFloat();
      f_style[f_count] = arg[5].toInt();

      f_count ++;             // advance to next entry in flow arrays

      return true;
   } // if cmd = flow

// FLOW_GO command to start up motion as previously defined in the flow arrays using the FLOW command
// the FLOW command builds arrays describing the desired movement
// the FLOW_GO command starts the movement and controls repetitions, resets, etc
//
// format: FLOW_GO, <action>,<cycles>,<msecPerFrame>
// action: 1 - start the flow currently defined in the flow arrays
//         0 - reset the current flow, and empty the flow arrays
// cycles: number of times to repeat the flow, 0 or 1 mean once
// msecPerFrame: duration of each frame (fraction of a position) in millis. this determines f_framesPerPosn

   if(cmd == "FLOW_GO" || cmd == "FG")
   {
      int f_action = arg[1].toInt();      // decode the first argument - either start flow, or reset it
      if(f_action == 0)                   // zero means reset
      {  f_flowing = false;
         f_count = 0;
         f_active = 0;
         return true;
      }
      if(f_action == 1)                // 1 means start up the currently define flow
      {
         if(f_count == 0)              // is there a flow defined to run?
         {  Serial.println("<flow_go>: tried to run flow, but none defined");
            return false;
         }
         f_flowing = 1;                // we're now executing a flow
         f_active = 0;                 // starting at the 0th entry in the flow arrays

         f_cycles = 1;                 // default is one time through the flow
         if( argN > 1)                 // if there were at least 2 numeric arguments for FG command
         {
            f_cycles = arg[2].toInt();  // second number is cycle count
spr2l("cycles from command= ",f_cycles);
            if(f_cycles < 1 || f_cycles > 20) {f_cycles = 1;}    // override invalid cycle counts
         }
         f_msecPerFrame = f_msecPerFrame_default;  // if not given in FG command, use the default
         if(argN > 2)                  // if there were at least 3 numeric args to FG command
         {                             // ... 3rd one is msecPerFrame
            f_msecPerFrame = arg[3].toInt();  // 3rd number is millis per frame
            if(f_msecPerFrame<10 || f_msecPerFrame>200) {f_msecPerFrame = f_msecPerFrame_default;}
         }
      } // if action = 1
      else
      {   Serial.println("<flow_go>: invalid action in MQTT flow_go command");
          return false;
      }
   return true;
   } // if cmd = flow_go


   // If the command sent is unrecognized.
   Serial.println("<processCmd> Warning - unrecognized command."); 
   return false;
} // processCmd()

 #include <m$do_flow.cpp>



// moving do_flow() to an include file called m$do_flow   where m$ identifies a module

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

   pinMode(LED, OUTPUT);         // set up onboard LED on gpio 2 for error signalling

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
     do_flow();             // yes. calculate and do next servo commands 
  }
} // loop()