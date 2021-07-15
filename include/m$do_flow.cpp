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
         if(f_active < f_count)             // if there are still positions to do
         {
            if(f_style[f_active] == 1)        // if we're moving by angle changes...
            {
               f_frameHip   = f_lastHa + (f_frame/f_framesPerPosn) * f_deltaHip;    // figure servo positions
               f_frameKnee  = f_lastKa + (f_frame/f_framesPerPosn) * f_deltaKnee; 
               f_frameAnkle = f_lastAa + (f_frame/f_framesPerPosn) * f_deltaAnkle;
            } // if style == 1

            // if we're using style = 2, those are coordinates, not angles. need to convert
            else if(f_style[f_active] == 2)       // if we're moving by coordinate changes
            {
               f_frameHip   = f_lastHc + (f_frame/f_framesPerPosn) * f_deltaHip;    // figure updated coordinates
               f_frameKnee  = f_lastKc + (f_frame/f_framesPerPosn) * f_deltaKnee; 
               f_frameAnkle = f_lastAc + (f_frame/f_framesPerPosn) * f_deltaAnkle;
   spr("` f_lastHc,f_lastKc, f_lastAc = ");spr(f_lastHc); sp; spr(f_lastKc); sp; spl(f_lastAc);

   spr("` f_deltaHip, f_deltaKnee, f_deltaAnkle= "); spr(f_deltaHip); sp; spr(f_deltaKnee); sp; spl(f_deltaAnkle);

               coordsToAngles(f_frameHip, f_frameKnee, f_frameAnkle);
               f_frameHip = f_angH;
               f_frameKnee = f_angK;
               f_frameAnkle = f_angA;
            } // if style ==2
            else
            {  f_frameHip = 0;    // appeasing crabby compiler
               f_frameKnee = 0;
               f_frameAnkle = 0;
            }
   spr("`f_frameHip... = "); spr(f_frameHip); sp; spr(f_frameKnee); sp; spl(f_frameAnkle);
            // move servo's a fraction of the way to next position
            pwm.setPWM(servoMotor[1].driverPort, SERVO_START_TICK, mapDegToPWM(f_frameHip, 0)); // Hip
            pwm.setPWM(servoMotor[2].driverPort, SERVO_START_TICK, mapDegToPWM(f_frameKnee, 0)); // Knee
            pwm.setPWM(servoMotor[3].driverPort, SERVO_START_TICK, mapDegToPWM(f_frameAnkle, 0)); // Ankle 

   spl("`------------------------------------------------------");
   spr("`active,frame,frame angles: ");sprs(f_active);sprs(f_frame);  sprs(f_frameHip); sprs(f_frameKnee); spl(f_frameAnkle);

            f_frame = f_frame + 1 ;           // on to next frame within this position
   // spr2("`f_frame=",f_frame); spr2(",  f_active=",f_active); nl;

            f_nextTime = millis() + f_msecPerFrame;
   sprs("`time now, nxt fram= "); sprs(millis()); spl(f_nextTime);
            if(f_frame > f_framesPerPosn)       // did we finish all frames for this position?
            {
               // yup, so we must be sitting at the next position. reorganize for next set of frames
               f_active ++ ;      // we might have run out of positions, but we'll see that after frame time expires

               // remember current position for delta calculations
               // easy to know position as angles - we just fed that to servos
               f_lastHa = f_frameHip;
               f_lastKa = f_frameKnee;
               f_lastAa = f_frameAnkle;

               // but we might not have come through the coordinate calculations, so recalc..
               anglesToCoords(f_lastHa, f_lastKa, f_lastAa);    // translate angles to coordinates
               
               // and remember the resulting coordinates
               f_lastHc = f_cx;
               f_lastKc = f_cy;
               f_lastAc = f_cz;

               if(f_style[f_active] == 1)     // next position is expressed as angles
               {
               f_deltaHip = f_x_hip[f_active] - f_lastHa;      // figure travel angle for hip to next position
               f_deltaKnee = f_y_knee[f_active] - f_lastKa;
               f_deltaAnkle = f_z_ankle[f_active] - f_lastAa;
               } // if style == 1

               if(f_style[f_active] == 2)     // next position is expressed as coordinates
               {
               f_deltaHip = f_x_hip[f_active] - f_lastHc;      // figure coordinate difference for hip to next position
               f_deltaKnee = f_y_knee[f_active] - f_lastKc;
               f_deltaAnkle = f_z_ankle[f_active] - f_lastAc;
               } // if style = 2
sprs("`deltas-2: "); sprs(f_deltaHip); sprs(f_deltaKnee); spl(f_deltaAnkle);               

               f_frame = 1;     // starting a new set of frames
//////////////// spl("just reset f_frame");
            }  // if frame > framesPerPosn
            int f_millis = millis();
            float f_tmp = f_msecs[f_active] / f_framesPerPosn +.5;
sprs("`mscs,fPP,tmp = "); sprs(f_msecs[f_active]); sprs(f_framesPerPosn); spl(f_tmp);
            f_nextTime = millis() + int(f_tmp);   //get speed info from next position
            sprs("`pre-millis= "); spl(f_millis);


         } // if active < count
         else    // ran out of positions. do we need to do more cycles?
         {
		   // cycles not implemented. stop after first
		   f_nextTime = 0;         // stop any further frame processing from moving servos
		   f_flowing = false;      // exit from flow processing
         } // else
       } //if millis > f_nextTime

   } // if f_active > 0
// ===============================================================================================
   if(f_active == 0 )         // if this is first call to do_flow after MQTT flow_go command...
   {
      // if we're in style 2, using coordinates, we'll need to translate to angles
      f_th = f_x_hip[0];
      f_tk = f_y_knee[0];
      f_ta = f_z_ankle[0];

      if(f_style[0] == 2)                    // if values are coordinates rather than servo angles...
      {  coordsToAngles(f_th, f_tk, f_ta);  // convert & store in global variables
         f_th = f_angH;                     // retrieve resulting angles
         f_tk = f_angK;
         f_ta = f_angA;
      }
spr("`initial angles= ");spr(f_th); sp; spr(f_tk); sp; spl(f_ta);
      // move the servos in parallel at top speed to angles in first array entry
      pwm.setPWM(servoMotor[1].driverPort, SERVO_START_TICK, mapDegToPWM(f_th, 0)); // Hip
      pwm.setPWM(servoMotor[2].driverPort, SERVO_START_TICK, mapDegToPWM(f_tk, 0)); // Knee
      pwm.setPWM(servoMotor[3].driverPort, SERVO_START_TICK, mapDegToPWM(f_ta, 0)); // Ankle 
      f_lastHa = f_th;      // remember angles of last position to calculate next deltas
      f_lastKa = f_tk;
      f_lastAa = f_ta;

      anglesToCoords(f_th, f_tk, f_ta);       // translate these to x,y,x coordinates
      f_lastHc = f_cx;      // and remember the resulting coordinates
      f_lastKc = f_cy;
      f_lastAc = f_cz;

      delay(200);          // delay to let servos move to initial position
sptv("`f_x_hip[0] ",f_x_hip[0]); sp; spr(f_y_knee[0]); sp; spl(f_z_ankle[0]);
sptv("`f_x_hip[1] ",f_x_hip[1]); sp; spr(f_y_knee[1]); sp; spl(f_z_ankle[1]);
      
      if(f_style[f_active] == 1)                 // if we're moving by angles...
      {  f_deltaHip = f_x_hip[1] - f_lastHa;         // figure the angle to be travelled for hip to next position
         f_deltaKnee = f_y_knee[1] - f_lastKa;
         f_deltaAnkle = f_z_ankle[1] - f_lastAc;
      }  // if style == 1

      if(f_style[f_active] == 2)                 // if we're moving by coordinates...
      {  f_deltaHip = f_x_hip[1] - f_lastHc;         // figure the coordinat delat to be travelled for hip to next position
         f_deltaKnee = f_y_knee[1] - f_lastKc;
         f_deltaAnkle = f_z_ankle[1] - f_lastAc; 
      }  // if style == 2

sprs("`deltas: "); sprs(f_deltaHip); sprs(f_deltaKnee); spl(f_deltaAnkle);

// following approach has changed...
      // we're setting servos at f_framesPerPosn frames between positions.
      // calculate initial time delay until 1st reposition, in rounded integer milliseconds
      // and schedule next flow processing
//here's the new approach
      // we move the servos at a fixed frequency, determined by f_msecPerFrame = milleseconds between servo moves
      // from this, we calculate f_framesPerPosn, by dividing the total time to get to the next position,
      // which is given in the flow, by the time per frame. This can change with every position in the flow.
      // position times are effectively rounded up to the next frame time.
      // --- Example ---
      // say we're doing a frame every 25 msec, or 40 frames per second.
      // we indicate this by setting f_msecPerFrame to 25
      // if we have a position specifying a time interval of 500 msec, or half a second,
      // we will do 500 / 25 = 20 frames to get to that position, each of them 25 msec apart
      // so for this position, f_framesPerPosn = 20.

      // TODO maybe make f_msecPerFrame an optional parameter in the flow_go command?

      f_nextTime = millis() + f_msecPerFrame;
sprs("`times: "); sprs(millis()); spl(f_nextTime);
      f_frame = 1;      // frame number we'll do next
      f_active = 1;     // we're now working towards the position in index 1 of the flow arrays
      f_framesPerPosn = int(msec[1] / f_msecPerFrame +.5);   // rounded calc of how many frames we have time for
   } // if f_active = 0
}// void do_flow
