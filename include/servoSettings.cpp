   // Servo motor calibration settings
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
      servoMotor[1].label = "DM2"; // The label marked on the servo.
      servoMotor[1].driverPort = 13; // The driver port (0-15) that the servo is connected to.
      servoMotor[1].stand = 300; // Position when standing to attention. 
      servoMotor[1].step = 400; // Position for full step formward.
      servoMotor[1].min = 72; // Minimal value servo can move to.
      servoMotor[1].max = 540; // Maximum value servo can move to.
      servoMotor[1].east = 120; // Servo is at the 0 position of a 180 degree arch.
      servoMotor[1].north = 300; // Servo is at the 90 degree position of a 180 degree arch.
      servoMotor[1].west = 505; // Servo is at the 180 degree position of a 180 degree arch.
      // Servo #2 - Knee Joint
      servoMotor[2].role = "Knee joint"; // The role this servo fills for the robot.
      servoMotor[2].label = "DM3"; // The label marked on the servo.
      servoMotor[2].driverPort = 14; // The driver port (0-15) that the servo is connected to.
      servoMotor[2].stand = 300; // Position when standing to attention. 
      servoMotor[2].step = 200; // Position for full step formward.
      servoMotor[2].min = 71; // Minimal value servo can move to.
      servoMotor[2].max = 530; // Maximum value servo can move to.
      servoMotor[2].east = 117; // Servo is at the 0 position of a 180 degree arch.
      servoMotor[2].north = 300; // Servo is at the 90 degree position of a 180 degree arch.
      servoMotor[2].west = 510; // Servo is at the 180 degree position of a 180 degree arch.
      // Servo #3 - Ankle Joint
      servoMotor[3].role = "Ankle joint"; // The role this servo fills for the robot.
      servoMotor[3].label = "DM4"; // The label marked on the servo.
      servoMotor[3].driverPort = 15; // The driver port (0-15) that the servo is connected to.
      servoMotor[3].stand = 300; // Position when standing to attention. 
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
