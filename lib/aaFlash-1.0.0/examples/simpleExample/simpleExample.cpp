/******************************************************************************
 * @file simpleExample.cpp
 *
 * @page Example sketch showing how to use the aaCFlash class. 
 * 
 * @section intro_sec Introduction
 *
 * This is an example sketch provided with the aaFlash library that shows how
 * to read and write to flash memory so the information  is not lost during reboot.
 *
 * @section dependencies Dependencies
 * 
 * This sketch class depends on on the following libraries:
 * * <a href="https://github.com/espressif/arduino-esp32">Arduino.h</a>. This 
 * is the Arduino core library that comes bundled with PlatformIO.
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
#include <Arduino.h> // Arduino Core for ESP32. Comes with Platform.io.
#include <aaFlash.h> // Store values that persist past reboot.

/**
 * Define global objects.
 * =================================================================================*/
aaFlash flashMem; // Non-volatile memory management. 

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
 * @brief Arduino mandatory function #1. Runs once at boot. 
 * =================================================================================*/
void setup()
{
   IPAddress ip1, ip2, ip3; // Declare IPAddress varaibles.
   IPAddress tmpIp1(0,0,0,0); // Declare and initialize IPAddress variable.
   IPAddress tmpIp2(1,1,1,1); // Declare and initialize IPAddress variable.

   setupSerial(); // Set serial baud rate. 
   Serial.println("<setup> Start of setup");

   ip1 = flashMem.readBrokerIP(); // Read IP address from flash.
   Serial.print("<simpleExample> IP Address = "); Serial.println(ip1);

   flashMem.writeBrokerIP(tmpIp1); // Write IP address to flash.
   ip2 = flashMem.readBrokerIP(); // Read IP address from flash.
   Serial.print("<simpleExample> IP Address = "); Serial.println(ip2);

   flashMem.writeBrokerIP(tmpIp2); // Write IP address to flash.
   ip3 = flashMem.readBrokerIP(); // Read IP address from flash.
   Serial.print("<simpleExample> IP Address = "); Serial.println(ip3);

   Serial.println("<setup> End of setup");
} // setup()

/**
 * @brief Arduino mandatory function #2. Runs continually.
 * =================================================================================*/
void loop()
{

} //loop()