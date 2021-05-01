# Introduction
This code is used to calibrate the frequency of the PCA9685 I2C 16 channel servo driver using a 30 pin ESP32 DEVKIT V1 board. This code and circuit demonstrates how to calibrate each individual servo motor.

## Overview
In order to ensure that all Hexapod robots behave the same way we need to follow two seperate calibration procedures. The first calibration procedure sets the output frequency of the PCA9685 servo driver. The second procedure calibrates the mapping of angles with the movement range of each servo motor attached to the driver. 

## Procedure 1. Calibrating the PCA9685 output Frequency
We started out using a procedure outline in [this repo](https://github.com/va3wam/PCA9685_Frequency_Calibration) for details and sample code for this procedure. Perhaps  a better approach is exaplined as follows:

In theory the internal oscillator (clock) is 25MHz but it really isn't that precise. You can 'calibrate' this by tweaking this number until you get the PWM update frequency you're expecting! The int.osc. for the PCA9685 chip is a range between about 23-27MHz and is used for calculating things like writeMicroseconds(). Analog servos run at ~50 Hz updates, It is importaint to use an oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.

1. Attach the oscilloscope to one of the PWM signal pins and ground on the I2C PCA9685 chip you are setting the value for.
2. Adjust setOscillatorFrequency() until the PWM update frequency is the expected value (50Hz for most ESCs)

Setting the value here is specific to each individual I2C PCA9685 chip and affects the calculations for the PWM update frequency. Failure to correctly set the int.osc value will cause unexpected PWM results

## Procedure 2. Calibrating the Servo Motor Angle
Put the steps for this procedure here.

The Adafruit library for controlling the PCA9685 module can be found [here](https://adafruit.github.io/Adafruit-PWM-Servo-Driver-Library/html/class_adafruit___p_w_m_servo_driver.html)

# Wiring
To follow the instructions in this section it will help to use the following orientations of the boards involved. 
* Orient your ESP32 so that the USB port is facing down.
* Orient the PCA9685 so that the big capacitor and the two screw down power terminals  are to the right.

## The ESP32
On the ESP32 use the following [physical pins](https://www.electronicshub.org/esp32-pinout/):
* Pin 11 goes to bottom right pin of first cluster of PWM pins on the PCA9685.  
* Pin 29 goes t the SCL header pin on the PCA9685.
* Pin 26 goes to the SDA header pin on the PCA9685.

## The PCA9685
In addition to the connections outlined in the previous section make the following additional connections to the PCA9685 board

To the header:
* V+ on the header pin row goes to +3.3vdc
* VCC on the header pin row goes to +3.3vdc
* GND on the header pin row goes to a GND on the ESP32 board
* Also connect +3.3vdc to the + screw down power terminal

# Wiring without LLC
These instructions tell you how to wire up a circuit to measure the PWM output frequency of the PCA9685 without the use of a logic level converter. This circuit is only useful for  bench testing. To follow the instructions in this section it will help to use the following orientations of the boards involved. 
* Orient your ESP32 so that the USB port is facing down.
* Orient the PCA9685 so that the big capacitor and the two screw down power terminals  are to the right.

## The ESP32
On the ESP32 use the following [physical pins](https://www.electronicshub.org/esp32-pinout/):
* Pin 11 goes to bottom right pin of first cluster of PWM pins on the PCA9685.  
* Pin 29 goes t the SCL header pin on the PCA9685.
* Pin 26 goes to the SDA header pin on the PCA9685.

## The PCA9685
In addition to the connections outlined in the previous section make the following additional connections to the PCA9685 board

To the header:
* V+ on the header pin row goes to +3.3vdc
* VCC on the header pin row goes to +3.3vdc
* GND on the header pin row goes to a GND on the ESP32 board
* Also connect +3.3vdc to the + screw down power terminal

# Wiring with an LLC
These instructions tell you how to wire up a circuit to measure the PWM output frequency of the PCA9685 with the use of a logic level converter. This circuit can be used in a working circuit not just a bench testing situation like the method from the previous section. To follow the instructions in this section it will help to use the following orientations of the boards involved. 
* Orient your ESP32 so that the USB port is facing down.
* Orient the PCA9685 so that the big capacitor and the two screw down power terminals  are to the right.
* Orient the LLC with the low level pins on the left hand side of the board.

## The ESP32
On the ESP32 use the following [physical pins](https://www.electronicshub.org/esp32-pinout/):
* Pin 11 goes to top left pin of the LLC (L1). 
* A second wire goes from the top right pin of the LLC (H1) to the bottom right pin of the first cluster of PWM pins on the PCA9685.  
* Pin 29 goes t the SCL header pin on the PCA9685.
* Pin 26 goes to the SDA header pin on the PCA9685.

## The PCA9685
In addition to the connections outlined in the previous section make the following additional connections to the PCA9685 board

To the header:
* V+ on the header pin row goes to +5VDC
* VCC on the header pin row goes to +3.3VDC
* GND on the header pin row goes to a GND on the ESP32 board

To the screw down power terminal block:
* Connect +5VDC to the V+ post
* Connect ground to the GND post

## The LLC
We are using the [KeeYees 4 Channel I2C Logic Level Converter Bi-Directional Module 3.3V to 5V Shifter for Arduino](https://www.amazon.ca/gp/product/B07LG646VS/ref=ppx_yo_dt_b_asin_title_o06_s00?ie=UTF8&psc=1). This board shifts the 5VDC signal coming out of the PCA9685 servo driver to 3.3VDC for input into pin 11 on the ESP32 dev board. 

* Connect the LV pin to the 3.3VDC power.
* Connect the GND pin on the left side of the board to ground.
* Connect the HV pin to the 5VDC power.
* Connect the GND pin on the right side of the board to ground.
