/*
Control of IO-expansion chip PCA9557 using I2C-protocol.

Connection
VSS - GND
VDD - 5V
SDA - PIN 20
SCL - PIN 21

IO2 - LED1 POSITIVE
IO5 - LED2 OSITIVE

Author: chatGPT 3.5

Question for chatGPT 3.5:
I have an arduino mega 2560, and I want to write a scetch for communication with a pca9557 chip. 
I want to configure it for output to control one led on each io pin io2 and io5. 
The scetch should have a loop that blinks the two leds on and off alternating between the leds.

chatGPT:
Sure, let's break down the steps to achieve this:

Connect PCA9557 to Arduino Mega: Connect SDA and SCL pins of PCA9557 to the corresponding 
pins on Arduino Mega (usually pin 20 for SDA and pin 21 for SCL). Additionally, connect the 
VCC and GND pins of PCA9557 to the appropriate power and ground pins on the Arduino.

Include Wire library: Begin by including the Wire library, which allows communication over I2C.

Define PCA9557 registers and addresses: Define the necessary registers and addresses for configuring the PCA9557 and controlling its output pins.

Initialize PCA9557: Write initialization code in the setup() function to configure PCA9557 for output.

Blink LEDs in the loop: Write code in the loop() function to alternate between turning on and off the LEDs connected to IO2 and IO5.
*/

#include <Wire.h>

// Define PCA9557 registers and addresses
#define PCA9557_ADDRESS 0x18
#define PCA9557_IO_CONFIG 0x03
#define PCA9557_OUTPUT_PORT 0x01

void setup() {
  Wire.begin(); // Initialize I2C communication
  configurePCA9557(); // Configure PCA9557 for output
}

void loop() {
  // Blink IO2 LED
  digitalWritePCA9557(2, HIGH); // Turn on IO2 LED
  delay(500); // Wait
  digitalWritePCA9557(2, LOW); // Turn off IO2 LED
  delay(500); // Wait

  // Blink IO5 LED
  digitalWritePCA9557(5, HIGH); // Turn on IO5 LED
  delay(500); // Wait
  digitalWritePCA9557(5, LOW); // Turn off IO5 LED
  delay(500); // Wait
}

void configurePCA9557() {
  Wire.beginTransmission(PCA9557_ADDRESS); // Start communication with PCA9557
  Wire.write(PCA9557_IO_CONFIG); // Select IO Configuration register
  Wire.write(0b11110000); // Configure all pins as outputs (IO0-IO7)
  Wire.endTransmission(); // End the transmission
}

void digitalWritePCA9557(int pin, bool state) {
  int portValue = 0;
  if (pin >= 0 && pin <= 7) {
    Wire.beginTransmission(PCA9557_ADDRESS); // Start communication with PCA9557
    Wire.write(PCA9557_OUTPUT_PORT); // Select Output Port register
    portValue = Wire.requestFrom(PCA9557_ADDRESS, 1); // Read current port value
    if (portValue > 0) {
      portValue = Wire.read(); // Read current port value
      if (state) {
        portValue |= (1 << pin); // Set the pin high
      } else {
        portValue &= ~(1 << pin); // Set the pin low
      }
      Wire.write(portValue); // Write new port value
      Wire.endTransmission(); // End the transmission
    }
  }
}
