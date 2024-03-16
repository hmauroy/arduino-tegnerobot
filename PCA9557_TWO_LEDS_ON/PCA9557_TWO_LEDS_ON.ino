#include <Wire.h>

#define PCA9557_ADDR 0x18 // Address of PCA9557 chip
#define CONFIGURATION_MODE 0x03 // Configuration register: which mode is pins operated at? 
#define OUTPUT_REGISTER 0x01

unsigned long previousMillis = 0;  // will store last time LED was updated.

// constants won't change:
const long interval = 50;  // interval at which to blink (milliseconds)
bool led1_state = true;
bool led2_state = false; 

void setup() {
  Wire.begin(); // Initialize I2C communication
  configureOutput(); // Configure PCA9557 for input/output
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    // Flip led states
    led1_state = !led1_state;
    led2_state = !led2_state;

    if (led1_state == true) {
      turnOffLEDs(); // Turn off LEDs connected to IO2 and IO5
    }
    else {
      turnOnLEDs(); // Turn on LEDs connected to IO2 and IO5
    }
  }

}

void configureOutput() {
  Wire.beginTransmission(PCA9557_ADDR); // Start transmission to PCA9557
  Wire.write(CONFIGURATION_MODE); // Configuration register for Ports.
  Wire.write(0b11011011); // Set IO2 and IO5 as outputs, all other pins as inputs. 0=output, 1=input
  Wire.endTransmission(); // End transmission
}

void turnOnLEDs() {
  Wire.beginTransmission(PCA9557_ADDR); // Start transmission to PCA9557
  Wire.write(OUTPUT_REGISTER); // Output register for Port 0 (OLAT0)
  Wire.write(0b00000100); // Set IO2 and IO5 high, all other pins unchanged
  Wire.endTransmission(); // End transmission
}

void turnOffLEDs() {
  Wire.beginTransmission(PCA9557_ADDR); // Start transmission to PCA9557
  Wire.write(OUTPUT_REGISTER); // Output register for Port 0 (OLAT0)
  Wire.write(0b00100000); // Set all pins low
  Wire.endTransmission(); // End transmission
}
