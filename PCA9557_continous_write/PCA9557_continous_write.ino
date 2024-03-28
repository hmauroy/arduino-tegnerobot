#include <Wire.h>

#define PCA9557_ADDR 0x18 // Address of PCA9557 chip
#define OUTPUT_REGISTER 0x01

void setup() {
  Wire.begin(); // Initialize I2C communication
  Wire.setClock(400000);
  configureOutput(); // Configure PCA9557 for output
}

void loop() {
  sendByteContinuously(); // Continuously send the byte over I2C
}

void configureOutput() {
  Wire.beginTransmission(PCA9557_ADDR); // Start transmission to PCA9557
  Wire.write(OUTPUT_REGISTER); // Output register for Port 0 (OLAT0)
  Wire.write(0b00000100); // Set IO2 high, all other pins low (decimal 4)
  Wire.endTransmission(); // End transmission
}

void sendByteContinuously() {
  Wire.beginTransmission(PCA9557_ADDR); // Start transmission to PCA9557
  Wire.write(OUTPUT_REGISTER); // Output register for Port 0 (OLAT0)
  Wire.write(0b00000100); // Write the byte
  Wire.endTransmission(false); // End transmission without sending stop condition
}
