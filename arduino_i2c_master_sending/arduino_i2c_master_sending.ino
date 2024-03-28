#include <Wire.h>

#define SLAVE_ADDRESS 0x18 // Address of the slave (Arduino Mega)

void setup() {
  Wire.begin(); // Initialize I2C communication as master
  Serial.begin(9600); // Initialize serial communication
}

int teller = 1;

void loop() {
  // Send data to Arduino Mega
  Wire.beginTransmission(SLAVE_ADDRESS); // Start transmission to slave
  Wire.write(teller); // Write data to the slave
  Wire.write(0); // Write data to the slave
  Wire.endTransmission(); // End transmission
  delay(100); // Wait for a second before sending the next data
  teller += 1;
  if (teller >= 127) {
    teller = 1;
  }
}
