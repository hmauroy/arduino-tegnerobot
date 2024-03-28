// Wire Slave Receiver
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Receives data as an I2C/TWI slave device
// Refer to the "Wire Master Writer" example for use with this

// Created 29 March 2006

// This example code is in the public domain.


#include <Wire.h>

#define SLAVE_ADDRESS 0x18 // Address of this device

byte redLed = 6;
byte greenLed = 7;

void setup() {
  Wire.begin(SLAVE_ADDRESS);                // join I2C bus with address #24 (0x18)
  Wire.onReceive(receiveEvent); // register event
  Wire.setClock(100000);
  Serial.begin(9600);           // start serial for output
  pinMode(redLed, OUTPUT); // Red
  pinMode(greenLed, OUTPUT); // green
  digitalWrite(redLed, HIGH);
  digitalWrite(greenLed, LOW);
  delay(500);
  digitalWrite(redLed, LOW);
  digitalWrite(greenLed, HIGH);
  delay(500);
  digitalWrite(redLed, LOW);
  digitalWrite(greenLed, LOW);
}

void loop() {
  delay(100);
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {
  while (Wire.available()) {
    int x = Wire.read(); // Read the incoming byte
    Serial.println(x); // Print the received byte
    if (x == 4) {
    digitalWrite(redLed, HIGH);
     digitalWrite(greenLed, LOW);
    }
    else if (x == 32) {
      digitalWrite(redLed, LOW);
      digitalWrite(greenLed, HIGH);
    }
  }
}
