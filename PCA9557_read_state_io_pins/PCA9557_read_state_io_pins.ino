#include <Wire.h>

#define PCA9557_ADDR 0x18 // Address of PCA9557 chip
#define CONFIGURATION_MODE 0x03 // Configuration register: which mode is pins operated at? 
#define OUTPUT_REGISTER 0x01
#define INPUT_REGISTER 0x00

unsigned long previousMillis = 0;  // will store last time LED was updated.
unsigned long ledOnTime = 0; // will store time when LED turned ON
unsigned long flipTime = 0;
unsigned long delayTime = 0; // Measuring delay of transmissions.

// constants won't change:
const long interval = 500;  // interval at which to blink (milliseconds)
bool led1_state = true;
bool led2_state = false; 

void setup() {
  Serial.begin(115200); // Initialize serial communication
  Wire.begin(); // Initialize I2C communication
  Wire.setClock(400000);
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
      //flipTime = micros();
      turnOffLEDs(); // Turn off LEDs connected to IO2 and IO5
    }
    else {
      // Record the time when LED turns ON
      //flipTime = micros();
      ledOnTime = millis();
      turnOnLEDs(); // Turn on LEDs connected to IO2 and IO5
    }
    
    // Read and print the state of IO pins
    //readState();
    //readAndPrintState();


  }
}

void configureOutput() {
  Wire.beginTransmission(PCA9557_ADDR); // Start transmission to PCA9557
  Wire.write(CONFIGURATION_MODE); // Configuration register for Ports.
  Wire.write(0b11011011); // Set IO2 and IO5 as outputs, all other pins as inputs. 0=output, 1=input
  Wire.endTransmission(); // End transmission
}

void turnOnLEDs() {
  flipTime = micros();
  Wire.beginTransmission(PCA9557_ADDR); // Start transmission to PCA9557
  Wire.write(OUTPUT_REGISTER); // Output register for Port 0 (OLAT0)
  Wire.write(0b00000100); // Set IO2 and IO5 high, all other pins unchanged, (io7 io6 io5 io4 io3 io2 io1 io0)
  Wire.endTransmission(); // End transmission
  delayTime = micros() - flipTime;
  if (delayTime > 140) {
    Serial.println(1);
  }
}

void turnOffLEDs() {
  Wire.beginTransmission(PCA9557_ADDR); // Start transmission to PCA9557
  Wire.write(OUTPUT_REGISTER); // Output register for Port 0 (OLAT0)
  Wire.write(0b00100000); // Set all pins low
  Wire.endTransmission(); // End transmission
}

void readState() {
  Wire.beginTransmission(PCA9557_ADDR); // Start transmission to PCA9557
  Wire.write(OUTPUT_REGISTER); // Input register for Port 0 (IOP0)
  Wire.endTransmission(); // End transmission
}

void readAndPrintState() {
  Wire.beginTransmission(PCA9557_ADDR); // Start transmission to PCA9557
  Wire.write(OUTPUT_REGISTER); // Input register for Port 0 (IOP0)
  Wire.endTransmission(); // End transmission
  
  Wire.requestFrom(PCA9557_ADDR, 1); // Request one byte from PCA9557
  
  if (Wire.available()) {
    byte ioState = Wire.read(); // Read the state of IO pins
    //Serial.print("IO states: ");
    //Serial.println(ioState);  // 4 = b00000100, 32 = b00100100
    Serial.print("IO2 state: ");
    Serial.println(bitRead(ioState, 2) == 1 ? "HIGH" : "LOW"); // Check the state of IO2
    Serial.print("IO5 state: ");
    Serial.println(bitRead(ioState, 5) == 1 ? "HIGH" : "LOW"); // Check the state of IO5
  }
}
