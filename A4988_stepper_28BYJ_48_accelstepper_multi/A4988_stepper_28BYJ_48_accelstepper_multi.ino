// MultiStepper.pde
// -*- mode: C++ -*-
// Use MultiStepper class to manage multiple steppers and make them all move to 
// the same position at the same time for linear 2d (or 3d) motion.

/* Edit by Henrik Mauroy
 April 2024

 Runs with stepper drivers.
 1/4 stepping
 vmax = 1600 steps/sec
 acceleration = 200 steps/s2


*/

#include <AccelStepper.h>
#include <MultiStepper.h>

// EG X-Y position bed driven by 2 steppers
// Alas its not possible to build an array of these with different pins for each :-(

// The X Stepper pins
#define STEPPER1_DIR_PIN 6
#define STEPPER1_STEP_PIN 8
// The Y stepper pins
#define STEPPER2_DIR_PIN 7
#define STEPPER2_STEP_PIN 9

// Define some steppers and the pins the will use
AccelStepper stepper1(AccelStepper::DRIVER, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
AccelStepper stepper2(AccelStepper::DRIVER, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);

// Up to 10 steppers can be handled as a group by MultiStepper
MultiStepper steppers;

void setup() {
  Serial.begin(9600);

  // Configure each stepper
  stepper1.setMaxSpeed(1600);
  stepper1.setAcceleration(200.0);
  stepper2.setMaxSpeed(1600);
  stepper2.setAcceleration(200.0);

  // Then give them to MultiStepper to manage
  steppers.addStepper(stepper1);
  steppers.addStepper(stepper2);
  Serial.println("Starts in 1000 ms");
  delay(1000);
}

void loop() {
  long positions[2]; // Array of desired stepper positions
  Serial.println("Moves to pos 1");
  
  positions[0] = 6452;
  positions[1] = 4032;
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); // Blocks until all are in position
  delay(1000);
  
  // Move to a different coordinate
  Serial.println("Moves to pos 2");
  positions[0] = 1000;
  positions[1] = 10000;
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); // Blocks until all are in position
  delay(1000);
}