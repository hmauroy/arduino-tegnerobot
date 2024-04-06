// DualMotorShield.pde
// -*- mode: C++ -*-
//
// Shows how to run 2 simultaneous steppers
// using the Itead Studio Arduino Dual Stepper Motor Driver Shield
// model IM120417015
// This shield is capable of driving 2 steppers at 
// currents of up to 750mA
// and voltages up to 30V
// Runs both steppers forwards and backwards, accelerating and decelerating
// at the limits.
//
// Copyright (C) 2014 Mike McCauley
// $Id:  $

#include <AccelStepper.h>

// The X Stepper pins
#define STEPPER1_DIR_PIN 6
#define STEPPER1_STEP_PIN 8
// The Y stepper pins
#define STEPPER2_DIR_PIN 7
#define STEPPER2_STEP_PIN 9

// Define some steppers and the pins the will use
AccelStepper stepper1(AccelStepper::DRIVER, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
AccelStepper stepper2(AccelStepper::DRIVER, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);

int xpos = 6000;
int ypos = 2000;
bool xfinish = false;
bool yfinish = false;

void setup()
{  
    Serial.begin(9600);
    stepper1.setMaxSpeed(1530.0);
    stepper1.setAcceleration(200.0);
    
    stepper2.setMaxSpeed(510.0);
    stepper2.setAcceleration(66.67);

    

    Serial.println("Starts in 1000 ms");
    delay(1000);

    stepper1.moveTo(xpos);
    stepper2.moveTo(ypos);
}

void loop()
{
  //   // Change direction at the limits
  //   if (stepper1.distanceToGo() == 0)
	// stepper1.moveTo(-stepper1.currentPosition());
  //   if (stepper2.distanceToGo() == 0)
	// stepper2.moveTo(-stepper2.currentPosition());
  if (stepper1.distanceToGo() == 0 && xfinish == false) {
    xfinish = true;
    Serial.println("Motor X finished");
  }
  if (stepper2.distanceToGo() == 0 && yfinish == false) {
    yfinish = true;
    Serial.println("Motor Y finished");
  }
  stepper1.run();
  stepper2.run();


  
}