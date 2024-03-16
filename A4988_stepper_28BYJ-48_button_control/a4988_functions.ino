/*
 * A4988 functions abstracted away
 */

 // Functions
void stopMotors() {
  digitalWrite(stepXpin, LOW);
  digitalWrite(stepYpin, LOW);
}

void stepperX(int dir, float T_rot) { // T_rot = timePerStep * stepsPerRevolution => timePerStep = T_rot/stepsPerRevolution
  timePerStepBuffer = T_rot/stepsPerRevolution;
  timePerStep = round(1000*timePerStepBuffer/2);  // Divide by 2 because step signal is 2 x delayMicroseconds.
  Serial.print("x: ");
  Serial.println(dir);
  if (dir == 1) {
    digitalWrite(dirXpin, HIGH); // Set the spinning direction clockwise
  }
  else if (dir == 0){
    digitalWrite(dirXpin, LOW); // CCW
  }

  // Spin the stepper motor in chunks of 100 steps
  for (int i = 0; i < 100; i++) {
    // These four lines result in 1 step:
    digitalWrite(stepXpin, HIGH);
    delayMicroseconds(timePerStep);
    digitalWrite(stepXpin, LOW);
    delayMicroseconds(timePerStep);
  }
}

void stepperY(int dir, float T_rot) { // T_rot = timePerStep * stepsPerRevolution => timePerStep = T_rot/stepsPerRevolution
  timePerStepBuffer = T_rot/stepsPerRevolution;
  timePerStep = round(1000*timePerStepBuffer/2);
  Serial.print("y: ");
  Serial.println(dir);
  if (dir == 1) {
    digitalWrite(dirYpin, HIGH); // Set the spinning direction clockwise
  }
  else {
    digitalWrite(dirYpin, LOW); // CCW
  }

  // Spin the stepper motor in chunks of 100 steps
  for (int i = 0; i < 100; i++) {
    // These four lines result in 1 step:
    digitalWrite(stepYpin, HIGH);
    delayMicroseconds(timePerStep);
    digitalWrite(stepYpin, LOW);
    delayMicroseconds(timePerStep);
  }
}
