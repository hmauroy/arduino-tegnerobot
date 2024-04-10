/*
 * A4988 functions abstracted away
 */
#define PI 3.1415926535897932384626433832795

 // Functions
void stopMotors() {
  digitalWrite(stepXpin, LOW);
  digitalWrite(stepYpin, LOW);
}

void homeHead() {
  /*
   * Moves head to 0,0 (upper left corner).
   * Moves steppers too much into negative direction.
   */
}

void zeroHead() {
  x0 = 0;
  y0 = 0;
}

void liftPen() {
  // Lift pen.
  lifted = true;
}

void lowerPen() {
  // Lower pen.
  lifted = false;
}

void moveHeadTo(float targetX, float targetY, double pulse_length) {

  /*
   Input is absolute pixel coordinates (stepper-motor pulses) 
   Moves to coordinate (x,y), then sets x0 and y0 to these values.
   1) alculate speed in each axis to reach destination at the same time.
   2) Move x and y in parallel using different speeds.
  */
  dx = targetX - x0;
  dy = targetY - y0;
  
  // Calculate distances (number of steps) and timePerStep and save to global variables..
  //calcTimePerStep(dx, dy);

  // Manual calculation
  n_stepsX = round(dx);
  n_stepsY = round(dy);
  double s = sqrt(dx*dx + dy*dy);
  double t = s/vmax;

  if (n_stepsX != 0) {
    timePerStepX = floor(1000*(1000*t / float(n_stepsX)));
  }
  if (n_stepsY != 0) {    
    timePerStepY = floor(1000*(1000*t / float(n_stepsY)));
  }

  //String message = "dx, dy " + String(dx) + "," + String(dy);
  //Serial.println(message);

  // Run steppers using speeds vx and vy which controls the timing of the steppers.

  //String temp2 = "nx, ny, timePerStepX,timePerStepY: " + String(n_stepsX) + "," + String(n_stepsY) + "," + String(timePerStepX) + "," + String(timePerStepY);
  String temp2 = "nx, ny, timePerStepX,timePerStepY: " + String(n_stepsX) + "," + String(n_stepsY) + "," + String(pulse_length) + "," + String(pulse_length);
  Serial.println(temp2);

  //runSteppers(n_stepsX, n_stepsY, timePerStepX, timePerStepY);  // nsteps, time per step in microseconds
  runSteppers(n_stepsX, n_stepsY, pulse_length, pulse_length);  // nsteps, time per step in microseconds

  // Update real integer coordinates

  x0 = x0 + dx; // Purely theoretic positions.
  y0 = y0 + dy;
  
  //x0 = x0 + n_stepsX; // This is correct, but may wreck the drawing.
  //y0 = y0 + n_stepsY;



  String str4 = "Head position: x0,y0: " + String(x0) + "," + String(y0);
  Serial.println(str4);
  
}

void drawCircle(float xcenter, float ycenter, float r) {
  liftPen();
  // Move to xy_start.
  //moveHeadTo(xcenter+radius, ycenter); // NB! + radius of circle!
  // Move along circle of radius r.
  // 1) Divide circle into segments based on lengt of circumference and resolution limit.
  // Length og line segment = 5 mm for first try.
  float segment_length = 5;  // Default 24 for circle with r = 30 mm
  float n_segments = (2*PI*r)/segment_length;
  byte n = ceil(n_segments);
  Serial.print("n_segments Circle: ");
  Serial.println(n);
  float d_theta = 2*PI/n;
  float theta = 0;

  // Initialize x0 and y0, radius as pixel values
  unsigned int x0_px = round(xcenter / stepLength); // Pixel-position of center of circle.
  unsigned int y0_px = round(ycenter / stepLength);
  radius = round(r / stepLength);
  x0 = x0_px + radius;
  y0 = y0_px;


  // 2) Run for loop of n line segments.
  //n = 3;
  for (byte i=0; i < n; i++) {
    /*
    Serial.println("--------------");
    Serial.print("segment: ");
    Serial.print(i+1);
    Serial.print(": ");
    String positions = "x0,y0: " + String(x0) + "," + String(y0);
    Serial.println(positions);*/
    theta = theta + d_theta;
    x = radius * cos(theta) + x0_px;
    y = radius * sin(theta) + y0_px;
    String target = String(x) + "," + String(y);
    //Serial.println(target);
    dx = x-x0;
    dy = y-y0;
    // Calculate distances (number of steps) and timePerStep and save to global variables..
    calcTimePerStep(dx, dy);
    //String message = "dx, dy " + String(dx) + "," + String(dy);
    //Serial.println(message);

    // Run steppers using speeds vx and vy which controls the timing of the steppers.

    String temp2 = "nx, ny, timePerStepX,timePerStepY: " + String(n_stepsX) + "," + String(n_stepsY) + "," + String(timePerStepX) + "," + String(timePerStepY);
    Serial.println(temp2);

    runSteppers(n_stepsX, n_stepsY, timePerStepX, timePerStepY);  // nsteps, time per step in microseconds

    // Update real integer coordinates

    x0 = x0 + dx; // Purely theoretic positions.
    y0 = y0 + dy;
    
    //x0 = x0 + n_stepsX; // This is correct, but may wreck the drawing.
    //y0 = y0 + n_stepsY;
    

  }

  String str4 = "Head position after Circle: x,y: " + String(x0) + "," + String(y0);
  Serial.println(str4);
}

void calcTimePerStep(float dx, float dy) {
  /*
    Calculates distances, nsteps + timings.
    Saves to global variables.
  */
  n_stepsX = round(dx);
  n_stepsY = round(dy);
  timePerStepX = 0;
  timePerStepY = 0;

  // Calculates distance and preserves the linear speed along line.
  double s = sqrt(dx*dx + dy*dy);
  double t = s/vmax;
  double vx = dx/t;
  double vy = dy/t; // px/s

  String speedInfo = "s, t, vx, vy: " + String(s) + "," + String(t) + ","+ String(vx) + ","+ String(vy);
  //Serial.println(speedInfo);


  // Calculates n_steps in each axis
  //String buffer = "dx, dy,vx,vy: " + String(dx) + "," + String(dy) + "," + String(vx) + "," + String(vy);
  //String buffer = "dx, dy: " + String(dx) + "," + String(dy);
  String buffer = "n_stepsX, n_stepsY: " + String(n_stepsX) + "," + String(n_stepsY);
  //Serial.println(buffer);
 

  // Calculates timePerStep in each axis
  /*float temp = (1000*tx)/float(n_stepsX);
  Serial.print("temp timePerStep (ms): ");
  Serial.println(temp);*/
  if (n_stepsX != 0) {
    timePerStepX = floor(1000*(1000*t / float(n_stepsX)));
  }
  if (n_stepsY != 0) {    
    timePerStepY = floor(1000*(1000*t / float(n_stepsY)));
  }

 
    
} // END calcTimePerStep()

/* Controls both stepper motors in parallell.
nx: number of steps in x-axis, negative value is counter clockwise.
ny: number of steps in y-axis, negative value is counter clockwise.
timePerStepX: time per step in microseconds for x-stepper.
timePerStepY: time per step in microseconds for ystepper.
*/ 
void runSteppers(int nx, int ny, double timePerStepX, double timePerStepY) {
  String dirX = "CW";
  String dirY = "CCW";
  pauseX = int(round(timePerStepX/2));  // Divide by 2 because step signal is 2 x delayMicroseconds.
  pauseY = int(round(timePerStepY/2));
  if (nx > 0) {
    digitalWrite(dirXpin, HIGH); // Set the spinning direction clockwise (x- and y-axis are wired opposite at the moment).
    dirX = "CW";
    if (xdirLast == -1) {
      // Direction change from negative movement!
      // Pulse steppers N times. Quick calculation: 2 deg move => N = 2 deg / 0.043945 deg/rot = 45.51 steps.
      // 20 pulses * 350 us total time/pulse = 7000 us = 7 ms.
      // Change timings to a shorter one.
      //changeStepperSpeed(true, pauseX, 5);
      xdirLast = 1;
    }
  }
  else if (nx < 0) {
    digitalWrite(dirXpin, LOW); // CCW
    dirX = "CCW";
    if (xdirLast == 1) {
      // Direction change from negative movement!
      // Pulse steppers N times. Quick calculation: 2 deg move => N = 2 deg / 0.043945 deg/rot = 45.51 steps.
      //changeStepperSpeed(true, pauseX, 5);
      xdirLast = -1;
    }
  }
  else {
    dirX = "STILL";
  }
  if (ny > 0) {
    digitalWrite(dirYpin, LOW); // Set the spinning direction counter clockwise
    dirY = "CCW";
    if (ydirLast == -1) {
      // Direction change from negative movement!
      // Pulse steppers N times. Quick calculation: 2 deg move => N = 2 deg / 0.043945 deg/rot = 45.51 steps.
      //changeStepperSpeed(false, pauseY, 30);
      ydirLast = 1;
    }
  }
  else if (ny < 0) {
    digitalWrite(dirYpin, HIGH); // CCW
    dirY = "CW";
    if (ydirLast == 1) {
      // Direction change from negative movement!
      // Pulse steppers N times. Quick calculation: 2 deg move => N = 2 deg / 0.043945 deg/rot = 45.51 steps.
      //changeStepperSpeed(false, pauseY, 30);
      ydirLast = -1;
    }
  }
  else {
    dirY = "STILL";
  }


  // Takes absolute values of both nsteps and pauseTime.
  nx = abs(nx);
  ny = abs(ny);
  pauseX = abs(pauseX);
  pauseY = abs(pauseY);
  /*Serial.print("nx, ny, pauseX, pauseY, dirX, dirY: ");
  String temp3 = String(nx) + "," + String(ny) + "," + String(pauseX)+ "," + String(pauseY) + "," + dirX + "," + dirY;
  Serial.println(temp3);
  Serial.println("---------");*/
  
  
  // Spin the steppers as long as there are steps.
  bool isRunning = true;
  bool sigX = true;
  bool sigY = true;
  unsigned long txLast = micros(); // time in microseconds. overflows in around 70 min.
  unsigned long tyLast = micros();
  unsigned long delta_tx = 0;
  unsigned long delta_ty = 0;
  while (isRunning) {
    delta_tx = micros() - txLast;
    delta_ty = micros() - tyLast;
    // Check x-axis for signal.
    if (delta_tx >= pauseX) {
      if (nx > 0) {
        sigX = !sigX;
        digitalWrite(stepXpin, sigX);
        txLast = micros();
        if (sigX == false) {
          nx -= 1;
        }
      }
      else {
        //Serial.print("X finished, time:");
        //Serial.println(micros());
        if (ny == 0) {
          isRunning = false;
        }
      }
    }
    // Check y-axis for signal
    if (delta_ty >= pauseY) {
      if (ny > 0) {
        sigY = !sigY;
        digitalWrite(stepYpin, sigY);
        tyLast = micros();
        if (sigY == false) {
          ny -= 1;
        }
      }
      else {
        //Serial.print("Y finished, time:");
        //Serial.println(micros());
        if (nx == 0) {
          isRunning = false;
        }
      }
    }

  } // End while loop
  //Serial.println("Finished runSteppers()");
} // END runSteppers()

void changeStepperSpeed(bool xaxis, int pause, int maxSteps) {
  String info = "ChangeSpeed, xaxis, pause, maxSteps: " + String(xaxis) + "," + String(pause) + "," + String(maxSteps);
  Serial.println(info);
  if (xaxis) {
    oldPauseX = abs(pause);
    pauseX = 500;
    // Reset counter to go back to normal speed after N steps.
    highSpeedCounterX = 0;
    maxHighSpeedStepsX = maxSteps;
  }
  else {
    oldPauseY = abs(pause);
    pauseY = 500;
    highSpeedCounterY = 0;
    maxHighSpeedStepsY = maxSteps;
  }
  
}

void pulseSteppers(bool xAxis, int n) {
  Serial.println("Pulses steppers!");
  int outputPin = 0;
  if (xAxis == true) {
    outputPin = stepXpin;
  }
  else {
    outputPin = stepYpin;
  }
  for (int i=0; i<n; i++) {
    //Serial.println("Pulses steppers!");
    digitalWrite(outputPin, y);
    delayMicroseconds(50);
    digitalWrite(outputPin, LOW);
    // Sleep for pulse off time
    delayMicroseconds(300); // Pulses rapidly at 1/4-stepping
  }
  
}

