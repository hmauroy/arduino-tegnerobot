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

void moveHeadTo(int targetX, int targetY) {
  float x_steps = targetX / (xCalibration / 5000);
  x = round(x_steps);
  float y_steps = targetY / (yCalibration / 5000);
  y = round(y_steps);
  runSteppersBres(x, y);
}

void runSteppersBres(int targetX, int targetY) {
  // Coordinate system is 0,0 upper left!
  x1 = targetX;
  y1 = targetY;
  dx = abs(x1 - x0);
  dy = -abs(y1 - y0);
  err = dx + dy;
  String message = "dx, dy " + String(x1-x0) + "," + String(y1-y0);
  Serial.println(message);
  if (x0 < x1) {
    sx = 1;
  }
  else {
    sx = -1;
  }
  if (y0 < y1) {
    sy = 1;
  }
  else {
    sy = -1;
  }

  // Initializes the steps.
  isDrawing = bresenham(); // Updates global variables dx, dy to either +1, -1 or 0 for a step or not.
  // Start loop with intervals
  while (isDrawing) {
    if (micros() - lastTime >= timePerStep) {
      //Serial.print("pulseOn, dx, dy: ");
      //String str1 = String(pulseOn) + "," + String(dx) + "," + String(dy);
      //Serial.println(str1);
      if (pulseOn) {
        lastTime = micros();
        pulseOn = !pulseOn; // Flips logic.
        digitalWrite(stepXpin, LOW);
        digitalWrite(stepYpin, LOW);
      }
      else {
        lastTime = micros();
        if (x_step < 0) {
          digitalWrite(dirXpin, LOW); // CCW
        }
        else {
          digitalWrite(dirXpin, HIGH); // CW
        }
        if (y_step < 0) {
          digitalWrite(dirYpin, LOW); // CCW
        }
        else {
          digitalWrite(dirYpin, HIGH); // CW
        }
        digitalWrite(stepXpin, abs(x_step)); // writes either a 1 (HIGH) or 0 (LOW) dependent on result from bresenham. dx can be +1, -1, 0.
        digitalWrite(stepYpin, abs(y_step));
        pulseOn = !pulseOn; // flips logic
        // Calculate next step while we wait for pulse to become low
        isDrawing = bresenham();
      }
    }
        
  } // END while (isDrawing)
  /*
  Serial.println("Finished line");
  Serial.print("Updatet stepper coordinates: x0,y0: ");
  Serial.print(x0);
  Serial.print(",");
  Serial.println(y0);
  */

}

bool bresenham() {
  // Calculates steps in x and y directions. 0, -1 or 1
  // Updates global variables dx, dy, and error variable err.
  err2 = 2 * err;
  x_step = 0;
  y_step = 0;

  if (x0 == x1 && y0 == y1) {
    return false;
  }
  if (err2 >= dy) {
    if (x0 == x1) {
      return false;
    }
    // Update step and error
    err = err + dy;
    x0 = x0 + sx;
    x_step = sx;
  }
  if (err2 <= dx) {
    if (y0 == y1) {
      return false;
    }
    err = err + dx;
    y0 = y0 + sy;
    y_step = sy;
  }
  //String str1 = "err, x_step, y_step, x0, y0: " + String(err) + "," + String(x_step) + "," + String(y_step) + "," + String(x0) + "," + String(y0);
  //Serial.println(str1);
  return true;

}

void drawCircle(float xcenter, float ycenter, float radius) {
  // In absolute coordinates, (0,0) upper left like a web page.
  liftPen();
  // Move to xy_start in absolute coordinates.
  moveHeadTo(xcenter+radius, ycenter); // NB! + radius of circle!
  // Move along circle of radius r.
  // 1) Divide circle into segments based on lengt of circumference and resolution limit.
  // Length og line segment = 5 mm for first try.
  float segment_length = 5;  // Default 24 for circle with r = 30 mm
  float n_segments = (2*PI*radius)/segment_length;
  byte n = ceil(n_segments);
  Serial.print("n_segments Circle: ");
  Serial.println(n);
  float d_theta = 2*PI/n;
  float theta = 0;

  // Initialize relative coordinates. The whole circle is shifted by xcenter and ycenter.
  float x_rel = radius;
  float y_rel = 0;
  float x_new = 0;
  float y_new = 0;


  // 2) Run for loop of n line segments.
  for (byte i=0; i < n; i++) {
    /*
    Serial.println("--------------");
    Serial.print("segment: ");
    Serial.print(i+1);
    Serial.print(": ");
    String positions = "x0,y0: " + String(x0) + "," + String(y0);
    Serial.println(positions);*/
    theta = theta + d_theta;
    x_new = radius * cos(theta) + xcenter;
    y_new = radius * sin(theta) + ycenter;
    //String target = "Target x_new,y_new: " + String(x_new) + "," + String(y_new);
    String target = String(x_new) + "," + String(y_new);
    //Serial.println(target);
    
    // Move using bresenham-stepper
    moveHeadTo(x_new, y_new);
    

    // Update integer coordinates
    //x0 = x_new;
    //y0 = y_new;
    

  }
  String str4 = "Head position after Circle: x0,y0: " + String(x0) + "," + String(y0);
  Serial.println(str4);
}

void moveHeadTo_old(float targetX, float targetY) {

  /*
   Moves to coordinate (x,y), then sets x0 and y0 to these values.
   1) Calculate longest distance.
   2) Calculate speed in each axis to reach destination at the same time.
   3) Move x and y in parallel using different speeds.
  */
  float dx = targetX - x0;
  float dy = targetY - y0;
  
  // Calculate distances, number of steps and timePerStep and save to global array.
  calcTimePerStep(dx, dy);

  int n_stepsX = stepperParams[0];
  int n_stepsY = stepperParams[1];
  timePerStepX = stepperParams[2];
  timePerStepY = stepperParams[3];

  String str1 = "Moves head to (x,y): (" + String(targetX) + "," + String(targetY) + ")";
  Serial.println(str1);
  String str2 = "dx, dy: " + String(dx) + "," + String(dy);
  Serial.println(str2);

  String str3 = "nx, ny, timePerStepX, timePerStepY: " + String(n_stepsX) + "," + String(n_stepsY) + "," + String(timePerStepX) + "," + String(timePerStepY);
  Serial.println(str3);

  runSteppers(n_stepsX, n_stepsY, timePerStepX, timePerStepY);  // nsteps, time per step in microseconds

  // Update real integer coordinates
  int dx_real = n_stepsX * stepLength;
  int dy_real = n_stepsY * stepLength;
  x0 = x0 + dx_real;
  y0 = y0 + dy_real;

  String str4 = "Head position: x,y: " + String(x0) + "," + String(y0);
  Serial.println(str4);
  
}

void drawCircle_old(float xcenter, float ycenter, float radius) {
  liftPen();
  // Move to xy_start.
  moveHeadTo(xcenter+radius, ycenter); // NB! + radius of circle!
  // Move along circle of radius r.
  // 1) Divide circle into segments based on lengt of circumference and resolution limit.
  // Length og line segment = 5 mm for first try.
  float segment_length = 30;  // Default 24 for circle with r = 30 mm
  float n_segments = (2*PI*radius)/segment_length;
  byte n = ceil(n_segments);
  Serial.print("n_segments Circle: ");
  Serial.println(n);
  float d_theta = 2*PI/n;
  float theta = 0;

  // Initialize x0 and y0
  x0 = radius;
  y0 = 0;


  // 2) Run for loop of n line segments.
  for (byte i=0; i < n; i++) {
    /*
    Serial.println("--------------");
    Serial.print("segment: ");
    Serial.print(i+1);
    Serial.print(": ");
    String positions = "x0,y0: " + String(x0) + "," + String(y0);
    Serial.println(positions);*/
    theta = theta + d_theta;
    x = radius * cos(theta);
    y = radius * sin(theta);
    /*String target = "Target x,y: " + String(x) + "," + String(y);
    Serial.println(target);*/
    dx = x-x0;
    dy = y-y0;
    // Calculate distances, number of steps and timePerStep and save to global array.
    calcTimePerStep(dx, dy);

    // Run steppers using speeds vx and vy which conrols the timing of the steppers.
    /*Serial.print("timePerStepX,timePerStepY: ");
    String temp2 = String(timePerStepX) + "," + String(timePerStepY);
    Serial.println(temp2);*/

    int n_stepsX = stepperParams[0];
    int n_stepsY = stepperParams[1];
    timePerStepX = stepperParams[2];
    timePerStepY = stepperParams[3];

    runSteppers(n_stepsX, n_stepsY, timePerStepX, timePerStepY);  // nsteps, time per step in microseconds

    // Update real integer coordinates
    int dx_real = n_stepsX * stepLength;
    int dy_real = n_stepsY * stepLength;
    x0 = x0 + dx_real;
    y0 = y0 + dy_real;
    

  }
  String str4 = "Head position after Circle: x,y: " + String(x0) + "," + String(y0);
  Serial.println(str4);
}

void calcTimePerStep(float dx, float dy) {
  /*
    Calculates distances, nsteps + timings.
    Saves to global array.
  */
  int n_stepsX = 0;
  int n_stepsY = 0;
  timePerStepX = 0;
  timePerStepY = 0;
  // Which axis is furthest away.
  if (abs(dx) > abs(dy)) {
    tx = abs(dx)/vmax;
    ty = tx;  // y-axis becomes slowest.
  }
  else if (abs(dx) < abs(dy)) {
    ty = abs(dy)/vmax;
    tx = ty;
  }
  else { // If equal they share the same time.
    //Serial.println("Equal!");
    if (abs(dx) <= 0.001) {
      n_stepsX = 0;
      tx = 0;
    }
    else if (abs(dy) <= 0.001) {
      n_stepsY = 0;
      ty = 0;
    }
    else {
      tx = abs(dx)/vmax;
      ty = tx;
    }
    
  }

  // Calculates n_steps in each axis
  /*Serial.print("dx,dy,tx,ty: ");
  String buffer = String(dx) + "," + String(dy) + "," + String(tx) + "," + String(ty);
  Serial.println(buffer);*/
  // check if movement:
  if (abs(dx) > 0.00001) {
    n_stepsX = round(dx / stepLength);
  }
  if (abs(dy) > 0.00001) {
    n_stepsY = round(dy / stepLength);
  }
  /*Serial.print("n_stepsX is now: ");
  Serial.println(n_stepsX);*/
  n_stepsY = round(dy / stepLength);

  // Calculates timePerStep in each axis
  /*float temp = (1000*tx)/float(n_stepsX);
  Serial.print("temp timePerStep (ms): ");
  Serial.println(temp);*/
  if (n_stepsX != 0) {
    timePerStepX = floor(1000*(1000*tx / float(n_stepsX)));
  }
  if (n_stepsY != 0) {    
    timePerStepY = floor(1000*(1000*ty / float(n_stepsY)));
  }

  // Sets global array
  stepperParams[0] = n_stepsX;
  stepperParams[1] = n_stepsY;
  stepperParams[2] = timePerStepX;
  stepperParams[3] = timePerStepY;

    
} // END calcTimePerStep()

/* Controls both stepper motors in parallell.
nx: number of steps in x-axis, negative value is counter clockwise.
ny: number of steps in y-axis, negative value is counter clockwise.
timePerStepX: time per step in microseconds for x-stepper.
timePerStepY: time per step in microseconds for ystepper.
*/ 
void runSteppers(int nx, int ny, float timePerStepX, float timePerStepY) {
  String dirX = "CW";
  String dirY = "CCW";
  int pauseX = int(round(timePerStepX/2));  // Divide by 2 because step signal is 2 x delayMicroseconds.
  int pauseY = int(round(timePerStepY/2)); 
  if (nx > 0) {
    digitalWrite(dirXpin, HIGH); // Set the spinning direction clockwise (x- and y-axis are wired opposite at the moment).
    dirX = "CW";
  }
  else if (nx < 0) {
    digitalWrite(dirXpin, LOW); // CCW
    dirX = "CCW";
  }
  else {
    dirX = "STILL";
  }
  if (ny > 0) {
    digitalWrite(dirYpin, LOW); // Set the spinning direction counter clockwise
    dirY = "CCW";
  }
  else if (ny < 0) {
    digitalWrite(dirYpin, HIGH); // CCW
    dirY = "CW";
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

