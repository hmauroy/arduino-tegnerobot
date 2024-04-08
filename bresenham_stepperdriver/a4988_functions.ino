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

void moveHeadTo(float targetX, float targetY) {
  // Wrapper function that will be the one always responsible for movement to absolute position in mm coordinates.
  runSteppersBres(round(targetX), round(targetY));
}

void moveHeadTo_2(float targetX, float targetY) {
  // Wrapper function that will be the one always responsible for movement to absolute position in mm coordinates.
  double x_distance = targetX - x;
  x = targetX;
  double y_distance = targetY - y;
  y = targetY;

  runSteppersBres(targetX, targetY);
}

void runSteppersBres(float targetX, float targetY) {
  // TargetX and targetY are in pixel-coordinates
  // Coordinate system is 0,0 upper left!
  x1 = int(targetX);
  y1 = int(targetY);
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
    if (micros() - lastTime >= updateTime) {
      //Serial.print("pulseOn, dx, dy: ");
      //String str1 = String(pulseOn) + "," + String(dx) + "," + String(dy);
      //Serial.println(str1);
      if (pulseOn) {
        lastTime = micros();
        pulseOn = !pulseOn; // Flips logic.
        digitalWrite(stepXpin, LOW);
        digitalWrite(stepYpin, LOW);
        digitalWrite(xpDirLed, LOW);
        digitalWrite(xnDirLed, LOW);
      }
      else {
        lastTime = micros();
        if (x_step < 0) {
          digitalWrite(dirXpin, HIGH); // CW
          digitalWrite(xnDirLed, HIGH);
        }
        else {
          digitalWrite(dirXpin, LOW); // CCW
          digitalWrite(xpDirLed, HIGH);
        }
        if (y_step < 0) {
          digitalWrite(dirYpin, LOW); // CCW
        }
        else {
          digitalWrite(dirYpin, HIGH); // CW
        }
        if (x_step == 0){
          digitalWrite(xpDirLed, LOW);
          digitalWrite(xnDirLed, LOW);
        }
        //digitalWrite(stepXpin, abs(x_step)); // writes either a 1 (HIGH) or 0 (LOW) dependent on result from bresenham. dx can be +1, -1, 0.
        //digitalWrite(stepYpin, abs(y_step));
        pulseOn = !pulseOn; // flips logic
        
        // Pulse steppers multiple times for one pixel movement. Needs microstepping.
        pulseSteppers(abs(x_step), abs(y_step), 1);
        // Calculate next step while we wait for next update.
        isDrawing = bresenham();
      }
    }
        
  } // END while (isDrawing)

  //x = x + (dx / stepLength);
  
  
  //Serial.println("Finished line");
  
  /*
  Serial.print("Updatet stepper coordinates: x0,y0: ");
  Serial.print(x0);
  Serial.print(",");
  Serial.println(y0);
  
  Serial.print("Updatet absolute coordinates: x,y: ");
  Serial.print(x);
  Serial.print(",");
  Serial.println(y);

  */

}

void pulseSteppers(bool x, bool y, int n) {
  for (int i=0; i<n; i++) {
    digitalWrite(stepXpin, x);
    digitalWrite(stepYpin, y);
    delayMicroseconds(50);
    digitalWrite(stepXpin, LOW);
    digitalWrite(stepYpin, LOW);
    // Sleep for pulse off time
    delayMicroseconds(timePerStep);
  }
  
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

void setAbsoluteCoordinates(float x_mm, float y_mm) {
  // Sets pixel values x0 and y0 from millimeter coordinates.
  float x_steps = x_mm / (xCalibration / 5000);
  x0 = round(x_steps);
  float y_steps = y_mm / (yCalibration / 5000);
  y0 = round(y_steps);
  String str4 = "Head position after manual override: x0,y0: " + String(x0) + "," + String(y0);
  Serial.println(str4);
}

void drawCircle(float xcenter, float ycenter, float r) {
  // In absolute coordinates, (0,0) upper left like a web page.
  liftPen();
  // Move to xy_start in absolute coordinates.
  //moveHeadTo(xcenter+radius, ycenter); // NB! + radius of circle!
  
  // For testing purposes, set absolute pixel positions manually to start drawing directly from start position.
  //setAbsoluteCoordinates(xcenter+r,ycenter);
  
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
  unsigned int x0_px = round(xcenter / (xCalibration / 5000)); // Pixel-position of center of circle.
  unsigned int y0_px = round(ycenter / (yCalibration / 5000));
  radius = round(r / (xCalibration / 5000));
  x0 = x0_px + radius;
  y0 = y0_px;
  String target = "x0,y0: " + String(x0) + "," + String(y0);
  Serial.println(target);


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
    x = radius * cos(theta) + x0_px;
    y = radius * sin(theta) + y0_px;
    //String target = "Target x_new,y_new: " + String(x_new) + "," + String(y_new);
    String target = String(x) + "," + String(y);
    //Serial.println(target);
    
    // Move using bresenham-stepper
    moveHeadTo(x, y);
    

    // Update integer coordinates
    //x0 = x_new;
    //y0 = y_new;
    

  }
  String str4 = "Head position after Circle: x0,y0: " + String(x0) + "," + String(y0);
  Serial.println(str4);
}



void drawCircleVectorSpeed(float xcenter, float ycenter, float r) {
  // In absolute coordinates, (0,0) upper left like a web page.
  liftPen();
  // Move to xy_start in absolute coordinates.
  //moveHeadTo(xcenter+radius, ycenter); // NB! + radius of circle!
  
  // For testing purposes, set absolute pixel positions manually to start drawing directly from start position.
  //setAbsoluteCoordinates(xcenter+r,ycenter);
  
  // Move along circle of radius r.
  // 1) Divide circle into segments based on lengt of circumference and resolution limit.
  // Length og line segment = 5 mm for first try.
  float segment_length = 5;  // Default 24 for circle with r = 30 mm
  float n_segments = (2*PI*r)/segment_length;
  byte n = ceil(n_segments);
  Serial.print("n_segments Circle: ");
  Serial.println(n);
  double v = 20.0;
  double dt = 1000 * 1000 * 2*PI*r / v; // microseconds
  double t = 0;

  // Initialize x0 and y0, radius as pixel values. NB! 1/16-stepping needs factor 4 in number of pixels.
  unsigned int x0_px = round(xcenter / (xCalibration / 5000)); // Pixel-position of center of circle.
  unsigned int y0_px = round(ycenter / (yCalibration / 5000));
  radius = round(r / stepLengthX);
  x0 = x0_px + radius;
  y0 = y0_px;
  String target = "x0,y0: " + String(x0) + "," + String(y0);
  //Serial.println(target);
  Serial.print("dt:");
  Serial.println(abs(dt));

  // Run a timer loop which updates the speed every dt seconds.
  // Calculate speed in x- and y-direction from speedvectors while running a loop
  /*
    Example circle 1:
    r = 30 mm
    v = 20 mm/s
    n = 38 segments (updates)
    vx(t) = -v*sin(v/r * t) = -20*sin(2*t / 3)
    vy(t) = v*cos(v/r * t) = 20*cos(2*t / 3)
  */
  bool isRunning = true;
  bool sigX = true; // pulse ON or OFF
  bool sigY = true;
  //dt = 0.01;
  while (isDrawing) {
    if (micros() - lastUpdate >= abs(dt)) {
      lastUpdate = micros();
      t = t + (dt/1000000);
      //Serial.println("Updates speeds");
      vx = -v*sin(v*t / r); // mm/s
      vy = v*cos(v*t / r);
      // Update timings for each speed
      double vx_px = vx / stepLengthX; // mm/s / mm/step = steps/s
      timePerStepX = 1000000 / vx_px;  // 1000000us / steps/s = s/step
      double vy_px = vy / stepLengthY; // mm/s / mm/step = steps/s
      timePerStepY = 1000000 / vy_px;  // 1000000us / steps/s = us/step
      //String speedInfo = "vx, vy, timeX, timeY: " + String(vx) + "," + String(vy) + "," + String(timePerStepX) + "," + String(timePerStepY);
      //String speedInfo = "vy, timeY: " + String(vy) + "," + String(timePerStepY);
      //Serial.println(speedInfo);
      // Update directions
      if (vx < 0) {
          digitalWrite(dirXpin, HIGH); // CW
        }
        else {
          digitalWrite(dirXpin, LOW); // CCW
          
        }
        if (vy < 0) {
          digitalWrite(dirYpin, LOW); // CCW
          digitalWrite(xpDirLed, LOW);
          digitalWrite(xnDirLed, HIGH);
        }
        else {
          digitalWrite(dirYpin, HIGH); // CW
          digitalWrite(xpDirLed, HIGH);
          digitalWrite(xnDirLed, LOW);
        }
        if (vx == 0){
          //digitalWrite(xpDirLed, LOW);
          digitalWrite(xnDirLed, LOW);
          digitalWrite(xpDirLed, LOW);
        }
        if (vy == 0){
          //digitalWrite(xpDirLed, LOW);
          digitalWrite(xnDirLed, LOW);
          digitalWrite(xpDirLed, LOW);
        }

    }
    if (micros() - txLast >= abs(timePerStepX)) {
      txLast = micros();
      sigX = !sigX; // Invert signal
      digitalWrite(stepXpin, sigX); // turn ON or OFF stepPin.
    }
    if (micros() - tyLast >= abs(timePerStepY)) {
      tyLast = micros();
      sigY = !sigY;
      digitalWrite(stepYpin, sigY);

    }

  }

  //String str4 = "Head position after Circle: x0,y0: " + String(x0) + "," + String(y0);
  //Serial.println(str4);
}

void drawCircle_bres(int centerX, int centerY, int radius) {
            int x = centerX;
            int y = centerY + radius;
            int d = 3 - (2 * radius);

            int x_old = x;
            int y_old = y;

            bool x_puls = false;
            bool y_puls = false;

            // Set directions along positive directions, (0,0) is upper left.
            digitalWrite(dirXpin, HIGH); // CW
            digitalWrite(dirYpin, LOW); // CCW


            while (x <= y) {
                x_puls = false;
                y_puls = false;
                // Move stepper motors
                String str1 = "x,y: " + String(x) + "," + String(y);
                Serial.println(str1);
                if (x != x_old) {
                  x_old = x;
                  x_puls = true;
                }
                if (y != y_old) {
                  y_old = y;
                  y_puls = true;
                }
                pulseSteppers(x_puls, y_puls, 8);

                /*
                // Draw 8 octants
                ctx.fillRect(centerX + x, centerY + y, 1, 1); // Octant 1
                ctx.fillRect(centerX - x, centerY + y, 1, 1); // Octant 2
                ctx.fillRect(centerX + x, centerY - y, 1, 1); // Octant 8
                ctx.fillRect(centerX - x, centerY - y, 1, 1); // Octant 7
                ctx.fillRect(centerX + y, centerY + x, 1, 1); // Octant 3
                ctx.fillRect(centerX - y, centerY + x, 1, 1); // Octant 4
                ctx.fillRect(centerX + y, centerY - x, 1, 1); // Octant 6
                ctx.fillRect(centerX - y, centerY - x, 1, 1); // Octant 5
                */

                // Update coordinates based on Bresenham's algorithm
                x += 1;
                if (d > 0) {
                    y -= 1;
                    d = d + 4 * (x - y) + 10;
                } else {
                    d = d + 4 * x + 6;
                }                
            }
        }


