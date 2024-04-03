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
  double x_steps = targetX / (xCalibration / 10000);
  double y_steps = targetY / (yCalibration / 10000);
  x = round(x_staps);
  y = round(y_steps);
  runSteppersBres(x, y);
}

void runSteppersBres(int targetX, int targetY) {
  // TargetX og targetY er i pixelverdier!!!!!!!
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
        digitalWrite(stepXpin, abs(x_step)); // writes either a 1 (HIGH) or 0 (LOW) dependent on result from bresenham. dx can be +1, -1, 0.
        digitalWrite(stepYpin, abs(y_step));
        pulseOn = !pulseOn; // flips logic
        delayMicroseconds(5);
        // Turns off the pulse after 50 microseconds
        digitalWrite(stepXpin, LOW);
        digitalWrite(stepYpin, LOW);
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

void setAbsoluteCoordinates(float x_mm, float y_mm) {
  // Sets pixel values x0 and y0 from millimeter coordinates.
  float x_steps = x_mm / (xCalibration / 5000);
  x0 = round(x_steps);
  float y_steps = y_mm / (yCalibration / 5000);
  y0 = round(y_steps);
  String str4 = "Head position after manual override: x0,y0: " + String(x0) + "," + String(y0);
  Serial.println(str4);
}

void drawCircle(float xcenter, float ycenter, float radius) {
  // In absolute coordinates, (0,0) upper left like a web page.
  liftPen();
  // Move to xy_start in absolute coordinates.
  //moveHeadTo(xcenter+radius, ycenter); // NB! + radius of circle!
  
  // For testing purposes, set absolute pixel positions manually to start drawing directly from start position.
  setAbsoluteCoordinates(xcenter+radius,ycenter);
  
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


