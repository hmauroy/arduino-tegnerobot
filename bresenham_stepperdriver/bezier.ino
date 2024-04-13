/*
Code to parse control points for a cubic Bezier curve into coordinates along the Bezier curve.

*/


// Function to parse SVG path data into control points
void parseBezierPoints(const char* svgContent, Point points[4]) {
    char* token;
    char buffer[100];
    strcpy(buffer, svgContent); // Copy the string to a buffer we can modify
    double pixelFactor = 5 / (xCalibration / 5000);

    token = strtok(buffer, " ,"); // Skip 'c' or 'C'
    for (int i = 0; i < 8; i += 2) {
        token = strtok(NULL, " ,");
        points[i / 2].x = pixelFactor * atof(token);
        token = strtok(NULL, " ,");
        points[i / 2].y = pixelFactor * atof(token);
    }
}

// Function to generate points along a cubic Bézier curve using De Casteljau's algorithm:
// https://en.wikipedia.org/wiki/De_Casteljau%27s_algorithm
void generateBezierPoints(Point controlPoints[4], Point result[], int numberOfPoints) {
    for (int i = 0; i <= numberOfPoints; i++) {
        float t = i / float(numberOfPoints);
        float a = (1.0 - t) * (1.0 - t) * (1.0 - t);
        float b = 3.0 * t * (1.0 - t) * (1.0 - t);
        float c = 3.0 * t * t * (1.0 - t);
        float d = t * t * t;

        result[i].x = a * controlPoints[0].x + b * controlPoints[1].x + c * controlPoints[2].x + d * controlPoints[3].x;
        result[i].y = a * controlPoints[0].y + b * controlPoints[1].y + c * controlPoints[2].y + d * controlPoints[3].y;
    }
}

void drawBezierCurve(char* path_data) {
  // Takes path_data as a char* string, e.g. "c 16 1 1 4 3 16 15 11"
  parseBezierPoints(path_data, controlPoints);
  generateBezierPoints(controlPoints, curvePoints, 29);
  double x, y;
  for (int i=0; i<30; i++) {
    x = curvePoints[i].x;
    y = curvePoints[i].y;
    String info = "x,y: " + String(x) + "," + String(y);
    Serial.println(info);
    moveHeadTo(x,y);
  }
}