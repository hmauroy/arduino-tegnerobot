/*
 * A4988 driver running dual 28BYJ-48 stepper motors.
 * Using 1/4 stepping. Microstepping needs to be set on the driver. Pull MS2 HIGH, MS1 and MS3 
 * can be disconnected due to active LOW on the pins.
 * 2x2 push buttons for driving each motor in each direction.
 * 
 * 28BYJ-48 specs
 * 32 steps/revolution
 * Gear box: 1:64
 * Total 32*64 = 2048 steps per revolution
 * Half-stepping: 4096 steps/revolution
 * 1/4 stepping: 8192 steps/revolution
 * 
 */

// Define stepper motor connections and steps per revolution:
#define dirXpin 6
#define dirYpin 7
#define stepXpin 8
#define stepYpin 9

#define xpDirLed 4  // positive movement along x
#define xnDirLed 5  // negative movement

unsigned int stepsPerRevolution = 4*2048; // 1/4 stepping => 8192 steps
unsigned long timePerStep = 500;
unsigned long updateTime = 500; // Time between recalculation of Bresenham.
float pulleyRadius = 33.0;  // radius of pulleys on stepper motors. Determines linear speed.
float pulleyCircumference = 2*PI*pulleyRadius;
float stepLength = pulleyCircumference / stepsPerRevolution;
float vmax = 20.0/stepLength;  // px/s maximum speed of any axis.



// Variables for "Henriks algorithm"
float xspeed = 20.0; // mm/second. 20 mm/s is maximum linear speed.
float yspeed = 20.0;
bool lifted = false;
int stepperParams[] = {1024,1024,3000,3000};  // nx, ny, timePerStepX, timePerStepY
float tx = 0;
float ty = 0;

double timePerStepX = 6000;
double timePerStepY = 6000;

// Bresenhams algorithm variables
// Variables for positioning
int x0 = 0; // Position in pixel values for Bresenham.
int y0 = 0;
int x1 = 0; // Target position in pixel values for Bresenham.
int y1 = 0;
unsigned int x0_px = 0; // Pixel value of a coordinate
unsigned int y0_px = 0;
int xLast = 0; // Last position before movement using Bresenham. Used for calculation of direction change.
int yLast = 0; // Last position before movement using Bresenham.
double x = 0; // Position in px.
double y = 0; // Position in px.
unsigned int radius = 1;  // pixel coordinates 
int dx = 0;
int dy = 0;
int err = 0;
int err2 = 0;
bool isDrawing = true;
bool pulseOn = true;
int sx = 0;
int sy = 0;
unsigned long lastTime = micros(); // time in microseconds. overflows in around 70 min.
int x_step = 0;
int y_step = 0;
float xCalibration = 64.6;  // Distance for 5000 steps in x-direction.
float yCalibration = 62.0;  // Distance for 5000 steps in y-direction.
float stepLengthX = xCalibration / 5000; // 0.0129 mm/step
float stepLengthY = yCalibration / 5000; // 0.0124 mm/step
double vx = 0;
double vy = 0;
unsigned long txLast = micros();
unsigned long tyLast = micros();
unsigned long lastUpdate = micros();


// Bezier variables
// Defines a class (struct) that can hold multiple values.
struct Point {
    float x;
    float y;
};
Point controlPoints[4];
Point curvePoints[30]; // Array to store computed curve points




void setup() {
  // Set pins as output:
  pinMode(dirXpin, OUTPUT);
  pinMode(dirYpin, OUTPUT);
  pinMode(stepXpin, OUTPUT);
  pinMode(stepYpin, OUTPUT);
  pinMode(xpDirLed, OUTPUT);
  pinMode(xnDirLed, OUTPUT);
  
  Serial.begin(9600);
  Serial.println("");
  Serial.println("Startup waits 1000 ms...");
  Serial.println("*************************************");

  delay(1000);
  

  // Runs only once
  //Serial.println("Moves in a circle radius = 10!");
  //drawCircle(20,20,10);

  //Serial.println("Draws Bresenham circle");
  //drawCircle_bres(5000,5000,2500);

  

  int radius = 20;
  //Serial.print("Moves in a circle radius = ");
  //Serial.println(radius);

  for (int i=0; i<5; i++) {
    //drawCircle(50,50,radius); 
    //drawCircleVectorSpeed(50,50,radius);
  }

  //drawSine(50,50,radius);

  double sum1 = calculateIntervalSinewave(3.82, 0, 1, 1, 0);

  double sum2 = calculateLengthSinewave(0,PI,1000, 1, 1, 0);

  Serial.print("sum1 = ");
  Serial.println(sum1);

  Serial.print("sum2 = ");
  Serial.println(sum2);

  // drawSineInterval(0,10,50,10,0.5);
  // drawSineInterval(50,10,70,5,1.5);
  // drawSineInterval(70,10,100,10,1.5);
  // drawSineInterval(0,40,100,20,0.1);
  //drawSineLength(0,40,200,20,0.25);
  //drawSineLength(60,30,100,10,0.25);

  // Bezier
  
  drawBezierCurve("c 16 1 1 4 3 16 15 11");

  

  /*
  for (int i =0; i<5; i++) {
    moveHeadTo(0,60);
    moveHeadTo(60,60);
    moveHeadTo(60,0);
    moveHeadTo(0,0);
  }

  */
  
  /*
  for (int i =0; i<5; i++) {
    Serial.println("New square");
    moveHeadTo(40,1);
    moveHeadTo(40,40);
    moveHeadTo(1,40);
    moveHeadTo(0,0);
  }
  */
  
 
  /*
  // Absolute steps
  moveHeadTo(0,5000);
  moveHeadTo(5000,5000);
  moveHeadTo(5000,0);
  moveHeadTo(0,0);
  */


  

}

void loop() {
  

}
