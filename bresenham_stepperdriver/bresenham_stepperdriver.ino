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

unsigned int stepsPerRevolution = 4*2048; // 1/4 stepping => 8192 steps
float timePerStepBuffer = 500.0;
unsigned long timePerStep = 500;
float pulleyRadius = 33.0;  // radius of pulleys on stepper motors. Determines linear speed.
float pulleyCircumference = 2*PI*pulleyRadius;
float stepLength = pulleyCircumference / stepsPerRevolution;
float vmax = 20.0;  // mm/s maximum speed of any axis.



// Variables for "Henriks algorithm"
float xspeed = 20.0; // mm/second. 20 mm/s is maximum linear speed.
float yspeed = 20.0;
bool lifted = false;
int stepperParams[] = {1024,1024,3000,3000};  // nx, ny, timePerStepX, timePerStepY
float tx = 0;
float ty = 0;

float timePerStepX = 6000;
float timePerStepY = 6000;

// Bresenhams algorithm variables
// Variables for positioning
int x0 = 0;
int y0 = 0;
int x1 = 0;
int y1 = 0;
int x = 0;
int y = 0;
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





void setup() {
  // Set pins as output:
  pinMode(dirXpin, OUTPUT);
  pinMode(dirYpin, OUTPUT);
  pinMode(stepXpin, OUTPUT);
  pinMode(stepYpin, OUTPUT);
  
  Serial.begin(9600);
  Serial.println("");
  Serial.println("Startup waits 3000 ms...");
  Serial.println("*************************************");

  delay(3000);
  

  // Runs only once
  //Serial.println("Moves in a circle radius = 10!");
  //drawCircle(20,20,10);

  //Serial.println("Moves in a circle radius = 20!");
  //drawCircle(0,0,20);

  int radius = 30;
  Serial.print("Moves in a circle radius = ");
  Serial.print(radius);
  drawCircle(0,0,radius);

  /*
  for (int i =0; i<5; i++) {
    moveHeadTo(0,60);
    moveHeadTo(60,60);
    moveHeadTo(60,0);
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

  /*
  moveHeadTo(-20,-20);
  moveHeadTo(-80,-20);
  moveHeadTo(-80,-80);
  moveHeadTo(-20,-20);
  */

}

void loop() {
  

}
