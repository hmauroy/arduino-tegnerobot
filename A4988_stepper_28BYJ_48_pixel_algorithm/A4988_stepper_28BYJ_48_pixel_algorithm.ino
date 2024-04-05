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
float timePerStepBuffer = 500;
unsigned int timePerStep = 500;
float pulleyRadius = 33.0;  // radius of pulleys on stepper motors. Determines linear speed.
float pulleyCircumference = 2*PI*pulleyRadius;
float stepLength = 62.0 / 5000; // Calibrated: mm/step. ca. 62mm/5000 steps ved 1/4-stepping
float vmax = 20.0 / stepLength;  // mm/s maximum speed of any axis.

// Variables for positioning
float x0 = 0;
float y0 = 0;
float x = 0;
float y = 0;
unsigned int radius = 1;  // pixel coordinates 
float xspeed = 20.0; // mm/second. 20 mm/s is maximum linear speed.
float yspeed = 20.0;
bool lifted = false;

float dx = 0;
float dy = 0;
double tx = 0;
double ty = 0;
int n_stepsX = 0;
int n_stepsY = 0;

double timePerStepX = 6000;
double timePerStepY = 6000;


void setup() {
  // Set pins as output:
  pinMode(dirXpin, OUTPUT);
  pinMode(dirYpin, OUTPUT);
  pinMode(stepXpin, OUTPUT);
  pinMode(stepYpin, OUTPUT);
  
  Serial.begin(9600);
  Serial.println("");
  Serial.println("Startup waits 1000 ms...");
  Serial.println("*************************************");

  delay(1000);
  
 /*
  // Runs only once
  int radius = 10;
  Serial.print("Moves in a circle radius = ");
  Serial.print(radius);
  drawCircle(20,20,radius);

  radius = 20;
  Serial.print("Moves in a circle radius = ");
  Serial.print(radius);
  drawCircle(0,0,20);

  radius = 30;
  Serial.print("Moves in a circle radius = ");
  Serial.print(radius);
  drawCircle(0,0,radius);
*/
  float r = 30;  // mm coordinates as input
  Serial.print("Moves in a circle radius = ");
  Serial.println(r);

  for (int i=0; i<5; i++) {
    //drawCircle(50,50,r);
  }

  // moveHeadTo(0,80);
  // moveHeadTo(80,80);
  // moveHeadTo(80,0);
  // moveHeadTo(0,0);

  /*
  moveHeadTo(-20,-20);
  moveHeadTo(-80,-20);
  moveHeadTo(-80,-80);
  moveHeadTo(-20,-20);
  */

  Serial.println("Waits for comma sep input: n_stepsX,n_stepsY,pulse_length");

}

void loop() {

  // if there's any serial available, read it:
  while (Serial.available() > 0) {

    // look for the next valid comma separated integer in the incoming serial stream:
    n_stepsX = Serial.parseInt();
    n_stepsY = Serial.parseInt();
    int pulse_length = Serial.parseInt();
    //float speed = Serial.parseFloat();

    // look for the newline. That's the end of your sentence:
    if (Serial.read() == '\n') {
      Serial.println("Got message!");
      moveHeadTo(n_stepsX,n_stepsY,pulse_length);
      
    }
  }
  

}






















