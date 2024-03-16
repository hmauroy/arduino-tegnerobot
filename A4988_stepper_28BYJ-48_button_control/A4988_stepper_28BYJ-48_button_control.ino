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
#define buttonXpin 2
#define buttonXNpin 3 // X negative
#define buttonYpin 4
#define buttonYNpin 5 // Y negative
// positive and negative buttons
byte xp = 0;
byte xn = 0;
byte yp = 0;
byte yn = 0;
unsigned int stepsPerRevolution = 4*2048; // 1/4 stepping => 8192 steps
float timePerStepBuffer = 500.0;
unsigned int timePerStep = 500;

void setup() {
  // Set pins as output:
  pinMode(dirXpin, OUTPUT);
  pinMode(dirYpin, OUTPUT);
  pinMode(stepXpin, OUTPUT);
  pinMode(stepYpin, OUTPUT);
  // Button pins
  pinMode(buttonXpin, INPUT);
  pinMode(buttonXNpin, INPUT);
  pinMode(buttonYpin, INPUT);
  pinMode(buttonYNpin, INPUT);
  
  Serial.begin(9600);
  Serial.println("press push buttons to move motors");
}

void loop() {
  // check if the pushbuttons are pressed.
  xp = digitalRead(buttonXpin);
  xn = digitalRead(buttonXNpin);
  yp = digitalRead(buttonYpin);
  yn = digitalRead(buttonYNpin);
  
  // if they are, the buttonState is HIGH:
  if ((xp == HIGH) && (xn == HIGH) || (yp == HIGH) && (yn == HIGH) )  //If both X buttons are pushed simultaneously
  {
    // Stop the motors.
    stopMotors();
  }
  else if (xp == HIGH) {
    stepperX(1,6000);  // direction, time per rotation in milliseconds.
  }
  else if (xn == HIGH) {
    stepperX(0,6000); // direction, time per rotation in milliseconds.
  }
  else if (yp == HIGH) {
    stepperY(0,6000);  // direction, time per rotation in milliseconds.
  }
  else if (yn == HIGH) {
    stepperY(1,6000); // direction, time per rotation in milliseconds.
  }
  else {
    stopMotors();
  }

}
