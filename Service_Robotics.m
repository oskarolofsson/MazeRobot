#include <QTRSensors.h>

int svart = 80;
int gray = 500;
int vit = 100;

// Wheel Servo
#include <Servo.h>
Servo rightWheel;  // servo object right wheel
Servo leftWheel;   // servo object left wheel
int rightDrive = 0; // right wheel actuator
int leftDrive = 0; // left wheel actuator

// Grab Servo
#include <Servo.h> // for all servos
Servo grab;    // servo object for grab function
int grabAct = 0;  // grab actuator

// Lift Servo
Servo lift;   // servo object for lift function
int liftAct = 0;  // lift actuator

// Ultrasound Sensor
#define echoPinF 2 // echo pin D2
#define trigPinF 3 // trig pin D3 
double durationF;    // duration of sound wave travel
double distanceF;     // measured distance

#define echoPinL 12 // echo pin D12
#define trigPinL 13 // trig pin D13 
long durationL;    // duration of sound wave travel
int distanceL;     // measured distance

//Line Sensor Matrix
QTRSensors qtr;
const uint8_t SensorCount = 5; //Digital antal
uint16_t sensorValues[SensorCount];
int l1;                 // sensor value of input
int l2;                 // sensor value of input
int r2;                 // sensor value of input
int r1;                 // sensor value of input

void setup() {

  // Line Sensor Matrix Calibration
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]) {4, 5, A0, 7, 8}, SensorCount);
  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode
  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  Serial.begin(9600); // Baud rate for serial monitor

  // Wheel Servo
  rightWheel.attach(10);  // attaches the right wheel servo object to pin 11
  leftWheel.attach(11);   // attaches the left wheel servo object to pin 10
  // Grab and Lift servo
  grab.attach(9);  // attaches the right wheel servo object to pin 11
  lift.attach(6);   // attaches the left wheel servo object to pin 10
  // Ultrasound
  pinMode(echoPinF, INPUT);  // Sets the echoPin as an INPUT
  pinMode(trigPinF, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPinL, INPUT);  // Sets the echoPin as an INPUT
  pinMode(trigPinL, OUTPUT); // Sets the trigPin as an OUTPUT
  grab.write(75);
  lift.write(20);
}

//Get reflectance sensor values
void lineMatrix() {
  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 5000 (for a white line, use readLineWhite() instead)
  qtr.readLineBlack(sensorValues);
  l1 = sensorValues[0];
  l2 = sensorValues[1];
  m  = sensorValues[2];
  r2 = sensorValues[3];
  r1 = sensorValues[4];

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum
  // reflectance and 1000 means minimum reflectance, followed by the line
  // position
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(" ");
}

//Get values from front ultrasonic sensor
void ultraSoundF() {
  digitalWrite(trigPinF, LOW);         // clears the trigPin condition
  delayMicroseconds(2);
  digitalWrite(trigPinF, HIGH);        // sets the trigPin HIGH (ACTIVE) for 10 microseconds
  delayMicroseconds(10);
  digitalWrite(trigPinF, LOW);         // sets the trigPin LOW (DEACTIVATED)
  durationF = pulseIn(echoPinF, HIGH);  // reads the echoPin, returns the sound wave travel time in microseconds
  distanceF = durationF * 0.034 / 2;    // calculating the distance, (Speed of sound wave divided by 2 (go and back))
  if (distanceF > 1100) {
    distanceF = 3;
  }
  // displays the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distanceF);             // distance
  Serial.println(" cm");              // unit
}

//Get values from left ultrasonic sensor
void ultraSoundL() {
  digitalWrite(trigPinL, LOW);         // clears the trigPin condition
  delayMicroseconds(2);
  digitalWrite(trigPinL, HIGH);        // sets the trigPin HIGH (ACTIVE) for 10 microseconds
  delayMicroseconds(10);
  digitalWrite(trigPinL, LOW);         // sets the trigPin LOW (DEACTIVATED)
  durationL = pulseIn(echoPinL, HIGH);  // reads the echoPin, returns the sound wave travel time in microseconds
  distanceL = durationL * 0.034 / 2;    // calculating the distance, (Speed of sound wave divided by 2 (go and back))
  if (distanceL > 1100) {
    distanceL = 3;
  }
  // displays the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distanceL);             // distance
  Serial.println(" cm");              // unit
}

// Regulates the position of the robot with reflectance sensor
void regMatrix() {
  if (l2 > svart) {
    leftDrive = 80;
    leftWheel.write(leftDrive);
    delay(50);
  } else if (r2 > svart) {
    rightDrive = 100;
    rightWheel.write(rightDrive);
    delay(50);
  }
}

// Regulates the position of the robot with left ultrasonic sensor
void regUltra() {
  if (distanceL > 10) {
    leftDrive = 80; 
    leftWheel.write(leftDrive);
    delay(50);
  } else if (distanceL < 8) {
    rightDrive = 100;
    rightWheel.write(rightDrive);
    delay(50);
  }
}

// Turns left until line found again
void turnLeft() {
  delay(500);
  rightDrive = 135;             //speed of right wheel 
  leftDrive = 135;              //speed of left wheel  
  rightWheel.write(rightDrive); // set right wheel speed
  leftWheel.write(leftDrive); // set left wheel speed
  delay(250);
  lineMatrix();
  while (l2 < svart) {
    lineMatrix();
  }
}

// Turns right until line found again
void turnRight() {
  delay(500);
  rightDrive = 45;             //speed of right wheel
  leftDrive = 45;              //speed of left wheel
  rightWheel.write(rightDrive); // set right wheel speed
  leftWheel.write(leftDrive); // set left wheel speed
  delay(250);
  lineMatrix();
  while (r2 < svart) {
    lineMatrix();
  }
}

//Sequence for grabbing and lifting a cylinder
void grabAndLift() {

  // lower
  for (liftAct = 20; liftAct <= 150; liftAct += 1) { // 120-0 degrees, in steps of 1 degree
    lift.write(liftAct);              // set grip servo position
    delay(15);                       // waits 10ms and updates
  }

  // grab
  for (grabAct = 75; grabAct <= 125; grabAct += 1) { // 0-90 degrees, in steps of 1 degree
    grab.write(grabAct);              // set grip servo position
    delay(15);                       // waits 10ms and updates
  }

  // lift
  for (liftAct = 150; liftAct >= 20; liftAct -= 1) { // 0-120 degrees, in steps of 1 degree
    lift.write(liftAct);              // set grip servo position
    delay(15);                       // waits 10ms and updates
  }

  // open
  for (grabAct = 125; grabAct >= 75; grabAct -= 1) { // 90-0 degrees, in steps of 1 degree
    grab.write(grabAct);              // set grip servo position
    delay(15);                       // waits 10ms and updates
  }
}

//Algorithm to make right turns when line is removed
void lostLine() {
  while ((l2 > 200) || (r2 > 200)) {
    forward();
  }
  delay(1000);
  while (distanceF > 12) {
    forwardU();
  }
  rightDrive = 45;             //speed of right wheel
  leftDrive = 45;              //speed of left wheel
  rightWheel.write(rightDrive); // set right wheel speed
  leftWheel.write(leftDrive); // set left wheel speed
  delay(850);
  while ((l2 < gray) && (r2 < gray)) {
    forwardU();
  }
}

//Takes the robot through the maze
void borja() {
  ultraSoundF();
  ultraSoundL();
  lineMatrix();
  forward();
  right();
  left();
  right();
  right();
  while (r1 < 700) {	// Wait for right turn, make a u-turn
    forward();
  }
  turnLeft();
  left();
  left();
  right();
  left();
  uTurn();
  forward();
  delay(300);
  right();
  left();
  left();
  right();
  uTurn();
  left();
  left();
  lostLine();
  right();
  right();
  left();
  left();
  uTurn();
  left();
  left();
  right();
  uTurn();
  left();
  left();
  left();
  right();
  left();
  left();
  uTurn();
  forward();
  delay(300);
  right();
  right();
  left();
  while (r1 < 700) {	// Wait for right turn, drive forward
    forward();
  }
  forward();
  delay(500);
  lineMatrix();
  right();
  right();
  uTurn();
  left();
  while (l1 < 700) {	// Wait for left turn, drive forward
    forward();
  }
  forward(); 
  delay(500); 
  lineMatrix();
  left();
  right();
  uTurn();
  left();
  left();
  right();
  right();
  right();
  lostLine();
  left();
  left();
  uTurn();
  forward();
  delay(300);
  right();
  right();
  left();
  left();
  while(l2 > 150 || r2 > 150) {	// Wait for end line to disappear
    forward();
  }
  stanna();
}

//Drives forward with regulation from reflectance sensor
void forward() {
  regMatrix();
  rightDrive = 180;
  leftDrive = 0;
  rightWheel.write(rightDrive);
  leftWheel.write(leftDrive);
  lineMatrix();
  ultraSoundF();

//Check for cylinders
  if (distanceF < 5) {
    stanna();
    grabAndLift();
    start();
  }
}

//Drives forward with regulation from left ultrasonic sensor
void forwardU() {
  regUltra();
  rightDrive = 180;
  leftDrive = 0;
  rightWheel.write(rightDrive);
  leftWheel.write(leftDrive);
  ultraSoundL();
  ultraSoundF();
  lineMatrix();
}

//Waiting for right turn, turning right when criteria is satisfied
void right() {
  while (r1 < 700) {	// höger
    forward();
    lineMatrix();
  }
  turnRight();
  lineMatrix();
}

//Waiting for left turn, turning left when criteria is satisfied
void left() {
  while (l1 < 700) {
    forward();
    lineMatrix();
  }
  turnLeft();
  lineMatrix();
}

// Makes a u-turn when the line disappears
void uTurn() {
  while ((l2 > 100) || (r2 > 100)) {
    forward();
  }
  turnRight();
}

void stanna() {
  rightWheel.detach();
  leftWheel.detach();
}

void start() {
  rightWheel.attach(10);
  leftWheel.attach(11);
}

void loop() {
  borja();
}
