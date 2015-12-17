#include <ArduinoRobot.h>
#include <Wire.h>
#include <TimerOne.h>

//define states
#define STATE_READY 0
#define STATE_RANDOM_WANDER 1
#define STATE_WANDER_AVOID 2
#define STATE_FOLLOW_WALL 3

float frontSensorValue;
float leftSensorValue;
float rightSensorValue;
float backSensorValue;
int frontSensorPin = TKD1;
int leftSensorPin = TK4;
int rightSensorPin = TK0;
int backSensorPin = TK6;
int ftSonarPin = TKD1;

int frontRightSensorPin = TK1;
int frontLeftSensorPin = TK3;

int wanderCount = 0;

int state = STATE_FOLLOW_WALL;

int defaultMotorSpeed = 150;
int leftMotorSpeed = 0;
int rightMotorSpeed = 0;

float spinDegreesPerMilisecond = (168 / 90) * 0.325;

int frontSensorDistance;
int leftSensorDistance;
int rightSensorDistance;
int backSensorDistance;

int targetWallDistance = 20; //cm
int lastError;

int startTime;
int lastTime;
int timeStep = 1;
float kP = 0.1;
float kD = 100;
float motorDifferential;

int frontLeftSensorDistance;
int frontRightSensorDistance;

int obstacleDistanceThreshold = 20;
int obstacleDistanceFarThreshold = 50;

int potentialFieldAngle = 0;
int potentialFieldMagnitude = 0;
int potentialFieldThreshold = 40;

int keyPressed;



void setup() {
  Robot.begin();
  Robot.beginTFT();
  //DisplayMenu();
  Timer1.initialize(250000);
  Timer1.attachInterrupt(UpdateMotorSpeeds);
  Timer1.attachInterrupt(FollowWall);
  //Serial.begin(9600);
  startTime = millis();
  lastTime = startTime;
}

void loop() {
  keyPressed = Robot.keyboardRead();
  if (keyPressed == BUTTON_LEFT) {
    //state = STATE_SHY_KID;
  } else if (keyPressed == BUTTON_DOWN) {
    //state = STATE_AGGRESSIVE_KID_CHARGE;
  } else if (keyPressed == BUTTON_RIGHT) {
    state = STATE_RANDOM_WANDER;
  } else if (keyPressed == BUTTON_UP) {
    state = STATE_WANDER_AVOID;
  } else if (keyPressed == BUTTON_MIDDLE) {
    state = STATE_READY;
    rightMotorSpeed = 0;
    leftMotorSpeed = 0;
    Robot.motorsWrite(rightMotorSpeed, leftMotorSpeed);
  }


  Robot.stroke(0, 0, 0);
  /*
    Robot.text("left sensor reading", 3, 80);
    Robot.text("right sensor reading", 3, 100);
    Robot.text("back sensor reading", 3, 120);
    Robot.text("front sensor reading", 5, 60);
  */


  Robot.fill(255, 255, 255);
  Robot.stroke(255, 255, 255);


  ////read digital sensor

  pinMode(ftSonarPin, OUTPUT);//set the PING pin as an output
  Robot.digitalWrite(ftSonarPin, LOW); //set the PING pin low first
  delayMicroseconds(2);//wait 2 us
  Robot.digitalWrite(ftSonarPin, HIGH);//trigger sonar by a 2 us HIGH PULSE
  delayMicroseconds(5);//wait 5 us
  Robot.digitalWrite(ftSonarPin, LOW);//set pin low first again
  pinMode(ftSonarPin, INPUT);//set pin as input with duration as reception time
  frontSensorValue = pulseIn(ftSonarPin, HIGH); //measures how long the pin is high
  float frontSensorDistanceTemp = 0.0236 * frontSensorValue + 0.2049;
  if (((int)frontSensorDistanceTemp > 0) && ((int)frontSensorDistanceTemp < 1000)) {
    frontSensorDistance = min(frontSensorDistanceTemp, 60);
  }


  Robot.rect(0, 70, 100, 10);

  Robot.debugPrint(motorDifferential, 50, 70);

  Robot.debugPrint(frontSensorDistance, 5, 70);

  // Read anaog sensors
  leftSensorDistance = min(IRSensorLinearize(Robot.analogRead(leftSensorPin)), 60);
  Robot.rect(0, 90, 100, 10);
  Robot.debugPrint(leftSensorDistance, 5, 90);

  rightSensorDistance = min(IRSensorLinearize(Robot.analogRead(rightSensorPin)), 60);
  Robot.rect(0, 110, 100, 10);
  Robot.debugPrint(rightSensorDistance, 5, 110);

  backSensorDistance = min(IRSensorLinearize(Robot.analogRead(backSensorPin)), 60);
  Robot.rect(0, 130, 100, 10);
  Robot.debugPrint(backSensorDistance, 5, 130);

  frontLeftSensorDistance = min(IRSensorLinearize(Robot.analogRead(frontLeftSensorPin)), 60);
  //Robot.rect(0, 130, 100, 10);
  Robot.debugPrint(frontLeftSensorDistance, 50, 90);

  frontRightSensorDistance = min(IRSensorLinearize(Robot.analogRead(frontRightSensorPin)), 60);
  //Robot.rect(0, 130, 100, 10);
  Robot.debugPrint(frontRightSensorDistance, 50, 110);

  potentialFieldMagnitude = sqrt(pow(leftSensorDistance + frontLeftSensorDistance * cos(PI / 4) - rightSensorDistance - frontRightSensorDistance * cos(PI / 4), 2) + pow(frontSensorDistance + frontLeftSensorDistance * sin(PI / 4) - backSensorDistance - frontRightSensorDistance * sin(PI / 4), 2));
  potentialFieldAngle = atan2(leftSensorDistance + frontLeftSensorDistance * cos(PI / 4) - rightSensorDistance - frontRightSensorDistance * cos(PI / 4), frontSensorDistance + frontLeftSensorDistance * sin(PI / 4) - backSensorDistance - frontRightSensorDistance * sin(PI / 4)) * 360 / (2 * PI);

  Robot.rect(0, 140, 100, 10);
  Robot.debugPrint(potentialFieldMagnitude, 5, 140);
  Robot.debugPrint(potentialFieldAngle, 50, 140);

  Robot.rect(0, 0, 150, 10);

  if (state == STATE_RANDOM_WANDER) {
    Robot.text("STATE: RANDOM_WANDR", 5, 0);
    RandomWander();

  }

  else if (state == STATE_WANDER_AVOID) {
    Robot.text("STATE: WANDER AVOID", 5, 0);
    WanderAvoid();
  } else if (state == STATE_READY) {
    Robot.text("STATE: READY", 5, 0);
    leftMotorSpeed = 0;
    rightMotorSpeed = 0;
  } else if (state == STATE_FOLLOW_WALL) {
    Robot.text("STATE: FOLLOW WALL", 5, 0);
    //FollowWall();
  }

  Robot.motorsWrite(rightMotorSpeed, leftMotorSpeed);
  delay(250);
  Robot.motorsWrite(0,0);
}

//void DisplayMenu() {
/*
  Robot.stroke(0, 0, 0);
  Robot.background(255, 255, 255);
  Robot.text("<: shy kid", 5, 10);
  Robot.text("v: aggressive kid", 5, 20);
  Robot.text(">: random wander", 5, 30);
  Robot.text("^: wander and avoid", 5, 40);
  //Robot.text("center: return", 5, 40);
*/
//}

void RandomWander() {
  wanderCount = wanderCount % 10;

  //if (wanderCount == 0) {

  leftMotorSpeed = random(75, 200);
  rightMotorSpeed = random(75, 200);

  if (rightMotorSpeed < leftMotorSpeed) {
    leftMotorSpeed = 200;
  } else {
    rightMotorSpeed = 200;
  }
  //}


  //Robot.motorsWrite(defaultMotorSpeed, defaultMotorSpeed);
  //delay(350);
  //Robot.motorsWrite(0, 0);
  //wanderCount++;
}
void DriveInputAngle(int inputAngle) {
  if (inputAngle > 0) {
    //rightMotorSpeed = defaultMotorSpeed;
    //leftMotorSpeed = -defaultMotorSpeed;
    Robot.motorsWrite(defaultMotorSpeed, -defaultMotorSpeed);
    delay(abs(inputAngle) / spinDegreesPerMilisecond);
    Robot.motorsWrite(0, 0);
  } else {
    //rightMotorSpeed = -defaultMotorSpeed;
    //leftMotorSpeed = defaultMotorSpeed;
    Robot.motorsWrite(-defaultMotorSpeed, defaultMotorSpeed);
    delay(abs(inputAngle) / spinDegreesPerMilisecond);
    Robot.motorsWrite(0, 0);
  }
}

void WanderAvoid() {
  // If there is an obstacle too close, take evasive action.
  if (potentialFieldMagnitude > 30) {
    DriveInputAngle(potentialFieldAngle);
    delay(250);
  }
  // Pick motor speeds for random value.
  RandomWander();
}

void FollowWall() {
  //Robot.rect(0, 90, 100, 10);
  //Robot.debugPrint(leftSensorDistance, 5, 90);
  if (state == STATE_FOLLOW_WALL) {
    int distanceToWall = min(min(leftSensorDistance, frontLeftSensorDistance), frontSensorDistance);
    int currentError = distanceToWall - targetWallDistance;
    if (abs(currentError) < 3) {
      currentError = 0;
    }
    int currentTime = millis();
    timeStep = currentTime - lastTime;
    //Robot.debugPrint(timeStep, 5,15);

    motorDifferential = PDController(currentError, lastError, timeStep);
    //Serial.println(motorDifferential);
    //int leftMotorValue;
    //int rightMotorValue;

    if (motorDifferential > 0) {
      rightMotorSpeed = defaultMotorSpeed;
      leftMotorSpeed = max(defaultMotorSpeed / (1 + abs(motorDifferential)), defaultMotorSpeed * 0.5);

    } else {
      leftMotorSpeed = defaultMotorSpeed;
      rightMotorSpeed = max(defaultMotorSpeed / (1 + abs(motorDifferential)), defaultMotorSpeed * 0.5);
    }

    lastTime = currentTime;
    lastError = currentError;
    Serial.print("Error: ");
    Serial.println(currentError);
    Serial.println(motorDifferential);
  }
}

bool FrontObstacleClose() {
  if (frontSensorDistance < obstacleDistanceThreshold) {
    return true;
  }
  return false;
}

float IRSensorLinearize(int input) {
  return 14235 * pow(input, -1.167);
}

float PDController(int error, int lastError, int timeStep) {
  //return kP * error + kD * (error - lastError) / timeStep;
  return kP * error;
}

void UpdateMotorSpeeds() {
  Robot.motorsWrite(rightMotorSpeed, leftMotorSpeed);
}


