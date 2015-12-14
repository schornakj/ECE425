#include <ArduinoRobot.h>
#include <math.h>
#include <Wire.h>

// Define the IR codes corresponding to our remote

#define IR_CODE_RETURN 16609423
#define IR_CODE_ONE 16582903
#define IR_CODE_TWO 16615543
#define IR_CODE_THREE 16599223
#define IR_CODE_FOUR 16591063


//define states
#define STATE_READY 0
#define STATE_SHY_KID 1
#define STATE_SHY_KID_OBSTACLE 2
#define STATE_AGGRESSIVE_KID_IDLE 3
#define STATE_AGGRESSIVE_KID_CHARGE 4
#define STATE_RANDOM_WANDER 5
#define STATE_WANDER_AVOID 6
#define STATE_WANDER_AVOID_OBSTACLE 7

#define FORWARD 0
#define RIGHT 1
#define BACK 2
#define LEFT 3

float frontSensorValue;
float leftSensorValue;
float rightSensorValue;
float backSensorValue;
int frontSensorPin = TKD1;
int leftSensorPin = TK4;
int rightSensorPin = TK0;
int backSensorPin = TK6;
int ftSonarPin = TKD1;
int wanderCount = 0;

int state = STATE_WANDER_AVOID;


int defaultMotorSpeed = 200;
int leftMotorSpeed = 0;
int rightMotorSpeed = 0;

float spinDegreesPerMilisecond = (168 / 90) * 0.325;

int sensorCount = 0;
float frontSensorDistance = -1;
int leftSensorDistance = -1;
int rightSensorDistance = -1;
int backSensorDistance = -1;
int obstacleDistanceThreshold = 20;
int obstacleDistanceFarThreshold = 50;

int potentialFieldAngle = 0;
int potentialFieldMagnitude = 0;

void setup() {
  // put your setup code here, to run once:
  Robot.begin();
  Robot.beginTFT();
  Robot.beginSD();
  Serial.begin(9600);

  //irrecv.enableIRIn(); // Start the receiver

}

void loop() {
  // put your main code here, to run repeatedly:
  Robot.stroke(0, 0, 0);
  Robot.text("left sensor reading", 3, 80);
  Robot.text("right sensor reading", 3, 100);
  Robot.text("back sensor reading", 3, 120);
  Robot.text("front sensor reading", 5, 60);

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
    frontSensorDistance = frontSensorDistanceTemp;
  }

  Robot.rect(0, 70, 100, 10);

  Robot.debugPrint(frontSensorDistance, 5, 70);

  leftSensorDistance = IRSensorLinearize(Robot.analogRead(leftSensorPin));
  Robot.rect(0, 90, 100, 10);
  Robot.debugPrint(leftSensorDistance, 5, 90);

  rightSensorDistance = IRSensorLinearize(Robot.analogRead(rightSensorPin));
  Robot.rect(0, 110, 100, 10);
  Robot.debugPrint(rightSensorDistance, 5, 110);

  backSensorDistance = IRSensorLinearize(Robot.analogRead(backSensorPin));
  Robot.rect(0, 130, 100, 10);
  Robot.debugPrint(backSensorDistance, 5, 130);

  potentialFieldMagnitude = sqrt(pow(leftSensorDistance - rightSensorDistance, 2) + pow(frontSensorDistance - backSensorDistance, 2));
  potentialFieldAngle = atan2(leftSensorDistance - rightSensorDistance, frontSensorDistance - backSensorDistance) * 360 / (2 * PI);

  Robot.rect(0, 140, 100, 10);
  Robot.debugPrint(potentialFieldMagnitude, 5, 140);
  Robot.debugPrint(potentialFieldAngle, 50, 140);
  Serial.print("Distance: ");
  Serial.println(potentialFieldMagnitude);
  Serial.print("Angle: ");
  Serial.println(potentialFieldAngle);
  
  if (Robot.keyboardRead() == BUTTON_MIDDLE) {

  }


  if (state == STATE_SHY_KID) {
    ShyKid();
  }
  else if (state == STATE_SHY_KID_OBSTACLE) {
    ShyKidAvoid();
  }
  else if (state == STATE_AGGRESSIVE_KID_IDLE) {
    AggressiveKidIdle();
  }
  else if (state == STATE_AGGRESSIVE_KID_CHARGE) {
    AggressiveKidCharge();
  } else if (state == STATE_WANDER_AVOID) {
    WanderAvoid();
  }

}
/*
  void processResult() {
  unsigned long res = results.value;
  // print the value to the screen
  //Robot.debugPrint(res, 5, 15);
  Serial.println(res, HEX);


  switch (results.value) {

    // shy kid
    case (IR_CODE_ONE):
      if (state == STATE_READY) {
        leftMotorSpeed = 150;
        rightMotorSpeed = 150;
        state = STATE_SHY_KID;
        Robot.stroke(0, 0, 0);
        Robot.background(255, 255, 255);
        Robot.text("shy kid", 5, 40);
        ShyKid();
      }
      break;

    //aggressive kid
    case (IR_CODE_TWO):
      if (state == STATE_READY) {
        leftMotorSpeed = 150;
        rightMotorSpeed = 150;
        state = STATE_AGGRESSIVE_KID_IDLE;
        Robot.stroke(0, 0, 0);
        Robot.background(255, 255, 255);
        Robot.text("aggressive kid", 5, 40);
        AggressiveKidIdle();
      }
      break;

    // wander
    case (IR_CODE_THREE):
      if (state == STATE_READY) {
        leftMotorSpeed = 150;
        rightMotorSpeed = 150;
        //Robot.motorsWrite(rightMotorSpeed, leftMotorSpeed);
        state = STATE_RANDOM_WANDER;
        Robot.stroke(0, 0, 0);
        Robot.background(255, 255, 255);
        Robot.text("random wander", 5, 40);
        RandomWander();
      }
      break;

    // wander and avoid
    case (IR_CODE_FOUR):
      if (state == STATE_READY) {
        leftMotorSpeed = 150;
        rightMotorSpeed = 150;
        //Robot.motorsWrite(rightMotorSpeed, leftMotorSpeed);
        state = STATE_WANDER_AVOID;
        Robot.stroke(0, 0, 0);
        Robot.background(255, 255, 255);
        Robot.text("wander and avoid", 5, 40);
        WanderAvoid();
      }
      break;

    case (IR_CODE_RETURN):
      leftMotorSpeed = 0;
      rightMotorSpeed = 0;
      state = STATE_READY;
      DisplayMenu();
      break;
  }
  }
*/

void DisplayMenu() {
  Robot.stroke(0, 0, 0);
  Robot.background(255, 255, 255);
  Robot.text("1: shy kid", 5, 10);
  Robot.text("2: aggressive kid", 5, 20);
  Robot.text("3: random wander", 5, 30);
  Robot.text("4: wander and avoid", 5, 40);
}

void ShyKid() {
  if (FrontObstacleClose()) {
    state = STATE_SHY_KID_OBSTACLE;
  } else {
    //Robot.motorsWrite(0, 0);

    BrakeMotors();

  }

}

void ShyKidAvoid() {
  if (!FrontObstacleClose()) {
    state = STATE_SHY_KID;
  } else {
    //Robot.motorsWrite(-defaultMotorSpeed, -defaultMotorSpeed);
    leftMotorSpeed = -defaultMotorSpeed;
    rightMotorSpeed = -defaultMotorSpeed;
  }
}

void AggressiveKidIdle() {
  if ((frontSensorDistance < obstacleDistanceFarThreshold) || (frontSensorDistance > obstacleDistanceThreshold)) {
    state = STATE_AGGRESSIVE_KID_CHARGE;
  }
  BrakeMotors();
}

void AggressiveKidCharge() {
  if ((frontSensorDistance > obstacleDistanceFarThreshold) || (frontSensorDistance < obstacleDistanceThreshold)) {
    state = STATE_AGGRESSIVE_KID_IDLE;
  }
  else {
    leftMotorSpeed = defaultMotorSpeed;
    rightMotorSpeed = defaultMotorSpeed;
  }
}

void RandomWander() {
  wanderCount++;
  if (wanderCount > 10) {
    wanderCount = 0;
    leftMotorSpeed = random(75, 225);
    rightMotorSpeed = random(75, 225);
    //Robot.motorsWrite(rightMotorSpeed, leftMotorSpeed);
  }
}
void DriveInputAngle(int inputAngle) {
  if (inputAngle > 0) {
    rightMotorSpeed = defaultMotorSpeed;
    leftMotorSpeed = -defaultMotorSpeed;
    delay(abs(inputAngle) / spinDegreesPerMilisecond);
    //Robot.motorsWrite(0, 0);
  } else {
    rightMotorSpeed = -defaultMotorSpeed;
    leftMotorSpeed = defaultMotorSpeed;
    delay(abs(inputAngle) / spinDegreesPerMilisecond);
    //Robot.motorsWrite(0, 0);
  }
}

void WanderAvoid() {
  // If there is an obstacle too close, take evasive action.
  if (potentialFieldMagnitude < 75) {
    if (potentialFieldAngle > 0) {

      Robot.motorsWrite(defaultMotorSpeed, -defaultMotorSpeed);
      delay(abs(potentialFieldAngle) / spinDegreesPerMilisecond);
      Robot.motorsWrite(0, 0);
    } else {
      Robot.motorsWrite(-defaultMotorSpeed, defaultMotorSpeed);
      delay(abs(potentialFieldAngle) / spinDegreesPerMilisecond);
      //Robot.motorsWrite(0, 0);
    }

    delay(250);
  }

  // Pick motor speeds for random value.
  int leftValue = random(75, 200);
  int rightValue = random(75, 200);

  // Maximize the overall robot speed.
  if (rightValue < leftValue){
    leftValue = 200;
  } else {
    rightValue = 200;
  }
  Robot.motorsWrite(rightValue, leftValue);
  // Wait a while
  delay(250);
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



void BrakeMotors() {
  if (leftMotorSpeed != 0 || rightMotorSpeed != 0) {
    Robot.motorsWrite(-rightMotorSpeed, -leftMotorSpeed);
    delay(10);
    leftMotorSpeed = 0;
    rightMotorSpeed = 0;
  }
}

