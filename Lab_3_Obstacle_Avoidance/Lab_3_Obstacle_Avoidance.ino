// Include libraries
#include <IRremote.h>
#include <ArduinoRobot.h>
#include <math.h>
#include <Wire.h>

// Define the IR codes corresponding to our remote
#define IR_CODE_VOLUME_MINUS 16580863
#define IR_CODE_PLAY_PAUSE 16613503
#define IR_CODE_VOLUME_PLUS 16597183
#define IR_CODE_SETUP 16589023
#define IR_CODE_UP 16621663
#define IR_CODE_STOP_MODE 16605343
#define IR_CODE_LEFT 16584943
#define IR_CODE_ENTER_SAVE 16617583
#define IR_CODE_RIGHT 16601263
#define IR_CODE_TEN_PLUS 16593103
#define IR_CODE_DOWN 16625743
#define IR_CODE_RETURN 16609423
#define IR_CODE_ONE 16582903
#define IR_CODE_TWO 16615543
#define IR_CODE_THREE 16599223
#define IR_CODE_FOUR 16591063
#define IR_CODE_FIVE 16623703
#define IR_CODE_SIX 16607383
#define IR_CODE_SEVEN 16586983
#define IR_CODE_EIGHT 16619623
#define IR_CODE_NINE 16603303

//define states
#define STATE_READY 0
#define STATE_SHY_KID 1
#define STATE_SHY_KID_OBSTACLE 2
#define STATE_AGGRESSIVE_KID 3
#define STATE_AGGRESSIVE_KID_OBSTACLE 4
#define STATE_RANDOM_WANDER 5
#define STATE_WANDER_AVOID 6
#define STATE_WANDER_AVOID_OBSTACLE 7

int RECV_PIN = TKD1; // the pin the IR receiver is connected to
IRrecv irrecv(RECV_PIN); // an instance of the IR receiver object
decode_results results; // container for received IR codes

long frontSensorValue;
long leftSensorValue;
long rightSensorValue;
long backSensorValue;
int frontSensorPin = TKD1;
int leftSensorPin = TK4;
int rightSensorPin = TK0;
int backSensorPin = TK6;
int ftSonarPin = TKD1;
int wanderCount = 0;
int state = STATE_READY;
int leftMotorSpeed = 0;
int rightMotorSpeed = 0;
int sensorCount = 0;
int frontSensorDistance;
int leftSensorDistance;
int rightSensorDistance;
int backSensorDistance;
int obstacleDistance;


void setup() {
  // put your setup code here, to run once:
  Robot.begin();
  Robot.beginTFT();
  Robot.beginSD();
  Serial.begin(9600);

  irrecv.enableIRIn(); // Start the receiver

  Robot.stroke(0, 0, 0);
  Robot.fill(255, 255, 255);
  Robot.text("1: shy kid", 5, 10);
  Robot.text("2: aggressive kid", 5, 20);
  Robot.text("3: random wander", 5, 30);
  Robot.text("4: wander and avoid", 5, 40);
}

void loop() {
  // put your main code here, to run repeatedly:
  //  // read analog sensor
  sensorCount = sensorCount + 1;
  if (sensorCount == 1) {
    leftSensorValue = Robot.analogRead(leftSensorPin);
    leftSensorValue = (Robot.analogRead(leftSensorPin) + leftSensorValue)/2;
    leftSensorValue = (Robot.analogRead(leftSensorPin) + leftSensorValue)/3;
    leftSensorDistance = 14235*pow(leftSensorValue,-1.167);
    Robot.text("left sensor reading", 3, 80);
    Robot.debugPrint(leftSensorDistance, 5, 90);
  }
  else if (sensorCount == 2) {
    rightSensorValue = Robot.analogRead(rightSensorPin);
    rightSensorValue = (Robot.analogRead(rightSensorPin) + rightSensorValue)/2;
    rightSensorValue = (Robot.analogRead(rightSensorPin) + rightSensorValue)/3;
    rightSensorDistance = 14235*pow(rightSensorValue,-1.167);
    Robot.text("right sensor reading", 3, 100);
    Robot.debugPrint(rightSensorDistance, 5, 110);
  }
  else if (sensorCount == 3) {
    backSensorValue = Robot.analogRead(backSensorPin);
    backSensorValue = (Robot.analogRead(backSensorPin) + backSensorValue)/2;
    backSensorValue = (Robot.analogRead(backSensorPin) + backSensorValue)/3;
    backSensorDistance = 14235*pow(backSensorValue,-1.167);
    Robot.text("back sensor reading", 3, 120);
    Robot.debugPrint(backSensorDistance, 5, 130);
  }
  else if (sensorCount == 4) {
    ////read digital sensor
    pinMode(ftSonarPin, OUTPUT);//set the PING pin as an output
    Robot.digitalWrite(ftSonarPin, LOW); //set the PING pin low first
    delayMicroseconds(2);//wait 2 us
    Robot.digitalWrite(ftSonarPin, HIGH);//trigger sonar by a 2 us HIGH PULSE
    delayMicroseconds(5);//wait 5 us
    Robot.digitalWrite(ftSonarPin, LOW);//set pin low first again
    pinMode(ftSonarPin, INPUT);//set pin as input with duration as reception time
    frontSensorValue = pulseIn(ftSonarPin, HIGH); //measures how long the pin is high
    frontSensorDistance = 0.0236*frontSensorValue + 0.2049;
    Robot.text("front sensor reading", 5, 60);
    Robot.debugPrint(frontSensorDistance, 5, 70);
    sensorCount = 0;
  }



  if (irrecv.decode(&results)) {
    processResult(); // if there is an IR command, process it
    irrecv.resume(); // resume receiver
  }

  // close obstacle
  if (frontSensorValue < obstacleDistance) {
    if (state == STATE_SHY_KID) {
      state = STATE_SHY_KID_OBSTACLE;
    }
    else if (state == STATE_AGGRESSIVE_KID) {
      state = STATE_AGGRESSIVE_KID_OBSTACLE;
    }
    else if (state == STATE_WANDER_AVOID) {
      state = STATE_WANDER_AVOID_OBSTACLE;
    }
  }
  else {
    if (state == STATE_SHY_KID) {
      ShyKid();
    }
    else if (state == STATE_SHY_KID_OBSTACLE) {

    }
    else if (state == STATE_AGGRESSIVE_KID) {
      AggressiveKid();
    }
    else if (state == STATE_AGGRESSIVE_KID_OBSTACLE) {

    }
    else if (state == STATE_RANDOM_WANDER) {
      RandomWander();
    }
    else if (state == STATE_WANDER_AVOID) {
      WanderAvoid();
    }
    else if (state == STATE_WANDER_AVOID_OBSTACLE) {
    }
  }
}
void processResult() {
  unsigned long res = results.value;
  // print the value to the screen
  //Robot.debugPrint(res, 5, 15);
  Serial.println(res, HEX);
  if (res == IR_CODE_VOLUME_MINUS || res == IR_CODE_PLAY_PAUSE  || res == IR_CODE_VOLUME_PLUS  || res == IR_CODE_SETUP  || res == IR_CODE_UP  || res == IR_CODE_STOP_MODE  || res == IR_CODE_LEFT  || res == IR_CODE_ENTER_SAVE  || res == IR_CODE_DOWN || res == IR_CODE_RETURN  || res == IR_CODE_ONE  || res == IR_CODE_TWO  || res == IR_CODE_THREE || res == IR_CODE_FOUR  || res == IR_CODE_FIVE  || res == IR_CODE_SIX  || res == IR_CODE_SEVEN || res == IR_CODE_EIGHT  || res == IR_CODE_NINE   )
  {
  }

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
        state = STATE_AGGRESSIVE_KID;
        Robot.stroke(0, 0, 0);
        Robot.background(255, 255, 255);
        Robot.text("aggressive kid", 5, 40);
        AggressiveKid();
      }
      break;

    // wander
    case (IR_CODE_THREE):
      if (state == STATE_READY) {
        leftMotorSpeed = 150;
        rightMotorSpeed = 150;
        Robot.motorsWrite(rightMotorSpeed, leftMotorSpeed);
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
        Robot.motorsWrite(rightMotorSpeed, leftMotorSpeed);
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


void DisplayMenu() {
  Robot.stroke(0, 0, 0);
  Robot.background(255, 255, 255);
  Robot.text("1: shy kid", 5, 10);
  Robot.text("2: aggressive kid", 5, 20);
  Robot.text("3: random wander", 5, 30);
  Robot.text("4: wander and avoid", 5, 40);
}

void ShyKid() {
if (state == STATE_SHY_KID){
  if(frontSensorValue < obstacleDistance)
}
}

void AggressiveKid() {
}

void RandomWander() {
  wanderCount++;
  if (wanderCount > 10) {
    wanderCount = 0;
    leftMotorSpeed = random(75, 225);
    rightMotorSpeed = random(75, 225);
    Robot.motorsWrite(rightMotorSpeed, leftMotorSpeed);
  }
}

void WanderAvoid() {
  wanderCount++;
  if (wanderCount > 10) {
    wanderCount = 0;
    leftMotorSpeed = random(75, 225);
    rightMotorSpeed = random(75, 225);
    Robot.motorsWrite(rightMotorSpeed, leftMotorSpeed);
  }
}

