#include "MPU6050_6Axis_MotionApps20.h"

// Motor driver parameters
#define RIGHT_enA 6
#define RIGHT_in1 7
#define RIGHT_in2 8

#define LEFT_enB 3
#define LEFT_in1 2
#define LEFT_in2 4

extern VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
extern float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
extern uint16_t r, g, b, c, colorTemp, lux;

// Motor speed to use
const float DEFAULT_SPEED = 1.0;



void setup() {
  Serial.begin(115200);
  //mpu_setup();
  //color_setup();
  
  // Right motor
  pinMode(RIGHT_enA, OUTPUT);
  pinMode(RIGHT_in1, OUTPUT);
  pinMode(RIGHT_in2, OUTPUT);
  stopRightWheel();

  // Left motor
  pinMode(LEFT_enB, OUTPUT);
  pinMode(LEFT_in1, OUTPUT);
  pinMode(LEFT_in2, OUTPUT);
  stopLeftWheel();

  // set enable pins to true
  digitalWrite(RIGHT_enA, HIGH);
  digitalWrite(LEFT_enB, HIGH);

}

void loop() {
  //mpu_run();
  //color_run();

  driveStraight(1000, DEFAULT_SPEED, 1);
  delay(1000);
    driveStraight(1000, DEFAULT_SPEED, 0);
    delay(1000);
    

  
}

void driveStraightFeedback(int runDuration, float motorSpeed, bool forward)
{
  runRightWheel(motorSpeed, forward);
  runLeftWheel(motorSpeed, forward);

  delay(runDuration);

  stopDrive();
}

//duration in ms, speed from 0-1 (run 1 if not on ext power, forward is bool where 1 is forward
void driveStraight(int runDuration, float motorSpeed, bool forward)
{
  runRightWheel(motorSpeed, forward);
  runLeftWheel(motorSpeed, forward);

  delay(runDuration);

  stopDrive();
}

// Turn the robot for a set time
void turn(int turnDuration, float motorSpeed, bool clockwise)
{

  if(clockwise)
  {
    runRightWheel(motorSpeed, false);
    runLeftWheel(motorSpeed, true);
  }
  else
  {
    runRightWheel(motorSpeed, true);
    runLeftWheel(motorSpeed, false);
  }

  delay(turnDuration);
  stopDrive();
}

//stop drive motors
void stopDrive()
{
  stopRightWheel();
  stopLeftWheel();
}

//motor speed is a number from 0 to 1
void runRightWheel(float motorSpeed, bool forward)
{
  //set motor direction
  if(forward)
  {
    digitalWrite(RIGHT_in1, HIGH);
    digitalWrite(RIGHT_in2, LOW);
  }
  else
  {
    digitalWrite(RIGHT_in1, LOW);
    digitalWrite(RIGHT_in2, HIGH);
  }
  analogWrite(RIGHT_enA, speedToPWM(motorSpeed));
}

void stopRightWheel()
{
  digitalWrite(RIGHT_in1, LOW);
  digitalWrite(RIGHT_in2, LOW);
}

//give a speed from 0 to 1

int speedToPWM(float motorSpeed)
{
  // For PWM maximum possible values are 0 to 255
  float converted = motorSpeed * 255;
  return (int)converted;
}

//LEFTWHEEL FUNCTIONS
//motor speed is a number from 0 to 1
void runLeftWheel(float motorSpeed, bool forward)
{
  //set motor direction
  if(forward)
  {
    digitalWrite(LEFT_in1, HIGH);
    digitalWrite(LEFT_in2, LOW);
  }
  else
  {
    digitalWrite(LEFT_in1, LOW);
    digitalWrite(LEFT_in2, HIGH);
  }
  
  //set PWM signal for speed
  analogWrite(LEFT_enB, speedToPWM(motorSpeed));
}

void stopLeftWheel()
{
  digitalWrite(LEFT_in1, LOW);
  digitalWrite(LEFT_in2, LOW);
}