// Motor driver parameters
#define RIGHT_enA 6
#define RIGHT_in1 7
#define RIGHT_in2 8

#define LEFT_enB 10
#define LEFT_in1 9
#define LEFT_in2 4

#define LIFT_EN 11
#define LIFT_up 12
#define LIFT_down 13

float stall_percentage = 0.23;

float p_gain = 1;

extern float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

void motor_setup() {
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

  
  // lift motor
  pinMode(LIFT_EN, OUTPUT);
  pinMode(LIFT_up, OUTPUT);
  pinMode(LIFT_down, OUTPUT);

  // set enable pins to true
  digitalWrite(RIGHT_enA, HIGH);
  digitalWrite(LEFT_enB, HIGH);

  
}

extern bool up, down;
float liftUpSpeed = 1;
float liftDownSpeed = 0.5;
void runLift() {
  if (up) {
    digitalWrite(LIFT_up, HIGH);
    digitalWrite(LIFT_down, LOW);
    analogWrite(LIFT_EN, speedToPWM(liftUpSpeed));
  } else if (down) {
    digitalWrite(LIFT_up, LOW);
    digitalWrite(LIFT_down, HIGH);
    analogWrite(LIFT_EN, speedToPWM(liftDownSpeed));
  } else {
     digitalWrite(LIFT_up, HIGH);
    digitalWrite(LIFT_down, LOW);
    analogWrite(LIFT_EN, speedToPWM(0.15));
  }

  Serial.println(down);
}

void driveStraightFeedback(int runDuration, float motorSpeed, bool forward)
{
  unsigned long starttime = millis();
  unsigned long loopduration = micros();
  float currentYaw = ypr[0];
  while(millis() < starttime + runDuration) {
    mpu_run();
    float yawOffset = ypr[0] - currentYaw;
    if(!forward) {
      yawOffset*=-1;
    }
    runLeftWheel(motorSpeed + yawOffset*p_gain, forward);
    runRightWheel(motorSpeed - yawOffset*p_gain, forward);
    delay(10);
  }

  stopDrive();
}

float targetHeading = 0;
bool wasStopped = true;
float teleopTurnRate = 0.02;
float teleopTurnGain = 1.0;
extern unsigned long y2value;
void teleop(float x, float y) {
  teleopTurnRate = map(y2value, 0, 2000, 20, 60)/1000.0f;
  Serial.println(teleopTurnRate);
  
  if(abs(x - 1000) < 100 && abs(y - 1000) < 100) {
    stopDrive();
    wasStopped = true;
    return;
  }
  if (wasStopped) {
    wasStopped = false;
    targetHeading = ypr[0];
  }
  targetHeading += map(float(x), 0, 2000, -100, 100)/100.0f*teleopTurnRate;
  float yawOffset = ypr[0] - targetHeading;
  float s = map(y, 0.0, 2000.0, -60, 60)/100.0f;
  bool forward = s < 0;
  float leftSpeed = max(min(s + yawOffset*teleopTurnGain, 1), -1);
  float rightSpeed = max(min(s - yawOffset*teleopTurnGain, 1), -1);
  if(abs(leftSpeed) < 0.2) {
    leftSpeed = 0;
  }
  if(abs(rightSpeed) < 0.2) {
    rightSpeed = 0;
  }
  runLeftWheel(abs(leftSpeed), leftSpeed > 0);
  runRightWheel(abs(rightSpeed)+0.1, rightSpeed > 0);
  delay(10);
}



float turnGain = 0.5;
void turnDegrees(float deg, float motorSpeed) {
  float currentYaw = ypr[0];
  float yawOffset = currentYaw;
  deg = deg / 180.0 * PI;
  if(deg > 0) {
    runLeftWheel(motorSpeed, 0);
    runRightWheel(motorSpeed, 1);
    while(yawOffset < deg) {
      mpu_run();
      yawOffset = ypr[0] - currentYaw;
      motorSpeed = max(min(motorSpeed, abs(deg - yawOffset)*turnGain), stall_percentage);
      runLeftWheel(motorSpeed, 0);
      runRightWheel(motorSpeed, 1);
      Serial.println(yawOffset);
      delay(10);
    }
  } else {
    runLeftWheel(motorSpeed, 1);
    runRightWheel(motorSpeed, 0);
    while(yawOffset > deg) {
      mpu_run();
      yawOffset = ypr[0] - currentYaw;
      motorSpeed = max(min(motorSpeed, abs(deg - yawOffset)*turnGain), stall_percentage);
      runLeftWheel(motorSpeed, 1);
      runRightWheel(motorSpeed, 0);
      Serial.println(yawOffset);
      delay(10);
    }
  }
  stopDrive();
}

void turnLeftDegrees(float deg, float motorSpeed) {
  
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
