#include "MPU6050_6Axis_MotionApps20.h"

extern VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
extern float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
extern uint16_t r, g, b, c, colorTemp, lux;

// Motor speed to use
const float DEFAULT_SPEED = 0.5;







void setup() {
  Serial.begin(115200);
  mpu_setup();
  //color_setup();
  
  motor_setup();
  //driveStraight(2000, DEFAULT_SPEED, 1);

}
extern unsigned long xvalue, yvalue, y2value;

void loop() {
  mpu_run();
  //color_run();
  
  //Serial.println(ypr[0]);
  remote_run();
  teleop(xvalue, yvalue);
  runLift();
  /*driveStraight(1000, DEFAULT_SPEED, 1);
  delay(1000);
  driveStraight(1000, 0.3, 0);
  delay(1000);*/
  /*driveStraightFeedback(1200, DEFAULT_SPEED, 1);
  stopDrive();
  delay(2000);
  driveStraightFeedback(600, DEFAULT_SPEED, 0);
  stopDrive();
  delay(2000);*/
  /*turnDegrees(90, 0.4);
  delay(2000);
  turnDegrees(-90, 0.4);
  delay(2000);*/
}
