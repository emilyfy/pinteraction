/*
 * rosserial subsriber ft_sub
 * subscribe to topic /fourier
 * actuate 10 pins to height
 */

#include <ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <math.h>
#include "Driver.h"

#define motorSteps 200
#define microStep 4

#define stepPin1   2
#define dirPin1    3
#define stepPin2   4
#define dirPin2    5
#define stepPin3   6
#define dirPin3    7
#define stepPin4   8
#define dirPin4    9
#define stepPin5   10
#define dirPin5    11
#define stepPin6   12
#define dirPin6    13
#define stepPin7   14
#define dirPin7    15
#define stepPin8   14
#define dirPin8    15
#define stepPin9   14
#define dirPin9    15
#define stepPin10  14
#define dirPin10   15

double target1, target2, target3, target4, target5, target6, target7, target8, target9, target10;
int target_1, target_2, target_3, target_4, target_5, target_6, target_7, target_8, target_9, target_10;

Driver Stepper1(motorSteps*microStep, dirPin1, stepPin1);
Driver Stepper2(motorSteps*microStep, dirPin2, stepPin2);
Driver Stepper3(motorSteps*microStep, dirPin3, stepPin3);
Driver Stepper4(motorSteps*microStep, dirPin4, stepPin4);
Driver Stepper5(motorSteps*microStep, dirPin5, stepPin5);
Driver Stepper6(motorSteps*microStep, dirPin6, stepPin6);
Driver Stepper7(motorSteps*microStep, dirPin7, stepPin7);
Driver Stepper8(motorSteps*microStep, dirPin8, stepPin8);
Driver Stepper9(motorSteps*microStep, dirPin9, stepPin9);
Driver Stepper10(motorSteps*microStep, dirPin10, stepPin10);

ros::NodeHandle nh;

void messageCb(const std_msgs::Float64MultiArray& height) {
  target1 = height.data[0];
  target2 = height.data[1];
  target3 = height.data[2];
  target4 = height.data[3];
  target5 = height.data[4];
  target6 = height.data[5];
  target7 = height.data[6];
  target8 = height.data[7];
  target9 = height.data[8];
  target10 = height.data[9];

  target_1 = target1/10.0*motorSteps*microStep;
  target_2 = target2/10.0*motorSteps*microStep;
  target_3 = target3/10.0*motorSteps*microStep;
  target_4 = target4/10.0*motorSteps*microStep;
  target_5 = target5/10.0*motorSteps*microStep;
  target_6 = target6/10.0*motorSteps*microStep;
  target_7 = target7/10.0*motorSteps*microStep;
  target_8 = target8/10.0*motorSteps*microStep;
  target_9 = target9/10.0*motorSteps*microStep;
  target_10 = target10/10.0*motorSteps*microStep;
  
}

ros::Subscriber<std_msgs::Float64MultiArray> sub("/fourier", &messageCb);

void setup() {
  Stepper1.setSpeed(250);
  Stepper2.setSpeed(250);
  Stepper3.setSpeed(250);
  Stepper4.setSpeed(250);
  Stepper5.setSpeed(250);
  Stepper6.setSpeed(250);
  Stepper7.setSpeed(250);
  Stepper8.setSpeed(250);
  Stepper9.setSpeed(250);
  Stepper10.setSpeed(250);
  
  digitalWrite(stepPin1,LOW);
  digitalWrite(stepPin2,LOW);
  digitalWrite(stepPin3,LOW);
  digitalWrite(stepPin4,LOW);
  digitalWrite(stepPin5,LOW);
  digitalWrite(stepPin6,LOW);
  digitalWrite(stepPin7,LOW);
  digitalWrite(stepPin8,LOW);
  digitalWrite(stepPin9,LOW);
  digitalWrite(stepPin10,LOW);
  
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  Stepper1.step(target_1);
  Stepper2.step(target_2);
  Stepper3.step(target_3);
  Stepper4.step(target_4);
  Stepper5.step(target_5);
  Stepper6.step(target_6);
  Stepper7.step(target_7);
  Stepper7.step(target_8);
  Stepper7.step(target_9);
  Stepper7.step(target_10);

  nh.spinOnce();  
}
