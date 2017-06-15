#include "Driver.h"
#include <math.h>

#define motorSteps 200
#define microStep 4

#define stepPin1  2
#define dirPin1   3
#define stepPin2  4
#define dirPin2   5
#define stepPin3  6
#define dirPin3   7
#define stepPin4  8
#define dirPin4   9
#define stepPin5  10
#define dirPin5   11
#define stepPin6  12
#define dirPin6   13
#define stepPin7  14
#define dirPin7   15
#define stepPin8  16
#define dirPin8   17
#define stepPin9  18
#define dirPin9   19
#define stepPin10 20
#define dirPin10  21

double target1, target2, target3, target4, target5, target6, target7, target8, target9, target10, target;

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

int pin = 1;
unsigned int lastChange = millis();

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
  //Serial.begin(230400);
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
}

void loop() {
  //get target
  unsigned int curr = millis();
  /*int curr2 = curr%8000;
  curr = curr%4000;
  double target2 = curr*10.0/4000.0;
  double target1 = 10*sin(curr2/8000.0*2*PI);*/
  unsigned int curr2 = curr%1000;
  if (curr2<500) target = curr2*7.5/500.0;
  else  target = curr2*-7.5/500.0 + 15;
  /*target2 = target1;
  target3 = target1;
  target4 = target1;
  target5 = target1;
  target6 = target1;
  target7 = target1;
  target8 = target1;
  target9 = target1;
  target10 = target1;*/

  pin = (curr/1000);
  pin = pin%7;
  pin = pin+1;

  target1 = 0;
  target2 = 0;
  target3 = 0;
  target4 = 0;
  target5 = 0;
  target6 = 0;
  target7 = 0;

  switch (pin){
    case 1: target1 = target; break;
    case 2: target2 = target; break;
    case 3: target3 = target; break;
    case 4: target4 = target; break;
    case 5: target5 = target; break;
    case 6: target6 = target; break;
    case 7: target7 = target; break;  
  }

  int target_1 = target1/10.0*motorSteps*microStep;
  int target_2 = target2/10.0*motorSteps*microStep;
  int target_3 = target3/10.0*motorSteps*microStep;
  int target_4 = target4/10.0*motorSteps*microStep;
  int target_5 = target5/10.0*motorSteps*microStep;
  int target_6 = target6/10.0*motorSteps*microStep;
  int target_7 = target7/10.0*motorSteps*microStep;
  int target_8 = target8/10.0*motorSteps*microStep;
  int target_9 = target9/10.0*motorSteps*microStep;
  int target_10 = target10/10.0*motorSteps*microStep;
  /*Serial.print("target1 = ");
  Serial.print(target_1);
  Serial.print("\tcurr1 = ");
  Serial.print(Stepper1.currStep());
  Serial.print("\ttarget2 = ");
  Serial.print(target_2);
  Serial.print("\tcurr2 = ");
  Serial.println(Stepper2.currStep());*/
  
  Stepper1.step(target_1);
  Stepper2.step(target_2);
  Stepper3.step(target_3);
  Stepper4.step(target_4);
  Stepper5.step(target_5);
  Stepper6.step(target_6);
  Stepper7.step(target_7);
  Stepper8.step(target_8);
  Stepper9.step(target_9);
  Stepper10.step(target_10);
  
}

