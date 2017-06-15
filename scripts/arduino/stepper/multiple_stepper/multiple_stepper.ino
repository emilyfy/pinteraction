#include "Driver.h"

#define motorSteps 200
#define motorNum 4

int dirPin[motorNum], stepPin[motorNum];
//connect dirPin[0]  to  2
//connect stepPin[0] to  3
//connect dirPin[1]  to  4
//connect stepPin[1] to  5
//and so on

Driver * stepper[motorNum];
//  Driver(motorSteps, dirPin[i], stepPin[i])
//}

void setup() {
  //Driver stepper[motorNum] = {
    //for (int i=0; i<motorNum; i++)
    //  Driver(motorSteps, dirPin[i], stepPin[i]);
  //}
  
  for (int i=0; i<motorNum; i++)
  {
    
    dirPin[i] = 2*i+2;
    stepPin[i] = 2*i+3;
    
    stepper[i] = new Driver(motorSteps, dirPin[i], stepPin[i]);
    
    stepper[i].setSpeed(500);
    digitalWrite(stepPin[i], LOW);
  }
}

void loop() {
  //get target
  
  for (int i=0; i<motorNum; i++)
    stepper[i].step(target[i]);
}

