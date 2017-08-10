#include "Driver.h"

#define motorSteps 200

#define dirPin   13
#define stepPin  12

Driver myStepper(motorSteps, dirPin, stepPin);

void setup() {
  myStepper.setSpeed(500); //rpm
  Serial.begin(9600);
}

void loop() {
  myStepper.step(150);
  delay(500);

  myStepper.step(-150);
  delay(500);
}

