#include "Arduino.h"
#include "Driver.h"

Driver::Driver(int numberOfSteps, int dirPin, int stepPin)
{
  _dirPin = dirPin;
  _stepPin = stepPin;
  _numberOfSteps = numberOfSteps;
	
  pinMode(_dirPin, OUTPUT);
  pinMode(_stepPin, OUTPUT);
	
	setSpeed(0);
}

void Driver::setSpeed(long whatSpeed)
{
	_stepDelay = 60L * 1000L * 1000L  / _numberOfSteps / whatSpeed / 2;
  //stepdelay = time taken for one step / 2
  //time taken for one rev = 1 / rpm * 60 * 10^6
  //time taken for one step = 1 / rpm * 60 * 10^6 / no_of_steps
}

void Driver::step(int steps_to_move)
{
	if (steps_to_move > 0) {
		digitalWrite(_dirPin, HIGH);
		delayMicroseconds(1);
		_direction = 1;
	}
	if (steps_to_move < 0) {
		digitalWrite(_dirPin, LOW);
		delayMicroseconds(1);
		_direction = 0;
	}
	
	steps_to_move = abs(steps_to_move);
	
  for(int i = 0; i < steps_to_move; i++)
  {
    digitalWrite(_stepPin, HIGH);
    delayMicroseconds(_stepDelay);
    digitalWrite(_stepPin, LOW);
    delayMicroseconds(_stepDelay);
  }
}
