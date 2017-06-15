#include "Arduino.h"
#include "Driver.h"

Driver::Driver(int numberOfSteps, int dirPin, int stepPin)
{
  _dirPin = dirPin;
  _stepPin = stepPin;
  _numberOfSteps = numberOfSteps;
  _currStep = 0;
  _lastEdge = micros();
	
  pinMode(_dirPin, OUTPUT);
  pinMode(_stepPin, OUTPUT);
	
	setSpeed(0);
}

void Driver::setSpeed(long whatSpeed)
{
	_stepDelay = 60L * 1000L * 1000L  / _numberOfSteps / whatSpeed / 2;
}

int Driver::currStep(void)
{
  return _currStep;
}

void Driver::step(int targetStep)
{
	if (targetStep == _currStep);
 
  else
  {
    if (targetStep > _currStep && _direction!=1) {
      digitalWrite(_dirPin, HIGH);
      _direction = 1;
    }

    if (targetStep < _currStep && _direction!=0) {
      digitalWrite(_dirPin, LOW);
      _direction = 0;
    }

    if (digitalRead(_stepPin)==HIGH && micros()-_lastEdge >= _stepDelay) {
      digitalWrite(_stepPin, LOW);
      _lastEdge = micros();
    }

    if (digitalRead(_stepPin)==LOW && micros()-_lastEdge >= _stepDelay) {
      digitalWrite(_stepPin, HIGH);
      _lastEdge = micros();
      _direction == 1 ? _currStep++ : _currStep--;
    }
  }
}
