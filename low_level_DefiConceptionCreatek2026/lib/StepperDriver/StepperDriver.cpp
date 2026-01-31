#include "StepperDriver.h"

StepperDriver::StepperDriver(uint8_t dirPin, uint8_t stepPin, uint8_t enablePin)
    : _dirPin(dirPin), _stepPin(stepPin), _enablePin(enablePin), 
      _position(0), _stepDelayMicros(1000) {
}

void StepperDriver::begin() {
    pinMode(_dirPin, OUTPUT);
    pinMode(_stepPin, OUTPUT);
    if (_enablePin != 255) {
        pinMode(_enablePin, OUTPUT);
        enable();  // Enable by default
    }
    
    // Set initial direction
    digitalWrite(_dirPin, LOW);
    digitalWrite(_stepPin, LOW);
}

void StepperDriver::step(int numSteps) {
    stepWithDelay(numSteps, _stepDelayMicros);
}

void StepperDriver::stepWithDelay(int numSteps, unsigned long delayMicros) {
    if (numSteps == 0) return;
    
    int direction = (numSteps > 0) ? 1 : -1;
    numSteps = abs(numSteps);
    
    // Set direction pin
    digitalWrite(_dirPin, (direction > 0) ? HIGH : LOW);
    
    // Small delay for direction to settle
    delayMicroseconds(5);
    
    // Pulse STEP pin for each step
    for (int i = 0; i < numSteps; i++) {
        digitalWrite(_stepPin, HIGH);
        delayMicroseconds(delayMicros / 2);
        digitalWrite(_stepPin, LOW);
        delayMicroseconds(delayMicros / 2);
        
        _position += direction;
    }
}

void StepperDriver::setSpeed(unsigned long stepsPerSecond) {
    if (stepsPerSecond == 0) {
        _stepDelayMicros = 1000;  // Default to 1ms
    } else {
        _stepDelayMicros = 1000000 / stepsPerSecond;  // Convert to microseconds
    }
}

void StepperDriver::enable() {
    if (_enablePin != 255) {
        digitalWrite(_enablePin, HIGH);
    }
}

void StepperDriver::disable() {
    if (_enablePin != 255) {
        digitalWrite(_enablePin, LOW);
    }
}

long StepperDriver::getPosition() {
    return _position;
}

void StepperDriver::resetPosition() {
    _position = 0;
}
