#ifndef STEPPER_DRIVER_H
#define STEPPER_DRIVER_H

#include <Arduino.h>

class StepperDriver {
public:
    // Constructor: takes DIR and STEP pin numbers
    StepperDriver(uint8_t dirPin, uint8_t stepPin, uint8_t enablePin = 255);
    
    // Initialize pins as outputs
    void begin();
    
    // Move motor by specified number of steps
    // Positive = forward, Negative = backward
    void step(int numSteps);
    
    // Move motor by specified number of steps with delay between steps
    void stepWithDelay(int numSteps, unsigned long delayMicros);
    
    // Set stepping speed (steps per second)
    void setSpeed(unsigned long stepsPerSecond);
    
    // Enable/disable motor (if enable pin is configured)
    void enable();
    void disable();
    
    // Get current position
    long getPosition();
    
    // Reset position counter
    void resetPosition();

private:
    uint8_t _dirPin;
    uint8_t _stepPin;
    uint8_t _enablePin;
    long _position;
    unsigned long _stepDelayMicros;
    
    // Helper function to perform a single step
    void singleStep(int direction);
};

#endif // STEPPER_DRIVER_H
