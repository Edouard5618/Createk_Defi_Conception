#include <Arduino.h>
#include <Servo.h>
#include "StepperDriver.h"

// Stepper motor pins (DIR and STEP)
#define STEPPER_X_DIR 2
#define STEPPER_X_STEP 3
#define STEPPER_X_ENABLE 4  // Optional: set to 255 if not using

#define STEPPER_Y_DIR 6
#define STEPPER_Y_STEP 7
#define STEPPER_Y_ENABLE 8  // Optional: set to 255 if not using

// Servo and relay pins
#define SERVO_PIN 10
#define RELAY1_PIN 11
#define RELAY2_PIN 12

// Control parameters
const float Kp = 1.0;  // Proportional gain for stepper control

// Tuning parameters for angle-based control
const float STEPS_PER_MM_X = 10.0;  // Adjust based on your stepper and mechanism
const float MAX_SPEED_X = 200.0; // Max speed in steps per second
const float STEPS_PER_MM_Y = 10.0;  // Adjust based on your stepper and mechanism
const float MAX_SPEED_Y = 200.0; // Max speed in steps per second

const float DEADZONE = 10;              // Ignore angles smaller than this (steps)

void parseCommand(String cmd);
void moveStepperX(int targetX);
void moveStepperY(int targetY);
void moveServo(int angle);

// Stepper driver instances (DIR pin, STEP pin, ENABLE pin)
StepperDriver stepperX(STEPPER_X_DIR, STEPPER_X_STEP, STEPPER_X_ENABLE);
StepperDriver stepperY(STEPPER_Y_DIR, STEPPER_Y_STEP, STEPPER_Y_ENABLE);
Servo servo;

int currentX = 0, currentY = 0;

void setup() {
    Serial.begin(115200);
    
    // Initialize steppers
    stepperX.begin();
    stepperY.begin();
    stepperX.setSpeed(100);  // 100 steps per second
    stepperY.setSpeed(100);
    
    // Initialize servo
    servo.attach(SERVO_PIN);
    
    // Initialize relays
    pinMode(RELAY1_PIN, OUTPUT);
    pinMode(RELAY2_PIN, OUTPUT);
    digitalWrite(RELAY1_PIN, LOW);
    digitalWrite(RELAY2_PIN, LOW);
    
    Serial.println("Ready");
}

void loop() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();  // Remove whitespace/newlines

        // Legacy position-based command format: "X100 Y200 S255 R1:1 R2:0"
        parseCommand(command);
    }
}

void parseCommand(String cmd) {
    int x = -999999, y = -999999, servo_val = -1, relay1 = -1, relay2 = -1;
    
    // Parse format: "X100 Y200 S255 R1:1 R2:0"
    int xIdx = cmd.indexOf("X");
    int yIdx = cmd.indexOf("Y");
    int sIdx = cmd.indexOf("S");
    int r1Idx = cmd.indexOf("R1:");
    int r2Idx = cmd.indexOf("R2:");
    
    if (xIdx != -1) {
        int endIdx = cmd.indexOf(' ', xIdx);
        if (endIdx == -1) endIdx = cmd.length();
        x = cmd.substring(xIdx + 1, endIdx).toInt();
    }
    
    if (yIdx != -1) {
        int endIdx = cmd.indexOf(' ', yIdx);
        if (endIdx == -1) endIdx = cmd.length();
        y = cmd.substring(yIdx + 1, endIdx).toInt();
    }
    
    if (sIdx != -1) {
        int endIdx = cmd.indexOf(' ', sIdx);
        if (endIdx == -1) endIdx = cmd.length();
        servo_val = cmd.substring(sIdx + 1, endIdx).toInt();
    }
    
    if (r1Idx != -1) {
        relay1 = cmd.substring(r1Idx + 3, r1Idx + 4).toInt();
    }
    
    if (r2Idx != -1) {
        relay2 = cmd.substring(r2Idx + 3, r2Idx + 4).toInt();
    }
    
    if (x != -999999) moveStepperX(x);
    if (y != -999999) moveStepperY(y);
    if (servo_val != -1) moveServo(servo_val);
    if (relay1 != -1) digitalWrite(RELAY1_PIN, relay1 ? HIGH : LOW);
    if (relay2 != -1) digitalWrite(RELAY2_PIN, relay2 ? HIGH : LOW);

    Serial.println("Command executed: " + cmd);
    while(Serial.available()) Serial.read();  // Clear serial buffer
}

void moveStepperX(int targetX) {
    int steps = targetX - currentX;
    if(targetX - currentX > 0){
        steps = 10;
    }
    else if(targetX - currentX < 0){
        steps = -10;
    }

    if (abs(steps) < DEADZONE) return;  // Within deadzone, ignore

    float speed = abs(steps) * Kp;
    if (speed > MAX_SPEED_X) speed = MAX_SPEED_X;
    if (speed < -MAX_SPEED_X) speed = -MAX_SPEED_X;
    stepperX.setSpeed(speed);  // Ensure speed is set

    stepperX.step(steps);
    currentX = targetX;
}

void moveStepperY(int targetY) {
    int steps = targetY - currentY;
    if(targetY - currentY > 0){
        steps = 10;
    }
    else if(targetY - currentY < 0){
        steps = -10;
    }
    if (abs(steps) < DEADZONE) return;  // Within deadzone, ignore

    float speed = abs(steps) * Kp;
    if (speed > MAX_SPEED_Y) speed = MAX_SPEED_Y;
    if (speed < -MAX_SPEED_Y) speed = -MAX_SPEED_Y;
    stepperY.setSpeed(speed);  // Ensure speed is set

    stepperY.step(steps);
    currentY = targetY;
}

void moveServo(int angle) {
    servo.write(angle);
}