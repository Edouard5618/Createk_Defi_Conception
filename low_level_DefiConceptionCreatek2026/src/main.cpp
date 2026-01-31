#include <Arduino.h>
#include <Servo.h>
#include <AccelStepper.h>

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
const float MAX_SPEED = 2000.0;        // Maximum steps per second
const float ACCELERATION = 400.0;     // Steps per second^2
// const float SPEED_FACTOR = 80.0;      // Speed multiplier for position difference

const float DEADZONE = 10;            // Ignore position changes smaller than this (steps)

void parseCommand(String cmd);
void moveStepperX(int targetX);
void moveStepperY(int targetY);
void moveServo(int angle);

// AccelStepper instances (interface type 1 = DRIVER with STEP and DIR pins)
AccelStepper stepperX(AccelStepper::DRIVER, STEPPER_X_STEP, STEPPER_X_DIR);
AccelStepper stepperY(AccelStepper::DRIVER, STEPPER_Y_STEP, STEPPER_Y_DIR);
Servo servo;

int currentX = 0, currentY = 0;

void setup() {
    Serial.begin(115200);
    
    // Initialize steppers with AccelStepper
    stepperX.setMaxSpeed(MAX_SPEED);
    stepperX.setAcceleration(ACCELERATION);
    stepperX.setCurrentPosition(0);
    
    stepperY.setMaxSpeed(MAX_SPEED);
    stepperY.setAcceleration(ACCELERATION);
    stepperY.setCurrentPosition(0);
    
    // Initialize enable pins
    if (STEPPER_X_ENABLE != 255) {
        pinMode(STEPPER_X_ENABLE, OUTPUT);
        digitalWrite(STEPPER_X_ENABLE, LOW);  // Enable motor (active LOW for most drivers)
    }
    if (STEPPER_Y_ENABLE != 255) {
        pinMode(STEPPER_Y_ENABLE, OUTPUT);
        digitalWrite(STEPPER_Y_ENABLE, LOW);  // Enable motor (active LOW for most drivers)
    }
    
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
    // Speed-based movement - continuously run at set speed
    stepperX.runSpeed();
    stepperY.runSpeed();
    
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();  // Remove whitespace/newlines
        
        // Speed-based command format: "X100 Y200 S255 R1:1 R2:0" where X/Y are speeds in steps/sec
        parseCommand(command);
    }
}

void parseCommand(String cmd) {
    int speedX = 0, speedY = 0, servo_val = -1, relay1 = -1, relay2 = -1;
    
    // Parse format: "X100 Y200 S255 R1:1 R2:0" where X/Y are speed values
    int xIdx = cmd.indexOf("X");
    int yIdx = cmd.indexOf("Y");
    int sIdx = cmd.indexOf("S");
    int r1Idx = cmd.indexOf("R1:");
    int r2Idx = cmd.indexOf("R2:");
    
    if (xIdx != -1) {
        int endIdx = cmd.indexOf(' ', xIdx);
        if (endIdx == -1) endIdx = cmd.length();
        speedX = cmd.substring(xIdx + 1, endIdx).toInt();
    }
    
    if (yIdx != -1) {
        int endIdx = cmd.indexOf(' ', yIdx);
        if (endIdx == -1) endIdx = cmd.length();
        speedY = cmd.substring(yIdx + 1, endIdx).toInt();
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
    
    moveStepperX(speedX);
    moveStepperY(speedY);
    if (servo_val != -1) moveServo(servo_val);
    if (relay1 != -1) digitalWrite(RELAY1_PIN, relay1 ? HIGH : LOW);
    if (relay2 != -1) digitalWrite(RELAY2_PIN, relay2 ? HIGH : LOW);

    Serial.println("Command executed: " + cmd);
    while(Serial.available()) Serial.read();  // Clear serial buffer
}

void moveStepperX(int speedX) {
    if (abs(speedX) < DEADZONE) {
        stepperX.setSpeed(0);
    } else {
        // Clamp speed to max
        if (speedX > MAX_SPEED) speedX = MAX_SPEED;
        if (speedX < -MAX_SPEED) speedX = -MAX_SPEED;
        stepperX.setSpeed(speedX);
    }
}

void moveStepperY(int speedY) {
    if (abs(speedY) < DEADZONE) {
        stepperY.setSpeed(0);
    } else {
        // Clamp speed to max
        if (speedY > MAX_SPEED) speedY = MAX_SPEED;
        if (speedY < -MAX_SPEED) speedY = -MAX_SPEED;
        stepperY.setSpeed(speedY);
    }
}

void moveServo(int angle) {
    servo.write(angle);
}