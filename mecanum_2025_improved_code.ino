#include <PS4Controller.h>
#include <ESP32Servo.h>

// Motor PWM pins
#define C620_PWM_1 33  // Front Left
#define C620_PWM_2 25  // Back Left
#define C620_PWM_3 26  // Front Right
#define C620_PWM_4 27  // Back Right

// PWM constants
#define PWM_NEUTRAL 1450
#define DEADZONE 15
#define MAX_SPEED 100

// Servo objects
Servo frontLeftWheel;
Servo backLeftWheel;
Servo frontRightWheel;
Servo backRightWheel;

// Motor control variables
int16_t M1_Target = 0, M2_Target = 0, M3_Target = 0, M4_Target = 0;
bool debugMode = true;

void setup() {
  Serial.begin(115200);
  PS4.begin("ec:64:c9:82:8d:1a");
  while (!PS4.isConnected()) {
    delay(500);
    Serial.println("Waiting for PS4 controller...");
  }

  // Attach servos
  frontLeftWheel.attach(C620_PWM_1);
  backLeftWheel.attach(C620_PWM_2);
  frontRightWheel.attach(C620_PWM_3);
  backRightWheel.attach(C620_PWM_4);

  // Initialize motors to neutral
  calibrateC620();
  Serial.println("System ready");
}

void loop() {
  if (!PS4.isConnected()) {
    stopAllMotors();
    return;
  }

  updateMotorTargets();
  writeMotorOutputs();
  delay(10);

  if (debugMode) {
    printDebugInfo();
    delay(100);
  } else {
    delay(20);
  }
}

void updateMotorTargets() {
  // Get joystick values and apply deadzone
  int8_t lx = applyDeadzone(PS4.LStickX());
  int8_t ly = applyDeadzone(PS4.LStickY()); // Keep original Y direction
  int8_t rx = applyDeadzone(PS4.RStickX());

  // Scale inputs smoothly
  float forward = smoothScale(ly) * MAX_SPEED;
  float strafe = smoothScale(lx) * MAX_SPEED;
  float rotation = smoothScale(rx) * MAX_SPEED * 0.7f; // Reduce rotation sensitivity

  // Mecanum wheel kinematics
  M1_Target = constrain(forward + strafe - rotation, -MAX_SPEED, MAX_SPEED);  // Front Left +
  M2_Target = constrain(forward - strafe - rotation, -MAX_SPEED, MAX_SPEED);  // Back Left +
  M3_Target = constrain(forward - strafe + rotation, -MAX_SPEED, MAX_SPEED);  // Front Right -
  M4_Target = constrain(forward + strafe + rotation, -MAX_SPEED, MAX_SPEED);  // Back Right -
}

int8_t applyDeadzone(int8_t value) {
  return (abs(value) < DEADZONE) ? 0 : value;
}

float smoothScale(int8_t input) {
  if (input == 0) return 0.0f;

  float normalized = input / 128.0f;
  // Linear scaling for full range (change to cubic if you want smoother low speeds)
  return normalized;
}

void writeMotorOutputs() {
  frontLeftWheel.writeMicroseconds(PWM_NEUTRAL + M1_Target);
  backLeftWheel.writeMicroseconds(PWM_NEUTRAL + M2_Target);
  frontRightWheel.writeMicroseconds(PWM_NEUTRAL + M3_Target);
  backRightWheel.writeMicroseconds(PWM_NEUTRAL + M4_Target);
}

void stopAllMotors() {
  M1_Target = M2_Target = M3_Target = M4_Target = 0;
  frontLeftWheel.writeMicroseconds(PWM_NEUTRAL);
  backLeftWheel.writeMicroseconds(PWM_NEUTRAL);
  frontRightWheel.writeMicroseconds(PWM_NEUTRAL);
  backRightWheel.writeMicroseconds(PWM_NEUTRAL);
}

void calibrateC620() {
  frontLeftWheel.writeMicroseconds(2000);
  backLeftWheel.writeMicroseconds(2000);
  frontRightWheel.writeMicroseconds(2000);
  backRightWheel.writeMicroseconds(2000);
  delay(100);

  frontLeftWheel.writeMicroseconds(1000);
  backLeftWheel.writeMicroseconds(1000);
  frontRightWheel.writeMicroseconds(1000);
  backRightWheel.writeMicroseconds(1000);
  delay(100);

  frontLeftWheel.writeMicroseconds(1500);
  backLeftWheel.writeMicroseconds(1500);
  frontRightWheel.writeMicroseconds(1500);
  backRightWheel.writeMicroseconds(1500);
  delay(1000);  

  Serial.println("Calibration complete");
}

void printDebugInfo() {
  Serial.print("Joy: LX="); Serial.print(PS4.LStickX());
  Serial.print(" LY="); Serial.print(PS4.LStickY());
  Serial.print(" RX="); Serial.print(PS4.RStickX());

  Serial.print(" | PWM: M1="); Serial.print(PWM_NEUTRAL + M1_Target);
  Serial.print(" M2="); Serial.print(PWM_NEUTRAL + M2_Target);
  Serial.print(" M3="); Serial.print(PWM_NEUTRAL + M3_Target);
  Serial.print(" M4="); Serial.println(PWM_NEUTRAL + M4_Target);

  Serial.print(" | PWM: M1_Target="); Serial.print(M1_Target);
  Serial.print(" M2="); Serial.print(M2_Target);
  Serial.print(" M3="); Serial.print(M2_Target);
  Serial.print(" M4="); Serial.print(M2_Target);
}