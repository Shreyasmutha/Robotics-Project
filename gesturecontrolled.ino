#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// Pin definitions for L298N
#define IN1 9
#define IN2 10
#define IN3 5
#define IN4 6
#define ENA 11
#define ENB 3
#define BATTERY_PIN A0

Adafruit_MPU6050 mpu;
float xOffset = 0.0, yOffset = 0.0;
const float DEAD_ZONE = 2.5;
const int MAX_SPEED = 200, MIN_SPEED = 50;
const float BATTERY_THRESHOLD = 6.5;
float prevXTilt = 0.0, prevYTilt = 0.0;
bool isCircular = false;

void setup() {
  Serial.begin(9600);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  calibrateSensor();
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float xTilt = a.acceleration.x - xOffset;
  float yTilt = a.acceleration.y - yOffset;
  int xSpeed = calculateSpeed(xTilt, DEAD_ZONE);
  int ySpeed = calculateSpeed(yTilt, DEAD_ZONE);
  
  if (abs(xTilt) > DEAD_ZONE && abs(yTilt) > DEAD_ZONE) {
    if ((prevXTilt > 0 && xTilt < 0 && prevYTilt > 0 && yTilt < 0) ||
        (prevXTilt < 0 && xTilt > 0 && prevYTilt < 0 && yTilt > 0)) {
      isCircular = true;
    }
  } else {
    isCircular = false;
  }
  
  if (isCircular) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    analogWrite(ENA, 150); analogWrite(ENB, 150);
    Serial.println("Spinning Right (Circular Tilt)");
  } else if (abs(xTilt) > abs(yTilt)) {
    if (xSpeed > 0) {
      moveForward(xSpeed);
      Serial.print("Forward, Speed: "); Serial.println(xSpeed);
    } else if (xSpeed < 0) {
      moveBackward(-xSpeed);
      Serial.print("Backward, Speed: "); Serial.println(-xSpeed);
    } else {
      stopMotors();
      Serial.println("Stopped");
    }
  } else {
    if (ySpeed > 0) {
      turnRight(ySpeed);
      Serial.print("Right, Speed: "); Serial.println(ySpeed);
    } else if (ySpeed < 0) {
      turnLeft(-ySpeed);
      Serial.print("Left, Speed: "); Serial.println(-ySpeed);
    } else {
      stopMotors();
      Serial.println("Stopped");
    }
  }
  
  prevXTilt = xTilt; prevYTilt = yTilt;
  checkBattery();
  Serial.print("X-Tilt: "); Serial.print(xTilt);
  Serial.print(" Y-Tilt: "); Serial.println(yTilt);
  delay(50);
}

void calibrateSensor() {
  Serial.println("Calibrating sensor... Keep MPU6050 still");
  float xSum = 0, ySum = 0;
  const int samples = 100;
  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    xSum += a.acceleration.x;
    ySum += a.acceleration.y;
    delay(10);
  }
  xOffset = xSum / samples;
  yOffset = ySum / samples;
  Serial.println("Calibration complete");
}

int calculateSpeed(float tilt, float deadZone) {
  if (abs(tilt) < deadZone) return 0;
  float speed = map(abs(tilt) * 100, deadZone * 100, 800, MIN_SPEED, MAX_SPEED);
  speed = constrain(speed, MIN_SPEED, MAX_SPEED);
  return (tilt > 0) ? speed : -speed;
}

void moveForward(int speed) {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, speed); analogWrite(ENB, speed);
}

void moveBackward(int speed) {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(ENA, speed); analogWrite(ENB, speed);
}

void turnRight(int speed) {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(ENA, speed); analogWrite(ENB, speed);
}

void turnLeft(int speed) {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, speed); analogWrite(ENB, speed);
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0); analogWrite(ENB, 0);
}

void checkBattery() {
  int sensorValue = analogRead(BATTERY_PIN);
  float voltage = sensorValue * (5.0 / 1023.0) * 2;
  if (voltage < BATTERY_THRESHOLD) {
    Serial.println("Warning: Low battery!");
  }
}