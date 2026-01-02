#include <Wire.h>
#include <MPU6050_light.h> 
#include <avr/wdt.h> 

#define ENA 9
#define IN1 11   
#define IN2 12

#define ENB 10
#define IN3 A0
#define IN4 A1

#define TRIG_F 3
#define ECHO_F 2
#define TRIG_L 5
#define ECHO_L 4
#define TRIG_R 7
#define ECHO_R 6

#define RELAY_PIN 8 


#define DRIVE_SPEED   70   
#define TURN_SPEED    90   
#define STOP_DIST     35    
#define GYRO_KP       5.0   

MPU6050 mpu(Wire);
float targetHeading = 0;
bool mpuAlive = true; 

void setup() {
  wdt_disable();
  Serial.begin(115200); 
  
  
  Wire.begin();
  Wire.setWireTimeout(3000, true); 
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  
  pinMode(TRIG_F, OUTPUT); pinMode(ECHO_F, INPUT);
  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);
  
  pinMode(RELAY_PIN, OUTPUT);

  digitalWrite(RELAY_PIN, HIGH); 
  stopMotors();

  Serial.println(F(">>> SYSTEM BOOT: FAULT TOLERANT <<<"));
  
  byte status = mpu.begin();
  if(status != 0) {
    Serial.println(F("WARNING: MPU6050 Failed! Switching to TIME-BASED mode."));
    mpuAlive = false;
  } else {
    Serial.println(F("MPU6050 Connected. Calibrating..."));
    delay(1000);
    mpu.calcOffsets();
    mpuAlive = true;
  }
  
  Serial.println(F("Starting in 4 seconds..."));
  delay(4000);
  
  if(mpuAlive) targetHeading = mpu.getAngleZ(); 
  
  Serial.println(F("ENGAGING MOTORS"));
  digitalWrite(RELAY_PIN, LOW); 
  
  wdt_enable(WDTO_1S); 
}

void loop() {
  wdt_reset(); 

  if (mpuAlive) {
    mpu.update();
    if (isnan(mpu.getAngleZ())) {
    }
  }
  int distFront = checkSonarAvg(TRIG_F, ECHO_F);
  static unsigned long lastP = 0;
  if (millis() - lastP > 200) {
    Serial.print(F("S: ")); Serial.print(distFront);
    if(mpuAlive) { Serial.print(F(" | H: ")); Serial.println(mpu.getAngleZ()); }
    else { Serial.println(F(" | H: N/A")); }
    lastP = millis();
  }

  if (distFront < STOP_DIST && distFront > 0) {
    Serial.println(F("OBSTACLE"));
    stopMotors();
    digitalWrite(RELAY_PIN, HIGH); 
    delay(200);

    Serial.println(F("Rev"));
    moveRaw(-140, -140);
    delay(600);
    stopMotors();

    int distLeft  = readSonarRaw(TRIG_L, ECHO_L);
    int distRight = readSonarRaw(TRIG_R, ECHO_R);

    if (distLeft > distRight) {
      Serial.println(F("L Turn"));
      safeTurn(90); 
    } else {
      Serial.println(F("R Turn"));
      safeTurn(-90); 
    }

    if(mpuAlive) targetHeading = mpu.getAngleZ(); 
    digitalWrite(RELAY_PIN, LOW);
    delay(200);
  } 
  else {
    if (mpuAlive) {
      driveStraightGyro();
    } else {
      moveRaw(DRIVE_SPEED, DRIVE_SPEED); 
    }
  }
}


void driveStraightGyro() {
  float currentAngle = mpu.getAngleZ();
  float error = targetHeading - currentAngle;
  int correction = error * GYRO_KP;
  correction = constrain(correction, -40, 40);
  moveRaw(DRIVE_SPEED - correction, DRIVE_SPEED + correction);
}

void safeTurn(int angle) {
  if (mpuAlive) {
    float startAngle = mpu.getAngleZ();
    float target = startAngle + angle;
    unsigned long s = millis();
    
    if (angle > 0) {
      moveRaw(-TURN_SPEED, TURN_SPEED);
      while((mpu.getAngleZ() < target) && (millis() - s < 2000)) { 
        mpu.update(); wdt_reset(); 
      }
    } else {
      moveRaw(TURN_SPEED, -TURN_SPEED);
      while((mpu.getAngleZ() > target) && (millis() - s < 2000)) { 
        mpu.update(); wdt_reset(); 
      }
    }
  } 
  else {
    if (angle > 0) {
      moveRaw(-TURN_SPEED, TURN_SPEED);
      delay(900); 
    } else {
      moveRaw(TURN_SPEED, -TURN_SPEED);
      delay(900);
    }
  }
  stopMotors();
}


void moveRaw(int left, int right) {
  // Left
  if (left > 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
  else if (left < 0) { digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); }
  else { digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); }
  analogWrite(ENB, abs(left));

  // Right
  if (right > 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
  else if (right < 0) { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); }
  else { digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); }
  analogWrite(ENA, abs(right));
}

void stopMotors() { moveRaw(0, 0); }

int checkSonarAvg(int trig, int echo) {
  int t = 0; int c = 0;
  for(int i=0; i<3; i++) {
    int r = readSonarRaw(trig, echo);
    if(r > 0 && r < 400) { t += r; c++; }
    delay(5);
  }
  if(c == 0) return 999;
  return t / c;
}

int readSonarRaw(int trig, int echo) {
  digitalWrite(trig, LOW); delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long d = pulseIn(echo, HIGH, 20000); 
  if (d == 0) return 999;
  return d * 0.034 / 2;
}