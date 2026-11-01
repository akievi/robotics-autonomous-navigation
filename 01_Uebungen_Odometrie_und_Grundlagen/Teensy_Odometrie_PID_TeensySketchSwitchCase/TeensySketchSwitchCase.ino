#include <Servo.h>

Servo servoRight;
Servo servoLeft;

byte pinServoLeft = 8;
byte pinServoRight = 16;
byte pinFeedbackLeft = 7;
byte pinFeedbackRight = 15;

unsigned long highRight = 0, highLeft = 0;
unsigned long lowRight = 0, lowLeft = 0;
char keyCommand;
double yaw;
double yawFiltered;
int baseRight, baseLeft;
double dutyCycleRight = 0.0, dutyCycleLeft = 0.0;
unsigned long cycleTimeRight, cycleTimeLeft;
double thetaRight = 0.0, thetaLeft = 0.0;
double thetaRightPrev = 0.0, thetaLeftPrev = 0.0;
double deltaThetaRight = 0, deltaThetaLeft = 0;
double totalThetaRight = 0, totalThetaLeft = 0;
double radDurchmesser = 0.068; // Durhchmesser eines Rads
double radUmfang = radDurchmesser * 3.14159; // Umfang eines Rades (Durchmesser 68mm) 
double drehDurchmesser = 0.103 * 2; // Abstand Räder (103mm) -> Radius beim drehen
double drehUmfang = drehDurchmesser * 3.14159;

double yawStart = 0.0;
double distanceTraveled = 0.0;
double deltaYaw = 0.0;
double Kp = 35.0 * 0.2; // Kp_max ist ungefähr 35
double Kd = 0.0; // D-Anteil schadet?
double Ki = 5.0;
double integralYaw  = 0.0;
double lastDeltaYaw = 0.0;
int correctionLeft = 0, correctionRight = 0;
unsigned long lastTime = millis();

void printInfos();
void calculateThetaRight();
void calculateThetaLeft();
void driveSetDistance(double targetDistance_m, int dir);
void turnSetAngle(double targetAngle);
void turnExactAngle(double deltaAngle);
void readYawFromSerial();
void driveHexagon();

void setup(){
  Serial.begin(115200);
  pinMode(pinFeedbackRight, INPUT);
  pinMode(pinFeedbackLeft, INPUT);
} 

void loop(){
  if (Serial.available() > 0){
    if ((byte)Serial.peek() == 0xFF) {
      readYawFromSerial(); 
      Serial.println(yawFiltered);
    }
    else {
      keyCommand = (char)Serial.read();
      switch(keyCommand) {
        case 'r':
          turnSetAngle(90);
          break;
        case 'k':
          driveSetDistance(3, 1); // 1m vorwärts
          break;
        case 'j':
          turnExactAngle(90); // 90° Drehung
          delay(1000);
          turnExactAngle(90); // 90° Drehung
          delay(1000);
          turnExactAngle(90); // 90° Drehung
          delay(1000);
          turnExactAngle(90); // 90° Drehung
          break;
        case 'h':
          for (int i = 0; i < 6; i++){
            driveSetDistance(0.5, 1);
            delay(1000);
            turnExactAngle(60);
            for (int j = 0; j < 100000; j++){
              readYawFromSerial();
              Serial.print("Yaw: ");
              Serial.println(yawFiltered);
            }
            delay(1000);
          }
          break;
        case 'g':
          for (int i = 0; i < 6; i++){
            driveSetDistance(0.5, 1);
            delay(1000);
            turnExactAngle(-60);
            for (int j = 0; j < 100000; j++){
              readYawFromSerial();
              Serial.print("Yaw: ");
              Serial.println(yawFiltered);
            }
            delay(1000);
          }
          break;
        default: break;
      }
    }
  }
}

void printInfos(){
  Serial.print("ThetaRight: ");
  Serial.print(thetaRight);
  Serial.print(" , ThetaLeft: ");
  Serial.println(thetaLeft);
  Serial.print("deltaThetaRight: ");       
  Serial.print(deltaThetaRight);
  Serial.print(" , deltaThetaLeft: ");       
  Serial.println(deltaThetaLeft);
  Serial.print("totalThetaRight: ");         
  Serial.print(totalThetaRight);
  Serial.print(" , totalThetaLeft: ");       
  Serial.println(totalThetaLeft);
}

void calculateThetaRight() {
  do {
      highRight  = pulseIn(pinFeedbackRight, HIGH, 3000UL);
      lowRight   = pulseIn(pinFeedbackRight, LOW,  3000UL);
      cycleTimeRight = highRight + lowRight;
  } while (cycleTimeRight < 1000 || cycleTimeRight > 1200);

  dutyCycleRight = ((double)highRight / (double)cycleTimeRight) * 100.0;

  thetaRightPrev = thetaRight;
  thetaRight = 359.0 - ((dutyCycleRight - 2.9) * 360.0) / (97.1 - 2.9 + 1.0);
  if (thetaRight < 0.0)   thetaRight = 0.0;
  if (thetaRight > 359.0) thetaRight = 359.0;

  double diff = thetaRight - thetaRightPrev;
  if (diff < -180.0) diff += 360.0;  // Übergang 359→0
  if (diff >  180.0) diff -= 360.0;  // Übergang 0→359

  totalThetaRight += diff;
  deltaThetaRight = diff;
}

void calculateThetaLeft() {
  do {
      highLeft  = pulseIn(pinFeedbackLeft, HIGH, 3000UL);
      lowLeft   = pulseIn(pinFeedbackLeft, LOW,  3000UL);
      cycleTimeLeft = highLeft + lowLeft;
  } while (cycleTimeLeft < 1000 || cycleTimeLeft > 1200);

  dutyCycleLeft = ((double)highLeft / (double)cycleTimeLeft) * 100.0;

  thetaLeftPrev = thetaLeft;
  thetaLeft = 359.0 - ((dutyCycleLeft - 2.9) * 360.0) / (97.1 - 2.9 + 1.0);
  if (thetaLeft < 0.0)   thetaLeft = 0.0;
  if (thetaLeft > 359.0) thetaLeft = 359.0;

  double diff = thetaLeft - thetaLeftPrev;
  if (diff < -180.0) diff += 360.0;  // Übergang 359→0
  if (diff >  180.0) diff -= 360.0;  // Übergang 0→359

  totalThetaLeft += diff;
  deltaThetaLeft = diff;
}

void driveSetDistance(double targetDistance_m, int dir) {
  readYawFromSerial();

  if (dir == 1) {
    baseRight = 1440;
    baseLeft  = 1568; // mit 1545 leichter linksdrift
  }
  else if (dir == -1) {
    baseRight = 1565;
    baseLeft  = 1444; // mit 1460 leichter linksdrift
  }
  else {
    baseRight = 1500;
    baseLeft  = 1500;
  }

  // Warmup: thetaPrev auf echte Startposition setzen
  calculateThetaRight();
  calculateThetaLeft();
  totalThetaRight = 0;
  totalThetaLeft = 0;
  distanceTraveled = 0.0;
  integralYaw = 0.0;
  lastDeltaYaw = 0.0;
  lastTime = millis();

  yawStart = yawFiltered;

  servoRight.attach(pinServoRight);
  servoLeft.attach(pinServoLeft);

  while (distanceTraveled < targetDistance_m) {
    calculateThetaRight();
    calculateThetaLeft();
    readYawFromSerial();

    // dt messen
    unsigned long now = millis();
    double dt = (now - lastTime) / 1000.0;
    lastTime = now;

    // P-Anteil: Aktueller Fehler
    deltaYaw = yawFiltered - yawStart;
    if (deltaYaw >  180.0) {
      deltaYaw -= 360.0;
    }    
    if (deltaYaw < -180.0) {
      deltaYaw += 360.0;
    }

    // I-Anteil mit Anti-Windup
    integralYaw += deltaYaw * dt;
    integralYaw = constrain(integralYaw, -30.0, 30.0);

    // D-Anteil
    double derivativeYaw;
    if (dt > 0) {
      derivativeYaw = (deltaYaw - lastDeltaYaw) / dt;
    } else {
      derivativeYaw = 0.0;
    }
    lastDeltaYaw = deltaYaw;

    // PID-Ausgabe
    double correction = Kp * deltaYaw + Ki * integralYaw + Kd * derivativeYaw;

    // Mittelwert beider Räder -> zurückgelegte Strecke
    distanceTraveled = (abs(totalThetaRight) + abs(totalThetaLeft)) / 2.0 / 360.0 * radUmfang;

    if (dir == 1) {
      if (correction > 1) {
        correctionLeft  = 0;
        correctionRight = constrain(correction, -50, 50);
      } 
      else if (correction < -1) {
        correctionLeft  = constrain(correction, -50, 50);
        correctionRight = 0;
      } 
      else {
        correctionLeft = 0;
        correctionRight = 0;
      }
    } 
    else if (dir == -1) {
      if (correction > 1) {
        correctionLeft  = 0;
        correctionRight = constrain(correction, -50, 50);
      } 
      else if (correction < -1) {
        correctionLeft  = constrain(correction, -50, 50);
        correctionRight = 0;
      } 
      else {
        correctionLeft = 0;
        correctionRight = 0;
      }
    }

    servoRight.writeMicroseconds(baseRight + correctionRight);
    servoLeft.writeMicroseconds(baseLeft + correctionLeft);

    Serial.print(correctionLeft);
    Serial.print(",");
    Serial.print(correctionRight);
    Serial.print(", Error: ");
    Serial.println(deltaYaw);
    Serial.print(", YawFiltered: ");
    Serial.println(yawFiltered);
    Serial.print(", PID: ");
    Serial.println(correction);
    delay(10);
  }

  servoRight.detach();
  servoLeft.detach();
  printInfos();
}

void turnSetAngle(double targetAngle) {
  double targetDistance_m = (abs(targetAngle) / 360 * drehUmfang) * (1.03); // Korrekturfaktor für Ungenauigkeiten;
  double distanceTraveledLeft = 0.0;
  double distanceTraveledRight = 0.0;

  if (targetAngle < 0) {
    baseRight = 1555;
    baseLeft = 1558;
  }
  else if (targetAngle > 0) {
    baseRight = 1450;
    baseLeft = 1454;
  }
  else {
    baseRight = 1500;
    baseLeft = 1500;
  }

  // Warmup: thetaPrev auf echte Startposition setzen
  calculateThetaRight();
  calculateThetaLeft();
  totalThetaRight = 0;
  totalThetaLeft = 0;

  servoRight.attach(pinServoRight);
  servoLeft.attach(pinServoLeft);

  while (distanceTraveledLeft + distanceTraveledRight < targetDistance_m) {
    calculateThetaRight();
    calculateThetaLeft();

    distanceTraveledLeft = (abs(totalThetaLeft) / 360.0) * radUmfang;
    distanceTraveledRight = (abs(totalThetaRight) / 360.0) * radUmfang;
    
    if (distanceTraveledLeft < targetDistance_m / 2) {
      servoLeft.writeMicroseconds(baseLeft);
    }
    else {
      servoLeft.writeMicroseconds(1500); // Stopp
    }
    if (distanceTraveledRight < targetDistance_m / 2) {
      servoRight.writeMicroseconds(baseRight);
    }
    else {
      servoRight.writeMicroseconds(1500); // Stopp
    }
  }
  servoRight.detach();
  servoLeft.detach();

  printInfos();
}

void turnExactAngle(double deltaAngle) {
  readYawFromSerial();
  double targetAngle = yaw + deltaAngle;

  if (targetAngle >  180.0) targetAngle -= 360.0;
  if (targetAngle < -180.0) targetAngle += 360.0;

  int baseSpeedRight = 1450;
  int baseSpeedLeft  = 1455;
  if (deltaAngle < 0) {
    baseSpeedRight = 1555;
    baseSpeedLeft  = 1560;
  }

  calculateThetaRight();
  calculateThetaLeft();
  totalThetaRight = 0;
  totalThetaLeft  = 0;

  servoRight.attach(pinServoRight);
  servoLeft.attach(pinServoLeft);

  double error = deltaAngle;
  while (error > 0.7 || error < -0.7) {
    calculateThetaRight();
    calculateThetaLeft();
    readYawFromSerial();

    double wheelDiff = abs(totalThetaRight) - abs(totalThetaLeft);
    int correction = (int)(wheelDiff * 2.0);

    servoRight.writeMicroseconds(baseSpeedRight + correction);
    servoLeft.writeMicroseconds(baseSpeedLeft - correction);

    error = targetAngle - yaw;
    
    if (error >  180.0) error -= 360.0;
    if (error < -180.0) error += 360.0;
  }

  Serial.print("Final Error: ");
  Serial.println(error);

  servoRight.detach();
  servoLeft.detach();
}

void readYawFromSerial() {
  while (Serial.available() > 0 && (byte)Serial.peek() != 0xFF) {
    Serial.read(); // Garbage vor 0xFF verwerfen
  }
  if (Serial.available() > 0 && (byte)Serial.peek() == 0xFF) {
    Serial.read(); // 0xFF konsumieren
    unsigned long t = millis();
    while (Serial.available() < 8 && millis() - t < 10) {}
    if (Serial.available() >= 8) {
      Serial.readBytes((char*)&yaw, sizeof(yaw));
      // Tiefpassfilter: yawFiltered folgt yaw mit Faktor 0.05
      double diff = yaw - yawFiltered;
      if (diff >  180.0) diff -= 360.0;
      if (diff < -180.0) diff += 360.0;
      yawFiltered += 0.05 * diff;
      if (yawFiltered >  180.0) yawFiltered -= 360.0;
      if (yawFiltered < -180.0) yawFiltered += 360.0;
    }
  }
}
