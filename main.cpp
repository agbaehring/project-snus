#include <Arduino.h>
#include <Wire.h>
#include <Zumo32U4.h>
#include <config>
#include <Zumo32U4IRPulses.h>
#include <HCSR04.h>

// --- Zumo objects ---
Zumo32U4Encoders encoders;
Zumo32U4ProximitySensors proxSensors;
Zumo32U4LineSensors lineSensors;
Zumo32U4IMU imu;
Zumo32U4OLED oled; 
Zumo32U4Buzzer buzzer;
Zumo32U4ButtonA buttonA;   // kept only the button actually used
Zumo32U4Motors motors;

// --- Function prototypes ---
void turnSensorSetup();
void turnSensorReset();
void turnSensorUpdate();
float getTurnAngleInDegrees(); // returns float degrees
void stop();
bool detectLine();
bool detectWall();
double getDistance();
void resetEncoders();
void navigateObstacle();
bool detectWallSide();
long microsecondsToCentimeters(long microseconds);
void ultra();

// Ultrasonic pins
const int trigPin = 22;
const int echoPin = 18;

// Globals
const int16_t maxSpeed = 400;
int cm;
int Counter = 0;

// --- Gyro / turn sensor globals (single set) ---
//int32_t turnAngle = 0;       // fixed-point accumulator (16.16)
//int16_t turnRate = 0;        // latest gyro rate
//int32_t gyroOffset = 0;
//unsigned long gyroLastUpdate = 0;
#include <TurnSensor>
// ---------------- setup ----------------
void setup() {
  Serial.begin(9600);

  proxSensors.initThreeSensors();
  lineSensors.initThreeSensors();

  turnSensorSetup();

  oled.clear();
  oled.print(F("Prox cal"));

  // --- Calibrate line sensors ---
  for (int j = 0; j < 100; j++) {
    lineSensors.calibrate();
    oled.clear();
    oled.print(F("Line cal"));
    delay(20);
  }

  oled.clear();
  oled.print(F("Cal done!"));

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  oled.clear();
  oled.print(F("Ready!"));
  delay(500);
}

// ---------------- movement ----------------
void stop() {
  motors.setSpeeds(0, 0);
}
float headingOffset = 0;  // global variable

// Call this to turn “virtually” 90 degrees
 void turnVirtual(float angle) {
    turnAngle += (angle*-turnAngle1);   

    for (int i = 0; i < 1000; i++) {
    turnSensorUpdate();

  // Calculate the motor turn speed using proportional and
  // derivative PID terms.  Here we are a using a proportional
  // constant of 56 and a derivative constant of 1/20.
  int32_t turnSpeed = 45 -(int32_t)turnAngle  / (turnAngle1 / 56)
    - turnRate / 20; // 20 is original

  // Constrain our motor speeds to be between
  // -maxSpeed and maxSpeed.
  turnSpeed = constrain(turnSpeed, -BASE_SPEED, BASE_SPEED);

  motors.setSpeeds(-turnSpeed, turnSpeed); 
} 
  motors.setSpeeds(0, 0);
  headingOffset = ((int32_t)turnAngle >> 16) * 360.0 / 65536.0;
  turnSensorUpdate();
}

// PD controller — preserves Zumo PD constants (56 and 1/20)
void straight() {
    turnSensorUpdate();

    // current heading in degrees
    float currentHeading = ((int32_t)turnAngle >> 16) * 360.0 / 65536.0;

    // compute error relative to offset
    float error = currentHeading - headingOffset;

    // normalize error to [-180, 180]
    if (error > 180) error -= 360;
    if (error < -180) error += 360;

    // PD control (same as before)
    int32_t turnSpeed = -(error * 56) - turnRate / 20;

    turnSpeed = constrain(turnSpeed, -BASE_SPEED, BASE_SPEED);

    motors.setSpeeds(BASE_SPEED - turnSpeed, BASE_SPEED + turnSpeed);

    
}

bool navLeft() {
    if ((Counter >= 0 && Counter < 6) && (Counter % 2 == 0)) return false;
    if ((Counter >= 9 && Counter < 15) && (Counter % 2 != 0)) return false;
    return true;
}

// ---------------- line navigation ----------------
void navigateLine() {
  if (navLeft()) turnVirtual(-TURN_ANGLE);
  else turnVirtual(TURN_ANGLE);
    while(getDistance() < 20){
    straight();}
    stop();
    if (navLeft()) turnVirtual(-TURN_ANGLE);
    else turnVirtual(TURN_ANGLE);}

// ---------------- obstacle navigation ----------------
void navigateObstacle() {

  // Approach obstacle a bit
  for (int k = 0; k < 50; k++) {
    ultra();
    #if SHOW_STATUS
    oled.clear();
    oled.print(F("Dist: "));
    oled.print(cm);
    #endif

    if (cm <= 4) {
      #if SHOW_STATUS
      oled.clear();
      oled.print(F("Close!"));
      #endif
      break;
    }

    if (detectLine()) {
      stop();
      return;
    }

    motors.setSpeeds(BASE_SPEED / 2, BASE_SPEED / 2);
  }

  stop();
  if (navLeft()) turnVirtual(-90);
  else turnVirtual(90);

  resetEncoders();
  while (detectWallSide()) {
    straight();
  }
  double firstLegDistance = getDistance();

  resetEncoders();
  while(getDistance() < 20){ // added to ensure it moves forward after losing the wall
    straight();}

  double secondLegDistance = getDistance();

  if (navLeft()) turnVirtual(90);
  else turnVirtual(-90);

  resetEncoders();
  while(getDistance() < 20){ // added to ensure it moves forward to detect wall
    straight();}

  while (detectWallSide()) {
    straight();
    if (detectLine()) { stop(); return; }
  }
  resetEncoders();
  while(getDistance() < 20){ // added to ensure it moves forward after losing the wall
    straight();}


  if (navLeft()) turnVirtual(90);
  else turnVirtual(-90);

  double totalDistance = firstLegDistance + secondLegDistance;

  resetEncoders();
  while (getDistance() < totalDistance) {
    straight();
  }

  if (navLeft()) turnVirtual(-90);
  else turnVirtual(90);

  stop();
}

// ---------------- sensors ----------------
bool detectWall() {
  ultra();
  if (cm < WALL_THRESHOLD) {
    #if SHOW_STATUS
      oled.clear();
      oled.print(F("Wall: "));
      oled.print(cm);
      oled.print(F("cm"));
    #endif
    return true;
  }
  return false;
}

bool detectWallSide() {
  proxSensors.read();
  uint8_t sideRight = proxSensors.countsRightWithRightLeds();
  uint8_t sideLeft = proxSensors.countsLeftWithLeftLeds();

  if (sideRight >= 5 || sideLeft >= 5) {
    #if SHOW_STATUS
      oled.clear();
      oled.print(F("Side: L"));
      oled.print(sideLeft);
      oled.print(F("Side: R"));
      oled.print(sideRight);
    #endif
    return true;
  }
  return false;
}

bool detectLine() {
  unsigned int sensorValues[5];
  lineSensors.read(sensorValues, QTR_EMITTERS_ON);

  if (sensorValues[1] > LINE_UPPER_THRESHOLD &&
      sensorValues[2] > LINE_UPPER_THRESHOLD) {
    #if SHOW_STATUS
      oled.clear();
      oled.print(F("Line Detected!"));
    #endif
    return true;
  }
  return false;
}

double getDistance() {
  int countsL = encoders.getCountsLeft();
  int countsR = encoders.getCountsRight();
  float distanceL = countsL / 909.0 * WHEEL_CIRC;
  float distanceR = countsR / 909.0 * WHEEL_CIRC;
  return (distanceL + distanceR) / 2.0;
}

void resetEncoders() {
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
}

void ultra() {
  long duration;

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout

  if (duration == 0) {
    cm = 999;
    Serial.println("Sensor timeout");
    return;
  }
  cm = microsecondsToCentimeters(duration);

}

long microsecondsToCentimeters(long microseconds) {
  return microseconds / 29 / 2;
}

// ---------------- gyro functions ----------------
float getTurnAngleInDegrees() {
  turnSensorUpdate();
  float angle = ((float)(turnAngle >> 16));
  while (angle < 0) angle += 360.0f;
  while (angle >= 360.0f) angle -= 360.0f;
  return angle;
}

// ---------------- main loop ----------------
void loop() {
bool wall = ENABLE_WALLS && detectWall();
bool line = ENABLE_LINES && detectLine();

  if (line) {
    stop();
    #if SHOW_STATUS
    oled.clear();
    oled.print(F("Line Det"));
    #endif
    resetEncoders();
    navigateLine();
    Counter++;
  } else if (wall) {
    stop();
    #if SHOW_STATUS
    oled.clear();
    oled.print(F("Wall:"));
    #endif
    unsigned long startWait = millis();
    while (millis() - startWait < 30000) {  // 30 seconds total
    turnSensorUpdate();  // Keep gyro updated
    if (!detectWall()) return;
    if ((millis() - startWait) % 5000 < 10) {  // Buzz every 5 sec
        buzzer.playFrequency(440, 200, 15);
    }
}
    navigateObstacle();
    return;
  } else {
    straight(); } }
