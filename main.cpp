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
long getDistance_mm();
void resetEncoders();
void navigateObstacle();
bool detectWallSide();
long microsecondsToCentimeters(long microseconds);
void ultra();

// Ultrasonic pins
const int trigPin = 22;
const int echoPin = 18;

// Globals
int cm;
int Counter = 0;
unsigned long lastUltraTime = 0;
const unsigned long ULTRA_INTERVAL = 60; // minimum 60 ms between readings
int lastValidCm = 999;                   // cached valid distance


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
  oled.clear();
}

// ---------------- movement ----------------
void stop() {
  motors.setSpeeds(0, 0);
}
float headingOffset = 0;  // global variable

// Call this to turn “virtually” 90 degrees
void turnVirtual(float angle)
{
    // Calculate target heading by adding desired angle to current heading
    float targetHeading = headingOffset + angle;
    
    // Normalize target to [0, 360)
    while (targetHeading < 0) targetHeading += 360;
    while (targetHeading >= 360) targetHeading -= 360;
    
    float error = 999;
    unsigned long startTime = millis();
    
    while (abs(error) > 0.5 && (millis() - startTime < 3000))  // Added timeout
    {
        turnSensorUpdate();
        
        // Current heading
        float currentHeading = ((int32_t)turnAngle >> 16) * 360.0 / 65536.0;
        
        // Error relative to target
        error = currentHeading - targetHeading;
        
        // Normalize error to [-180, 180]
        if (error > 180) error -= 360;
        if (error < -180) error += 360;
        
        int32_t turnSpeed = -(error * 56) - turnRate / 20;
        turnSpeed = constrain(turnSpeed, -BASE_SPEED, BASE_SPEED);
        motors.setSpeeds(-turnSpeed, turnSpeed);
    }
    stop();
    
    // Update heading offset to actual final position
    headingOffset = ((int32_t)turnAngle >> 16) * 360.0 / 65536.0;
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

bool navLeftObstacle() { //Depends on Counter value, decides direction.
    if ((Counter >= 0 && Counter < 7) && (Counter % 2 == 0)) return false;
    if ((Counter >= 0 && Counter < 7) && (Counter % 2 != 0)) return true;
    if ((Counter >= 8 && Counter < 15) && (Counter % 2 != 0)) return false;
    if ((Counter >= 8 && Counter < 15) && (Counter % 2 == 0)) return true;
    if ((Counter >= 16 && Counter < 23) && (Counter % 2 == 0)) return false;
    if ((Counter >= 16 && Counter < 23) && (Counter % 2 != 0)) return true;
    if ((Counter >= 24 && Counter < 30) && (Counter % 2 != 0)) return false;
    if ((Counter >= 24 && Counter < 30) && (Counter % 2 == 0)) return true;
   
}

bool navLeftLine() { //Depends on Counter value, decides direction.
    if ((Counter >= 0 && Counter < 15) && (Counter % 2 == 0)) return false;
    if ((Counter >= 16 && Counter < 30) && (Counter % 2 != 0)) return false;
    return true;
}

// ---------------- line navigation ----------------
void navigateLine() {
  if (navLeftLine()) turnVirtual(-TURN_ANGLE);
  else turnVirtual(TURN_ANGLE);
    resetEncoders();
    while (true) {
    long d = getDistance_mm();
    if (d >= 70) break;
    straight();}
    stop();
    if (navLeftLine()) turnVirtual(-TURN_ANGLE);
    else turnVirtual(TURN_ANGLE);}

// ---------------- obstacle navigation ----------------
void navigateObstacle() {

  // Approach obstacle a bit
  while (true) {
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
      stop();
      break;
    }

    if (detectLine()) {
      stop();
      return;
    }

    straight();
  }

  stop();
  if (navLeftObstacle()) turnVirtual(-90);
  else turnVirtual(90);

  resetEncoders();
  while (detectWallSide()) {
    straight();
  }
  long firstLegDistance = getDistance_mm();

  resetEncoders();
  while(getDistance_mm() < 100){ // added to ensure it moves forward after losing the wall
    straight();}

  long secondLegDistance = getDistance_mm();

  if (navLeftObstacle()) turnVirtual(90);
  else turnVirtual(-90);

  resetEncoders();
  while(getDistance_mm() < 100){ // added to ensure it moves forward to detect wall
    straight();}
resetEncoders();

while (getDistance_mm() < 200) {   // follow wall for 20 cm
    straight();

    if (!detectWallSide()) break; 
    if (detectLine()) {
        stop();
        return;
    }
}
  resetEncoders();
  while(getDistance_mm() < 100){ // added to ensure it moves forward after losing the wall
    straight();}


  if (navLeftObstacle()) turnVirtual(90);
  else turnVirtual(-90);

  long totalDistance = firstLegDistance + secondLegDistance;

  resetEncoders();
  while (getDistance_mm() < totalDistance) {
    straight();
  }

  if (navLeftObstacle()) turnVirtual(-90);
  else turnVirtual(90);

  stop();
}

// ---------------- sensors ----------------
bool detectWall() {
    ultra();  // trigger a new reading if possible
    if (lastValidCm < WALL_THRESHOLD) {
        #if SHOW_STATUS
          oled.clear();
          oled.print(F("Wall: "));
          oled.print(lastValidCm);
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

  if (sideRight >= SIDE_THRESHOLD || sideLeft >= SIDE_THRESHOLD) {
    #if SHOW_STATUS
      oled.clear();
      oled.print(F("L"));
      oled.print(sideLeft);
      oled.print(F("R"));
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

long getDistance_mm(){
  long countsL = encoders.getCountsLeft();
  long countsR = encoders.getCountsRight();
  long distance_mm_L = countsL * WHEEL_CIRC_mm / 909;
  long distance_mm_R = countsR * WHEEL_CIRC_mm / 909;
  return (distance_mm_L + distance_mm_R) / 2;
}


void resetEncoders() {
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
}

void ultra() {
    unsigned long now = millis();
    if (now - lastUltraTime < ULTRA_INTERVAL) return;  // rate limit
    lastUltraTime = now;

    long duration;
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout
    if (duration == 0) {
        cm = 999; // failed reading
        return;
    }

    cm = duration / 29 / 2; // convert to cm
    lastValidCm = cm;        // cache valid reading
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
    while (millis() - startWait < 30000) {  // Should be 30000 for 30 seconds total
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
