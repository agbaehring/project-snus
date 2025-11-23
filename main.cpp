#include <Arduino.h>
#include <Wire.h>
#include <Zumo32U4.h>
#include <config>

// --- Zumo objects ---
Zumo32U4Encoders encoders;
Zumo32U4ProximitySensors proxSensors;
Zumo32U4LineSensors lineSensors;
Zumo32U4IMU imu;
Zumo32U4OLED oled;
Zumo32U4Buzzer buzzer;
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonA buttonB;
Zumo32U4ButtonA buttonC;
Zumo32U4Motors motors;

// --- Function prototypes ---
void turnSensorSetup();
void turnSensorReset();
void turnSensorUpdate();
uint32_t getTurnAngleInDegrees();
void stop();
void forward();
void turnByAngle();
void turnByAngle(float angle);
bool detectLine();
bool detectWall();
float getDistance();
void resetEncoders();
void turnRight();
void turnLeft();
float wheelCirc = 10.0;
void navigateObstacle();
void driveByAngle();
int Counter = 0;

// --- Variables for gyro ---
uint32_t turnAngle = 0;
int16_t turnRate;
int16_t gyroOffset;
uint16_t gyroLastUpdate = 0;

void setup() {
  proxSensors.initThreeSensors();
  lineSensors.initFiveSensors();
  Serial.begin(9600);
  turnSensorSetup();

  oled.clear();
  oled.print(F("Prox cal"));

  // --- Calibrate line sensors ---
  for (int i = 0; i < 100; i++) {
    lineSensors.calibrate();
    oled.clear();
    oled.print(F("Line cal"));
    delay(20);
  }

  oled.clear();
  oled.print(F("Cal done!"));
  delay(00);

  oled.clear();
  oled.print(F("Ready!"));
  delay(500);
}

void (*actions[2])() = { turnRight, turnLeft };
int actionIndex = 0;

void loop() {
  bool wall = ENABLE_WALLS && detectWall();
  bool line = ENABLE_LINES && detectLine();

  if (line) {
    stop();
    delay(STOP_DELAY);
     actions[actionIndex]();   // <-- perform alternating turn
    actionIndex = 1 - actionIndex;   // <-- flip index
    Counter++;

  } else if (wall) {
  stop();

  // Check 6 times, each 5 seconds apart, for a total of 30 seconds.
  const int checks = 6;
  const unsigned long waitTime = 5000; // 5 seconds
  
  for (int i = 0; i < checks; i++) {
    if (!detectWall()){
      // Obstacle is gone → return
      return;
    }
    
    // Obstacle still present → wait and check again
    if (detectWall()) {
      oled.clear();
      oled.print(F("Wall"));
    }

    for (int i = 0; i > checks; i++) {
      navigateObstacle();
      return;  
    
    }
    

    delay(waitTime);
  }

  // If still blocked after 5 checks → perform avoidance turn
  stop();
  delay(300);
  
  actions[actionIndex](); // turnLeft/turnRight alternating
  actionIndex = 1 - actionIndex;
} else if (getDistance() > 10.0) {
    


} else {
    forward();
  }

  delay(50);
}


/*---------------------------------------------------
-----------------MOVEMENT FUNCTIONS------------------
---------------------------------------------------*/
void stop() {
  motors.setSpeeds(0, 0);
}

void forward() {
  motors.setSpeeds(BASE_SPEED, BASE_SPEED);
}

void turnRight() {
  motors.setSpeeds(-BASE_SPEED, -BASE_SPEED);
  delay(100);
  turnByAngle(TURN_ANGLE);
  delay(100);
  motors.setSpeeds(BASE_SPEED, BASE_SPEED);
  delay(300);
  turnByAngle(TURN_ANGLE);
}

void turnLeft() {
  motors.setSpeeds(-BASE_SPEED, -BASE_SPEED);
  delay(100);
  turnByAngle(-TURN_ANGLE);
  delay(100);
  motors.setSpeeds(BASE_SPEED, BASE_SPEED);
  delay(300);
  turnByAngle(-TURN_ANGLE);
}

void navigateObstacel() { //ignore wall while driving 25 cm forward, then move around.
  resetEncoders();

  int ObstacleSpeed = BASE_SPEED / 2;  // 50% speed
  motors.setSpeeds(ObstacleSpeed, ObstacleSpeed);

  while (getDistance() < 25.0) {

    if (detectLine()) {
      stop();
      return;
    }
        delay(20); 
  }



  stop();
}

// TURN FUNCTION
void turnByAngle(float angle) {
  // Reset gyro to start measuring turn
  turnSensorReset();
  float startAngle = getTurnAngleInDegrees();
  float targetAngle = fmod(startAngle + angle, 360.0);
  if (targetAngle < 0) targetAngle += 360.0;

  while (true) {
    float currentAngle = getTurnAngleInDegrees();
    float diff = targetAngle - currentAngle;

    // Normalize difference to [-180,180] to avoid wrap issues
    if (diff > 180) diff -= 360;
    else if (diff < -180) diff += 360;

    if (fabs(diff) < 3) break; // tolerance

    if (diff < 0) motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
    else motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
  }

  stop();
}

void driveByAngle() {


}

/*------------------------------------------------------
---------------------SENSOR FUNCTIONS-------------------
------------------------------------------------------*/ 
bool detectWall() {
  proxSensors.read();
  uint8_t frontRight = proxSensors.countsFrontWithRightLeds();
  uint8_t frontLeft = proxSensors.countsFrontWithLeftLeds();

  if (frontRight > WALL_THRESHOLD || frontLeft > WALL_THRESHOLD) {
    if (SHOW_STATUS) {
      oled.clear();
      oled.print(F("Wall Detected!"));
    }
    return true;
  }
  return false;
}

bool detectLine() {
  unsigned int sensorValues[5];
  lineSensors.read(sensorValues, QTR_EMITTERS_ON);

  // Detect black line across center sensors
  if (sensorValues[1] > LINE_UPPER_THRESHOLD &&
      sensorValues[2] > LINE_UPPER_THRESHOLD &&
      sensorValues[3] > LINE_UPPER_THRESHOLD) {

    if (SHOW_STATUS) {
      oled.clear();
      oled.print(F("Line Detected!"));
    }
    return true;
  }
  return false;


// Detect RED line across center sensors
  if (sensorValues[1] < LINE_LOWER_THRESHOLD &&
      sensorValues[2] < LINE_LOWER_THRESHOLD &&
      sensorValues[3] < LINE_LOWER_THRESHOLD) {

    if (SHOW_STATUS) {
      oled.clear();
      oled.print(F("Line Detected!"));
    }
    return true;
  }
  return false;
}

void turnSensorSetup() {
  Wire.begin();
  imu.init();
  imu.enableDefault();
  imu.configureForTurnSensing();

  oled.clear();
  oled.print(F("Gyro Cal..."));
  ledYellow(1);
  delay(500);

  int32_t total = 0;
  for (uint16_t i = 0; i < 1024; i++) {
    while (!imu.gyroDataReady()) {}
    imu.readGyro();
    total += imu.g.z;
  }
  gyroOffset = total / 1024;
  ledYellow(0);
  turnSensorReset();
  oled.clear();
}

void turnSensorReset() {
  gyroLastUpdate = micros();
  turnAngle = 0;
}

float getDistance() {
  int countsL = encoders.getCountsLeft();
  int countsR = encoders.getCountsRight();
  float distanceL = countsL / 909.0 * WHEEL_CIRC;
  float distanceR = countsR / 909.0 * WHEEL_CIRC;
  return (distanceL + distanceR) / 2;
}

void resetEncoders() {
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
}

void turnSensorUpdate() {
  imu.readGyro();
  turnRate = imu.g.z - gyroOffset;
  uint16_t m = micros();
  uint16_t dt = m - gyroLastUpdate;
  gyroLastUpdate = m;
  int32_t d = (int64_t)turnRate * dt;
  turnAngle += (int64_t)d * 14680064 / 17578125;
}

uint32_t getTurnAngleInDegrees() {
  turnSensorUpdate();
  return (((uint32_t)turnAngle >> 16) * 360) >> 16;
}

bool detectObstacleSide() {
  proxSensors.read();
  uint8_t leftSide  = proxSensors.countsLeftWithLeftLeds();
  uint8_t rightSide = proxSensors.countsRightWithRightLeds();

  return (leftSide > SIDE_THRESHOLD || rightSide > SIDE_THRESHOLD);
}


