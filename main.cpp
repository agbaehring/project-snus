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
void navigateObstacle();
void driveByAngle();
bool detectWallSide();
long microsecondsToInches(long microseconds);
long microsecondsToCentimeters(long microseconds);
void ultra ();

int inches;
int cm;
int i = 0;
bool navigateWallLeft;
const int trigPin = 22;  
const int echoPin = 18;
int Counter = 0;
float wheelCirc = 10.0;

// --- Variables for gyro ---
uint32_t turnAngle = 0;
int16_t turnRate;
int16_t gyroOffset;
uint16_t gyroLastUpdate = 0;

void setup() {
  proxSensors.initThreeSensors();
  lineSensors.initThreeSensors();
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

  pinMode(trigPin, OUTPUT);  
  pinMode(echoPin, INPUT);   
  
  oled.clear();
  oled.print(F("Ready!"));
  delay(500);
}

void (*actions[2])() = { turnRight, turnLeft };
int actionIndex = 0;

void loop() {
  proxSensors.read();
  uint8_t sideRight = proxSensors.countsRightWithRightLeds();
  uint8_t sideLeft = proxSensors.countsLeftWithLeftLeds();
  
  oled.clear();
  oled.print(F("L:"));
  oled.print(sideLeft);
  oled.print(F(" R:"));
  oled.print(sideRight);
  delay(100);

  bool wall = ENABLE_WALLS && detectWall();
  bool line = ENABLE_LINES && detectLine();

  if (line) {
    stop();
    resetEncoders();
    motors.setSpeeds(-BASE_SPEED, -BASE_SPEED);
    for (int i = 0; i < 50; i++) {  // Try 50 iterations max
    getDistance() ;  // Update distance
            
    if (getDistance() <= 9) {
          
      delay(STOP_DELAY);
      actions[actionIndex]();   // <-- perform alternating turn
      actionIndex = 1 - actionIndex;   // <-- flip turn direction
      Counter++; } }

  } else if (wall) {
    stop();
    oled.clear();
    oled.print(F("Wall"));
    // Check 6 times, each 5 seconds apart, for a total of 30 seconds.
    const int checks = 1;
    const unsigned long waitTime = 5000; // 5 seconds
  
    // --- WAIT UP TO 30 SECONDS ---
    for (int i = 0; i < checks; i++) {
    if (!detectWall()) {
      return;   // wall disappeared → exit loop and resume
    }
    delay(waitTime);
    }

    // --- WALL STILL BLOCKING → AVOID IT ---
    navigateObstacle();
    return;
   } else  {
    oled.clear();
    oled.print(F("Forward"));
    forward();
  }
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
  delay(100); // Change to encoder based movement later
  turnByAngle(TURN_ANGLE);
  delay(100);
  motors.setSpeeds(BASE_SPEED, BASE_SPEED); 
  delay(400); 
  turnByAngle(TURN_ANGLE);
}

void turnLeft() {
  motors.setSpeeds(-BASE_SPEED, -BASE_SPEED);
  delay(100); // Change to encoder based movement later
  turnByAngle(-TURN_ANGLE);
  delay(100);
  motors.setSpeeds(BASE_SPEED, BASE_SPEED);
  delay(400);
  turnByAngle(-TURN_ANGLE);
}

void navigateObstacle() {   // AI solution for debugging
  oled.clear();
  oled.print(F("Approach"));
  
  // Step 1: Drive closer to the obstacle
  for (int i = 0; i < 50; i++) {  // Try 50 iterations max
    ultra();  // Update distance
    
    oled.clear();
    oled.print(F("Dist: "));
    oled.print(cm);
    
    if (cm <= 9) {
      oled.clear();
      oled.print(F("Close!"));
      break;  // Close enough
    }
    
    if (detectLine()) {
      stop();
      return;
    }
    
    motors.setSpeeds(BASE_SPEED / 2, BASE_SPEED / 2);
    delay(100);  // Small delay between checks
  }
  
  stop();
  delay(500);
  
  bool navigateWallLeft = true;
  
  if ((Counter >= 0 && Counter < 6) && (Counter % 2 == 0)) {
    navigateWallLeft = false;
  } 
  else if ((Counter >= 9 && Counter < 15) && (Counter % 2 != 0)) {
    navigateWallLeft = false;
  }

  if (navigateWallLeft) {
    turnByAngle(-TURN_ANGLE);
  } else {
    turnByAngle(TURN_ANGLE);
  }
  
  resetEncoders();
    while (detectWallSide()) {
        forward();
    }
    float firstLegDistance = getDistance();  // record distance
    // Second 90° turn to go around
    if (navigateWallLeft) {
         turnByAngle(TURN_ANGLE);      // opposite direction
    } else {
         turnByAngle(-TURN_ANGLE);
    }
       

    // --- Move until obstacle disappears again ---
    while (detectWallSide()) {
        forward();
         if (detectLine()) {
         stop();
         return;
         }
    }
    forward();
    delay(250);
    // --- Final 90° turn back toward original path ---
    if (navigateWallLeft) {
         turnByAngle(TURN_ANGLE);
    } else {
         turnByAngle(-TURN_ANGLE);
    }

    // --- Use recorded distance to get back on track ---
    resetEncoders();
    while (getDistance() < firstLegDistance) {
        forward();
    }
    
    // --- Final turn to re-align with original heading ---
    if (navigateWallLeft) {
         turnByAngle(-TURN_ANGLE);
    } else {
         turnByAngle(TURN_ANGLE);
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

    if (diff < 0) motors.setSpeeds(TURN_SPEED, TURN_SPEED / 30);
    else motors.setSpeeds(TURN_SPEED / 30 , TURN_SPEED);
  }

  stop();
}

/*------------------------------------------------------
---------------------SENSOR FUNCTIONS-------------------
------------------------------------------------------*/ 
bool detectWall() {
  ultra();  // Update distance readings
  
  // Check if obstacle is within threshold distance
  if (cm < WALL_THRESHOLD) {   
    if (SHOW_STATUS) {
      oled.clear();
      oled.print(F("Wall: "));
      oled.print(cm);
      oled.print("cm");
    }
    return true;
  }
  return false;
}

bool detectWallSide() {
  proxSensors.read();
  uint8_t sideRight = proxSensors.countsRightWithRightLeds();
  uint8_t sideLeft = proxSensors.countsLeftWithLeftLeds();

  if (sideRight >= 5 || sideLeft >= 5) {  // Detect if 4 or higher
    if (SHOW_STATUS) {
      oled.clear();
      oled.print(F("Side: L"));
      oled.print(sideLeft);
      oled.print(F(" R"));
      oled.print(sideRight);
    }
    return true;
  }
  return false;
}

bool detectLine() {
  unsigned int sensorValues[5];
  lineSensors.read(sensorValues, QTR_EMITTERS_ON);

  // Detect black line across center sensors
  if (/*sensorValues[0] > LINE_UPPER_THRESHOLD &&*/
      sensorValues[1] > LINE_UPPER_THRESHOLD &&
      sensorValues[2] > LINE_UPPER_THRESHOLD) {

    if (SHOW_STATUS) {
      oled.clear();
      oled.print(F("Line Detected!"));
    }
    return true;
  } return false; }

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

void ultra() {
  long duration;
  
  pinMode(trigPin, OUTPUT);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout
  
  // Handle failed reading
  if (duration == 0) {
    cm = 999;  // No obstacle detected
    inches = 999;
    Serial.println("Sensor timeout");
    return;
  }

  // Update global variables
  inches = microsecondsToInches(duration);
  cm = microsecondsToCentimeters(duration);
  
  Serial.print(cm); 
  Serial.println("cm");
}

long microsecondsToInches(long microseconds)
{
  // According to Parallax's datasheet for the PING))), there are
  // 73.746 microseconds per inch (i.e. sound travels at 1130 feet per
  // second).  This gives the distance travelled by the ping, outbound
  // and return, so we divide by 2 to get the distance of the obstacle.
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}



