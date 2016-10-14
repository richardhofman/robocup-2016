/*
 * File: LIDARController.cpp
 * LIDARController class, developed for the RoboCup control program.
 * Authors: Richard Hofman, Matthew Poole, Daniel Catto [GROUP 9]
 * 15.10.2016
 */

#include "LIDARController.h"
#include "Wire.h"

//#define SHOW_FCTN_NAMES // Uncomment this to print function names during execution.

/*
 * Constructor function for the LIDAR control class.
*/
LIDARController::LIDARController()
{
    // Herkulex smart servo address
    sweepMotor = 0xfd;

    // VL53L0X sensor I2C addresses
    lidarTopAddr = 0x30;
    lidarBottomAddr = 0x31;

    lidarTopPin = LIDAR_TOP_PIN;
    lidarBottomPin = LIDAR_BOTTOM_PIN;
       
    // Instance variable initialisations
    topRange = 8192;
    bottomRange = 8192;
    obstacleAheadDistance = -1;
    weightAheadDistance = -1;
    weightAngle = -360.0;
   
    sweepDirection = 0;
    mode = CENTERED;

    targetAngle[0] = LIDAR_LEFT_ANGLE;
    targetAngle[1] = LIDAR_RIGHT_ANGLE;
}

/*
 * Initialise the sweep servo and the three LIDAR sensors that provide range data.
 */
void LIDARController::init(void) {
    #ifdef SHOW_FCTN_NAMES
    Serial.println("LIDARController::init()");
    #endif

    // Reset fault counters
    sensorFault[TOP] = 0;
    sensorFault[BOTTOM] = 0;

    // Only start I2C if it isn't already running (allows for re-initialisation).
    if (TWCR == 0) {
      Wire.begin();
    }

    // Instantiate LIDAR control instances.
    lidarTop = VL53L0X();
    lidarBottom = VL53L0X();

    // Initialise and address VL53L0Xs in sequence
    
    // Pull XSHUT pins LOW (shut down sensors if active)
    pinMode(lidarTopPin, OUTPUT);
    pinMode(lidarBottomPin, OUTPUT);
    digitalWrite(lidarTopPin, LOW);
    digitalWrite(lidarBottomPin, LOW);
   
    Serial.println("Initialising top LIDAR...");
    pinMode(lidarTopPin, INPUT); // carrier board will pull XSHUT to Vdd (2.8v)
    delay(200);
    lidarTop.init();
    lidarTop.setAddress(lidarTopAddr);
    lidarTop.setTimeout(500);
    Serial.println("Initialised top LIDAR");

    Serial.println("Initialising bottom LIDAR...");
    pinMode(lidarBottomPin, INPUT); // carrier board will pull XSHUT to Vdd (2.8v)
    delay(200);
    lidarBottom.init();
    lidarBottom.setAddress(lidarBottomAddr);
    lidarBottom.setTimeout(500);
    Serial.println("Initialised bottom LIDAR");

    // Set up VL53L0X sensors with 33ms timing budget.
    lidarBottom.setMeasurementTimingBudget(33000);
    lidarTop.setMeasurementTimingBudget(33000);
    
    Serial.println("Finished LIDAR initialisation.");
}

/*
 * Sweep the sensors from side to side, scanning for weights.
 */
void LIDARController::setSweeping(void) {
  if (mode != SWEEPING) {
    mode = SWEEPING;
    sweepDirection = 0; // 0 = left, 1 = right
  }
}

/*
 * Center the LIDAR sensors for direct-ahead obstacle detection.
 */
void LIDARController::setCenter(void) {
  mode = CENTERED;
  Herkulex.moveOneAngle(sweepMotor, 0.0, 1000, 1); // center sensor column
}

/*
 * Called at rate of main loop - change sweep direction if we have completed the current sweep.
 */
void LIDARController::stepSweep(void) {
  #ifdef SHOW_FCTN_NAMES
  Serial.println("stepSweep()");
  #endif
  
  // Update current sweep data
  updateLidarData();
  
  if (mode == CENTERED) {
    return;
  }

  // Update Herkulex servo
  if (sweepDirection == 1) {
    Herkulex.moveOneAngle(sweepMotor, -130, 750, LED_BLUE);
    if (Herkulex.getAngle(sweepMotor) < targetAngle[1] + 2){
      sweepDirection = 0;
    }

  } else if (sweepDirection == 0) {
    Herkulex.moveOneAngle(sweepMotor, 130, 750, LED_GREEN); //move motor backward
    if (Herkulex.getAngle(sweepMotor) > targetAngle[0] - 2){
      sweepDirection = 1;
    }
  }
}

/*
 * Update class-local values for each LIDAR range (in mm).
 * Record failure count in sensorFault[SENSOR_POSITION] if timeout occurs.
 */
void LIDARController::updateLidarData(void) {
  #ifdef SHOW_FCTN_NAMES
  Serial.println("updateLidarData()");
  #endif
  
  topRange = lidarTop.readRangeSingleMillimeters();
  
  //Rudimentary fault detection
  if (lidarTop.timeoutOccurred() || topRange < 0 || topRange > 65000) {
    sensorFault[TOP] += 1;
    topRange = 65535;
  } else {
    sensorFault[TOP] = 0;
  }

  bottomRange = lidarBottom.readRangeSingleMillimeters();
  
  // Rudimentary fault detection
  if (lidarBottom.timeoutOccurred() || bottomRange < 0 || bottomRange > 65000) {
    sensorFault[BOTTOM] += 1;
    bottomRange = 65535;
  } else {
    sensorFault[BOTTOM] = 0;
  }

  processLidarData(Herkulex.getAngle(sweepMotor));
}

void LIDARController::printTopBottom(void) {
  Serial.print("\t Top: "); Serial.print(topRange); Serial.print(" Bottom: "); Serial.print(bottomRange); Serial.print(" Angle: "); Serial.print(Herkulex.getAngle(sweepMotor));
}

/*
 * Processes the three most recent LIDAR data points:
 * If there's a return on bottom but not top, or if there's a closer return
 * on the bottom, a weight is detected and angle saved.
 * If there's a closer/only return on top, an obstacle is detected if it's within an
 * 80 degree arc of the robot's front.
 */
void LIDARController::processLidarData(float angle) {
  #ifdef SHOW_FCTN_NAMES
  Serial.println("processLidarData()");
  #endif
  
  // Only process readings within valid angle range (accounts for servo overshoot)
  if (angle < LIDAR_LEFT_ANGLE && angle > LIDAR_RIGHT_ANGLE) {
    // If there's a weight within reliable detection range, update values.
    if (bottomRange < LIDAR_DETECT_DIST && topRange - bottomRange > LIDAR_WEIGHT_DIFF) {
      weightAheadDistance = bottomRange;
      weightAngle = angle;
    } else {
      weightAheadDistance = -1;
      weightAngle = -360.0;
    }
  } else {
    weightAheadDistance = -1;
    weightAngle = -360.0;
  }
  
  // If an obstacle exists within +/- 50 degrees of robot's front, set obstacleAheadDistance.
  if (topRange < LIDAR_DETECT_DIST && abs(topRange - bottomRange) < LIDAR_WEIGHT_DIFF
       && abs(angle) < 50) {
    obstacleAheadDistance = topRange;
  }
}

/*
 * Returns 1 if there have been repeated faults with one or more sensors,
 * 0 otherwise.
 */
int LIDARController::getSensorFault(void){
  #ifdef SHOW_FCTN_NAMES
  Serial.println("getSensorFault()");
  #endif
  
  return (sensorFault[BOTTOM] > 3 || sensorFault[TOP] > 3);
}

/*
 * Resets the derived values from the previous sweep,
 * ready for a new dataset (sweep).
 */
void LIDARController::resetDetectionData(void) {
  #ifdef SHOW_FCTN_NAMES
  Serial.println("resetDetectionData()");
  #endif
  
  obstacleAheadDistance = -1;
  weightAngle = -360.0;
  weightAheadDistance = -1;
}

/*
 * Return value for distance from obstacle ahead (in mm)
 * Returns -1 if no obstacle within detection range,
 *  or if sensor data is stale.
 */
int LIDARController::getObstacleAheadDistance(void){
  #ifdef SHOW_FCTN_NAMES
  Serial.println("getObstacleAheadDistance()");
  #endif
    
  int distance = obstacleAheadDistance;
  obstacleAheadDistance = -1;
  return distance;
}

/*
 * Returns distance in mm for weight ahead of LIDAR array.
 */
int LIDARController::getWeightAheadDistance(void) {
  return weightAheadDistance;
}

/*
 * Return value for angle to nearest detected weight (in degrees)
 * Returns -360 if no weight within detection range,
 *  or if sensor data is stale.
 */
float LIDARController::getWeightAngle(void){
  #ifdef SHOW_FCTN_NAMES
  Serial.println("getWeightAngle()");
  #endif
  
  float angle = weightAngle;
  
  weightAngle = -360.0;
  return angle;
}




