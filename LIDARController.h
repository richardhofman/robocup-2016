/*
 * File: LIDARController.h
 * LIDARController class, developed for the RoboCup control program.
 * Authors: Richard Hofman, Matthew Poole, Daniel Catto [GROUP 9]
 * 15.10.2016
 */

#ifndef Lidar_h
#define Lidar_h
#define LIDAR_LEFT_ANGLE 50.0
#define LIDAR_RIGHT_ANGLE -50.0
#define LIDAR_DETECT_DIST 700 // 25mm/tan (3deg/2) = ??
#define LIDAR_NOISE_MARGIN 20 // +/-20mm
#define LIDAR_WEIGHT_DIFF 50 // 50mm difference
#define LIDAR_TOP_PIN 30
#define LIDAR_BOTTOM_PIN 31

// Include Herkulex smart servo API
#include "Herkulex.h"

// Include Pololu Vl53L0X sensor API
#include <VL53L0X.h>

typedef enum {SWEEPING, CENTERED} LIDARMode;
enum {TOP, BOTTOM};

// library interface description
class LIDARController
{
  // user-accessible "public" interface
  public:
    // Lifecycle and config methods
    LIDARController(void);
    void init(void);
    void setSweeping(void);
    void setCenter(void);

    // Update tasks
    void stepSweep(void);
    void updateLidarData(void);

    // Accessors
    int   getWallDistance(void);
    int   getObstacleAheadDistance(void);
    float getWeightAngle(void);
    int   getSensorFault(void);
    int   getWeightAheadDistance(void);
    void  printTopBottom(void);
    

  // library-accessible "private" interface
  private:
    // LIDAR sensor operating mode
    LIDARMode mode; // SWEEPING, CENTERED

    // Hardware addresses and status variables
    int   sensorFault[2];
    float currentAngle;
    int   sweepMotor; // motor addresses
    int   lidarTopAddr, lidarBottomAddr; // VL53L0X addresses
    int   lidarTopPin, lidarBottomPin;
    
    VL53L0X lidarTop, lidarBottom;

    // Data and control variables
    int   topRange, bottomRange;
    int   obstacleAheadDistance;
    int   weightAheadDistance;
    int   sweepDirection; // 0 for left, 1 for right, always alternating
    float weightAngle;

    float targetAngle[2];

    // Private helper methods
    void processLidarData(float angle);
    void resetDetectionData(void);
};

#endif



