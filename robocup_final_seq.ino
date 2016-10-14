
/*
 * Main sketch file for the final RoboCup 2016 control program.
 * Authors: Richard Hofman, Matthew Poole, Daniel Catto [GROUP 9]
 * 15.10.2016
 */

// Include provided libraries that are required.
#include <Servo.h>
#include "Herkulex.h"
#include <avr/wdt.h>

// Include custom classes and types.
#include "robocup_final_seq.h"
#include "PickupController.h"
#include "LIDARController.h"

// Enable this for debug output over Serial1
#define RB_DEBUG   // Enables Serial output for debugging.
#define IMMOBILISE // Disables the main motors for on-desk debugging.

/* Declare and instantiate global state variables and subsystem abstractions. */
PickupController pickup;
LIDARController sensors;

RCState systemState;        // -- current state of global state machine.
Manoeuvre currentManoeuvre; // -- current motion controller state.
float targetAngle;          // -- target angle for current manoeuvre.
int manoeuvreEnded;         // -- true if most recent manoeuvre has ended.
unsigned long stateTimer;   // -- wall-clock time reference for nonblocking timed state changes
int weightDetectCounter;    // -- number of obstacle-free readings required from sensor to confirm weight.
int targetWeightAngle;      // -- target angle to which we are turning to orient towards weight.

// This should not have been necessary. [angry face]
void softReset() {
  #ifdef RB_DEBUG
  Serial.println("I2C BUS FAILURE.");
  Serial.println("Attempting to soft-reinitialise the LIDAR sensors/I2C bus...");
  #endif
  sensors.init();
}

// Main motor servo objects
Servo rightMotor;
Servo leftMotor;

/*
 * Does what it says on the tin.
 */
void setup() 
{
  /* Drive ultrasonic trigger pins at 120Hz using PWM (ATmega2560 timer 4).
   * Used to generate interference on other teams' US sensors.
   * NOT used in final due to Servo control requiring all CPU timers,
   * but left in because it was a cool idea.
   */
  /*int prescaler = 4;
  int prescalerBits = 0b111;
  TCCR4B &= ~prescalerBits;
  TCCR4B |= prescaler;
  analogWrite(8, 200);
  analogWrite(6, 200);*/
  // End countermeasures
  
  targetWeightAngle = 0;
  weightDetectCounter = 0;
  stateTimer = 0;
  manoeuvreEnded = 0;

  #ifdef RB_DEBUG
  Serial.begin(9600);                  // initialize serial communication:
  delay(1000);
  Serial.println("Initialised RoboCup control program.') DROP TABLE Competitors;");
  #endif
  
  pinMode(49, OUTPUT);                 // Pin 49 is used to enable IO power
  digitalWrite(49, 1);                 // Enable IO power on main CPU board
  
  // Set initial variable values
  currentManoeuvre = NONE_M;
  targetAngle = 0.0;

  // Setup main drive motors
  #ifndef IMMOBILISE
  rightMotor.attach(3);
  leftMotor.attach(2);
  #endif

  // Setup Herkulex smart servos
  // Initialise Herkulex library
  #ifdef RB_DEBUG
  Serial.println("Initialise Herkulex...");
  #endif
  Herkulex.beginSerial2(115200);
  Herkulex.reboot(0x01); // motorA (pickup)
  Herkulex.reboot(0x02); // motorB (pickup)
  Herkulex.reboot(0xFE); // sweepMotor (LIDAR)
  delay(500);
  Herkulex.initialize();
  delay(200);

  // Initialise pickup subsystem
  #ifdef RB_DEBUG
  Serial.println("Initialise pickup subsystem...");
  #endif
  pickup.init();
  pickup.vertical();

  // Initialise the sensor subsystem
  #ifdef RB_DEBUG
  Serial.println("Initialise LIDAR sensor subsystem...");
  #endif
  
  sensors.init();
  sensors.setSweeping();
  
  systemState = FORWARD_ROAM;
}


/* 
 *  Accepts a steering differential as well as a base speed,
 *  and applies these to the main motors.
 */
void updateMotors(int steer, int driveSpeed) {
  int leftSpeed, rightSpeed;
  
  // Sanity checks
  if (driveSpeed > 700) {
    driveSpeed = 700;
  } else if (driveSpeed < -700) {
    driveSpeed = -700;
  }
  
  driveSpeed = driveSpeed+1500; // adjust for middle/stop torque
  
  leftSpeed = driveSpeed + steer;
  rightSpeed = driveSpeed - steer;
  
  leftMotor.write(leftSpeed);
  rightMotor.write(rightSpeed);
}

/*
 * Very basic implementation of ~=.
 */
int approxFloat (float a, float b, float threshold) {
  return (a - b) < threshold;
}

/*
 * Set the current manoeuvre to execute.
 * The change will not first come into effect until the next call to
 * stepManoeuvre().
 */
void executeManoeuvre(Manoeuvre type, float angle) {
  manoeuvreEnded = 0;
  currentManoeuvre = type;
  targetAngle = angle;
}


/*
 * Specify motion-specific control logic for each manoeuvre state.
 * Due to time constraints, no P[I]D control was used, as it wasn't
 * hugely useful for our purposes.
 */
void stepManoeuvre(void) {
  float angle;
  switch (currentManoeuvre) {
    case NONE_M:
      leftMotor.write(1500);
      rightMotor.write(1500);
      break;
    case FORWARD_M:
      leftMotor.write(1850);
      rightMotor.write(1850);
      break;
    case FORWARD_SLOW_M:
      leftMotor.write(1700);
      rightMotor.write(1700);
      break;
    case TURN_M:
      if (targetAngle < 0) {
        // Clockwise/right
        leftMotor.write(1800);
        rightMotor.write(1200);
      } else {
        // Anticlockwise/left
        leftMotor.write(1200);
        rightMotor.write(1800);
      }
      break;
    case AVOID_TURN_M:
      if (targetAngle < 0) {
        // Clockwise/right
        leftMotor.write(1850);
        rightMotor.write(1150);
      } else {
        // Anticlockwise/left
        leftMotor.write(1150);
        rightMotor.write(1850);
      }
      break;
    case PICKUP_M:
      leftMotor.write(1500);
      rightMotor.write(1500);
      break;
    case REVERSE_M:
      leftMotor.write(1150);
      rightMotor.write(1150);
      break;
  }
}

/*
 * Main logic loop - handles global state transitions.
 */
void loop()
{
  // !! ENABLE THE WATCHDOG TIMER !!
  wdt_enable(WDTO_8S);
  // !! ENABLE THE WATCHDOG TIMER !!

  // Do we need to soft reset?
  if (sensors.getSensorFault()) {
    softReset();
  }

  // Reboot if UART bus fails (there was some deliberation as to whether
  // this should have slammed the robot repeatedly into a wall instead).
  if (Herkulex.stat(0x01) > 0) {
    wdt_reset();
    wdt_enable(WDTO_15MS);
    delay(100);
  }
  
  // Update sensor array and enforce pickup mechanism position
  sensors.stepSweep();
  pickup.vertical();
  
  // Get latest detection parameters.
  int weightAngle = sensors.getWeightAngle();
  int weightDistance = sensors.getWeightAheadDistance();
  int obstacleDistance = sensors.getObstacleAheadDistance();

  // Update motion controller
  stepManoeuvre();  
   
  switch (systemState) {
    case FORWARD_ROAM:
      // Ensure correct sensor configuration and motion state.
      sensors.setSweeping();
      executeManoeuvre(FORWARD_M, 0.0);
      
      // If we've detected a weight, wait for no obstacles to appear for
      // consecutive measurements, BEFORE assuming there's a weight ahead.
      if (weightAngle > -300.0) {
        targetWeightAngle = weightAngle;
        weightDetectCounter = 5;
      }

      // Check the measurement count.
      if (weightDetectCounter > 0) {
        #ifdef RB_DEBUG
        Serial.print("\t Obstacle misses to go: "); Serial.print(weightDetectCounter);
        #endif
        
        if(obstacleDistance > -1) {
          // If there's an obstacle, give up.
          weightDetectCounter = 0;
        } else {
          // If there isn't an obstacle ahead, continue...
          weightDetectCounter--;
        }
      } 
      
      // We made it through four readings - it's probably a weight!
      if (weightDetectCounter == 1) { // If we've had four iterations without seeing an obstacle, it's a weight.
        #ifdef RB_DEBUG
        Serial.print("\t Status: Found weight at: "); Serial.print(targetWeightAngle);
        #endif
        
        systemState = TURN;
      }

      if (obstacleDistance > 0 && obstacleDistance < 300) {
        #ifdef RB_DEBUG
        Serial.print("\t Status: Seen obstacle!");
        sensors.printTopBottom();
        #endif
        
        systemState = AVOID;
      }
      break;
      
    case FORWARD_WEIGHT:
      // Set correct sensor state and manoeuvre.
      sensors.setSweeping();
      executeManoeuvre(FORWARD_SLOW_M, 0.0);
      
      // If we're within pickup distance of weight, do that.
      if (weightDistance < PICKUP_DISTANCE && weightDistance > 0) {
        #ifdef RB_DEBUG
        Serial.print("\t Status: Ready for pickup");
        sensors.printTopBottom();
        #endif
        
        delay(100); // overturn slightly to get past the edge of the weight.
        systemState = PICKUP;
      }
      
      // If we've bumped into an obstacle, abort!
      if (obstacleDistance > 0 && obstacleDistance < 160) {
        #ifdef RB_DEBUG
        Serial.print("\t Status: Obstacle Detected");
        sensors.printTopBottom();
        #endif
        
        systemState = AVOID;
      }
      break;
      
    case PICKUP:
      executeManoeuvre(PICKUP_M, 0.0);
      stepManoeuvre(); // required due to the use of delay()
      
      // !! DISABLE THE WATCHDOG TIMER !!
      wdt_reset();
      // !! DISABLE THE WATCHDOG TIMER !!
      
      // Execute timed pickup manoeuvre.
      // Blocking delay(X) used because weird things happened using millis() :( )
      pickup.lower();
      delay(2000);
      pickup.activateMagnet();
      delay(100);
      pickup.raise();
      delay(2000);
      pickup.deactivateMagnet();
      pickup.vertical();
      
      // Get clear of weight if we missed it, or it's a decoy.
      systemState = AVOID; 

      // !! RE-ENABLE THE WATCHDOG TIMER !!
      wdt_enable(WDTO_8S);
      // !! RE-ENABLE THE WATCHDOG TIMER !!
      break;
      
    case TURN:
      if (currentManoeuvre != TURN_M) {
        // If we're transitioning into this state, do one-time things.
        #ifdef RB_DEBUG
        sensors.printTopBottom();
        Serial.print("\t Status: Turning to: "); Serial.println(targetWeightAngle);
        #endif
        
        // Stop forward motion and center sensors.
        executeManoeuvre(NONE_M, 0.0);
        stepManoeuvre();
        sensors.setCenter();
        
        // Wait 1.5s for sensors and other motion to settle.
        delay(1500);
        
        // Begin turning slowly to seek weight.
        executeManoeuvre(TURN_M, targetWeightAngle);
        targetWeightAngle = 0;
        
        // Start timeout.
        stateTimer = millis();
      } else if (sensors.getWeightAheadDistance() > 0
              && sensors.getWeightAheadDistance() < LIDAR_DETECT_DIST) {
        // If we're lined up with a weight, start moving to collect it.
        #ifdef RB_DEBUG
        Serial.print("\t Status: Lined up");
        sensors.printTopBottom();
        #endif
        
        systemState = FORWARD_WEIGHT;
        sensors.setSweeping();
      } else if (millis() - stateTimer > 15000) {
        //If we've been turning for >15s, give up and turn away from weight.
        systemState = AVOID;
      }
      break;
      
    case AVOID:
      if (currentManoeuvre != REVERSE_M && currentManoeuvre != AVOID_TURN_M) {
        stateTimer = millis();
      }
      
      // Reverse for 1.5s
      if (millis() - stateTimer < 1500 && currentManoeuvre != REVERSE_M) {
        sensors.printTopBottom();
        delay(500);
        executeManoeuvre(REVERSE_M, 30.0);
      }

      // Turn a random direction for remaining time (3.0s)
      if (millis() - stateTimer > 1500 && currentManoeuvre != AVOID_TURN_M) {
        executeManoeuvre(AVOID_TURN_M, 1.0*random(-1,2));
      }

      // Return to roaming after completion of avoidance manoeuvres.
      if (millis() - stateTimer > 4500) {
        executeManoeuvre(NONE_M, 0.0);
        systemState = FORWARD_ROAM;
        sensors.setSweeping();
      }
      break;
      
    case REVERSE:
      // Never implemented due to time constraints.
      break;
    case NONE:
      // Debug state - we don't want to end up here!
      sensors.setCenter();
      executeManoeuvre(NONE_M, 0.0);
      break;
    default:
      systemState = NONE;
  }
  
  // Debug state machine printouts.
  #ifdef RB_DEBUG
  Serial.print("\tState: ");
  if (systemState == FORWARD_ROAM) {
    Serial.print("FORWARD_ROAM");
  } else if (systemState == FORWARD_WEIGHT) {
    Serial.print("FORWARD_WEIGHT");
  } else if (systemState == TURN) {
    Serial.print("TURN");
  } else if (systemState == AVOID) {
    Serial.print("AVOID");
  } else if (systemState == PICKUP) {
    Serial.print("PICKUP");
    Serial.print("\t Left motor: "); Serial.print(leftMotor.read());
    Serial.print(" Right motor: "); Serial.print(rightMotor.read());
  } else if (systemState == NONE) {
    Serial.print("NONE");
  }
  #endif

  Serial.println();

  // !! RESET THE WATCHDOG TIMER !!
  wdt_reset();
  // !! RESET THE WATCHDOG TIMER !!

  //== END FUNCTION ==
}

