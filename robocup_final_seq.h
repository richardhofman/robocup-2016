
/*
 * Main sketch header for the final RoboCup 2016 control program.
 * Authors: Richard Hofman, Matthew Poole, Daniel Catto [GROUP 9]
 * 15.10.2016
 */

typedef enum {FORWARD_ROAM, FORWARD_WEIGHT, AVOID, TURN, REVERSE, PICKUP, NONE} RCState;
typedef enum {TURN_M, REVERSE_M, FORWARD_M, FORWARD_SLOW_M, NONE_M, PICKUP_M, AVOID_TURN_M} Manoeuvre;

#define PICKUP_DISTANCE 90 // 90mm

