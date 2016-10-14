/*
 * File: PickupController.cpp
 * PickupController class, developed for the RoboCup control program.
 * Authors: Richard Hofman, Matthew Poole, Daniel Catto [GROUP 9]
 * 15.10.2016
 */

#include "PickupController.h"

/*
 * Constructor function for the pickup mechanism class.
*/
PickupController::PickupController()
{
    // Store Herkulex singleton
    //herkulexControl = HerkulexControl
    
    // Set motor addresses and magnet pin
    motorA = 0x01;
    motorB = 0x02;
    magnetPin = 34;
}

/*
 * Set up the magnet control pin.
 * (Herkulex motors are handled in main sketch)
*/
void PickupController::init(void) {
    pinMode(magnetPin, OUTPUT);
    digitalWrite(magnetPin, LOW);
}

/*
 * Rather self explanatory.
*/
void PickupController::activateMagnet(void) {
  digitalWrite(magnetPin, HIGH);
}

/*
 * Rather self explanatory.
*/
void PickupController::deactivateMagnet(void) {
  digitalWrite(magnetPin, LOW);
}

/*
 * Lower the pickup crane to the level of a weight.
*/
void PickupController::lower(void){
    Herkulex.moveOneAngle(motorA, 133.25, 800, LED_BLUE);
    Herkulex.moveOneAngle(motorB, -136.83, 800, LED_BLUE);
}

/*
 * Stow the pickup mechanism in the "vertical" position,
 * when not in use.
*/
void PickupController::vertical(void){
    Herkulex.moveOneAngle(motorA, 20, 800, LED_BLUE);
    Herkulex.moveOneAngle(motorB, -34, 800, LED_BLUE);
}

/*
 * Raise the pickup mechanism, to carry a captured weight over
 * the collection ramp area within the robot's chassis.
*/
void PickupController::raise(void){
    Herkulex.moveOneAngle(motorA, -43.22, 800, LED_BLUE);  //move motorB forward
    Herkulex.moveOneAngle(motorB, 41.60, 800, LED_BLUE);
}

/*
 * Return the angle (NOT encoder position) of motorA.
*/
int PickupController::getPositionA(void) {
    return Herkulex.getAngle(motorA);
}

/*
 * Return the angle (NOT encoder position) of motorB.
*/
int PickupController::getPositionB(void) {
    return Herkulex.getAngle(motorB);
}
