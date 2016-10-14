/*
 * File: PickupController.h
 * PickupController class, developed for the RoboCup control program.
 * Authors: Richard Hofman, Matthew Poole, Daniel Catto [GROUP 9]
 * 15.10.2016
 */

// ensure this library description is only included once
#ifndef Pickup_h
#define Pickup_h

// Include Herkulex smart servo API
#include "Herkulex.h"

// library interface description
class PickupController
{
  // user-accessible "public" interface
  public:
    PickupController(void);
    void init(void);
    void lower(void);
    void vertical(void);
    void raise(void);
    void activateMagnet(void);
    void deactivateMagnet(void);
    int  getPositionA(void);
    int  getPositionB(void);

  // library-accessible "private" interface
  private:
    int targetPosition, currentPosition;
    int motorA, motorB; // motor addresses
    int magnetPin;
};

#endif



