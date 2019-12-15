#include <Romi32U4.h>

Romi32U4Motors motors;
Romi32U4ButtonA buttonA;
Romi32U4Encoders encoders;

void setup()
{
  // Wait for the user to press button A.
  buttonA.waitForButton();

  // Delay so that the robot does not move away while the user is
  // still touching it.
  delay(1000);
}

void loop()
{
  // Run left and right motor forward.
  //go
  motors.setSpeeds(200,201);
  delay(1300);
  motors.setSpeeds(0,0);
  delay(1000);
  //turn
  motors.setSpeeds(0,100);
  delay(1100);
  //go
  motors.setSpeeds(200,201);
  delay(1300);
  motors.setSpeeds(0,0);
  delay(1000);
  //turn
  motors.setSpeeds(0,100);
  delay(1100);
  //go
  motors.setSpeeds(200,201);
  delay(1300);
  motors.setSpeeds(0,0);
  delay(1000);
  //turn
  motors.setSpeeds(0,100);
  delay(1100);
  //go
  motors.setSpeeds(200,201);
  delay(1300);
  motors.setSpeeds(0,0);
  delay(1000);
  //turn
  motors.setSpeeds(0,100);
  delay(1100);
  motors.setSpeeds(0,0);
  delay(1000);
}
