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
  int16_t countsLeft = encoders.getCountsLeft();
  int16_t countsRight = encoders.getCountsRight();


  motors.setSpeeds(100, 100);
    if(countsLeft < countsRight)
    {
      motors.setSpeeds(100*4, 100);
    }

    else if(countsLeft > countsRight)
    {
      motors.setSpeeds(100, 100*4);
    }

}
