#include <AccelStepper.h>
#include <MultiStepper.h>

// Define our maximum and minimum speed in steps per second (scale pot to these)
#define  MAX_SPEED 500
#define  MIN_SPEED 0.1

// Define a stepper and the pins it will use
AccelStepper stepper(1, 3, 2);

int pos = 5000;

void setup()
{  
  stepper.setMaxSpeed(5000);
  stepper.setAcceleration(110);
}

void loop()
{
  if (stepper.distanceToGo() == 0)
  {
    delay(500);
    pos = -pos;
    stepper.moveTo(pos);
  }
  stepper.run();
}

