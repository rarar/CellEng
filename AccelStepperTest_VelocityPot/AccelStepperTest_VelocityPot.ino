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
  //stepper.setAcceleration(100);
}

void loop()
{
  static float current_speed = 0.0;
  static int analog_read_counter = 1000;
  static char sign = 1;
  static int analog_value = 0;

  if (analog_read_counter > 0) {
    analog_read_counter--;
  } else {
    analog_read_counter = 3000;
    // now read the pot (from 0 to 1023)
    analog_value = analogRead(0);
    stepper.runSpeed();
    current_speed = sign * ((analog_value / 1023.0) * (MAX_SPEED - MIN_SPEED)) + MIN_SPEED;
    stepper.setSpeed(current_speed);
  }

  stepper.runSpeed();
}

