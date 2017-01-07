#include <AccelStepper.h>

#define HOME_BUTTON 8
#define FLIP_DIR_BUTTON 9
#define PAUSE_RUN_SWITCH 10
#define VELOCITY_POT A0
#define DISTANCE_POT A1

#define MOTOR_DIR 2
#define MOTOR_STEP 3

#define MAX_DISTANCE 15000
// Define our maximum and minimum speed in steps per second (scale pot to these)
#define  MAX_SPEED 500
#define  MIN_SPEED 0.1

// Define a stepper and the pins it will use
AccelStepper stepper(1, 3, 2);
int pos = MAX_DISTANCE;

unsigned long lastFlipDebounceTime = 0;
unsigned long lastHomeDebounceTime = 0;
unsigned long debounceDelay = 1;

// Direction Variables
int flipDirState;
int lastFlipDirState = LOW;
boolean inReverse = false;
char sign = 1;

// Home Button Variables
int homeState;
int lastHomeState = LOW;

void setup()
{

  stepper.setMaxSpeed(MAX_SPEED);
  //  stepper.setAcceleration(800);

  pinMode(HOME_BUTTON, INPUT_PULLUP);
  pinMode(FLIP_DIR_BUTTON, INPUT_PULLUP);
  pinMode(PAUSE_RUN_SWITCH, INPUT_PULLUP);
  pinMode(VELOCITY_POT, INPUT);
  pinMode(DISTANCE_POT, INPUT);

  Serial.begin(115200);
}

// Simple debounce to toggle the flip direction pushbutton
void updateDirection() {
  int reading = digitalRead(FLIP_DIR_BUTTON);
  if (reading != lastFlipDirState) {
    lastFlipDebounceTime = millis();
  }

  if ((millis() - lastFlipDebounceTime) > debounceDelay) {
    if (reading != flipDirState) {
      flipDirState = reading;
      if (flipDirState == LOW) {
        inReverse = !inReverse;
        Serial.println("flipping!");
        if (inReverse) {
          sign = -1;
        } else if (!inReverse) {
          sign = 1;
        }
      }
    }
  }
  lastFlipDirState = reading;
}

void updateHomeState() {
  int reading = digitalRead(HOME_BUTTON);
  if (reading != lastHomeState) {
    lastHomeDebounceTime = millis();
  }

  if ((millis() - lastHomeDebounceTime) > debounceDelay) {
    if (reading != homeState) {
      homeState = reading;
      if (homeState == LOW) {
        Serial.println("Setting new home direction");
        stepper.setCurrentPosition(0);
        delay(1000);
        stepper.moveTo(pos);
      }
    }
  }
  lastHomeState = reading;
}

void loop()
{


  static float current_speed = 0.0;
  static int velocity_val = 0;
  static int distance_val = 0;
  static int analog_read_counter = 1000;

  // There is some noise here - track absolute values to make sure that we haven't overshot
  if ((stepper.distanceToGo() == 0) || (abs(stepper.targetPosition()) < abs(stepper.currentPosition()))) {
    sign = -sign;
    stepper.setCurrentPosition(0);
    stepper.moveTo(pos);
  }

  // check button states
  updateDirection();
  updateHomeState();


  // Check if we're powered on
  if (digitalRead(PAUSE_RUN_SWITCH)) return;

  // poll pots only so often to optimize
  if (analog_read_counter > 0 ) {
    analog_read_counter--;
  } else {
    analog_read_counter = 3000;
    // Now read the pot (from 0 to 1023)
    velocity_val = analogRead(VELOCITY_POT);
    distance_val = analogRead(DISTANCE_POT);
    distance_val = sign * map(distance_val, 0, 1023, 0, MAX_DISTANCE);
    //    stepper.moveTo(sign * distance_val);
    Serial.print("target = ");
    Serial.print(stepper.targetPosition());
    Serial.print(" | current = ");
    Serial.print(stepper.currentPosition());
    Serial.print(" | distance to go = ");
    Serial.println(stepper.distanceToGo());
    // Give the stepper a chance to step if it needs to
    if (pos != distance_val) {
      stepper.moveTo(distance_val);
      pos = distance_val;
    }
    //  And scale the pot's value from min to max speeds
    current_speed = sign * ((velocity_val / 1023.0) * (MAX_SPEED - MIN_SPEED)) + MIN_SPEED;
    // Update the stepper to run at this new speed
    stepper.setSpeed(current_speed);
  }

  stepper.runSpeed();
}

