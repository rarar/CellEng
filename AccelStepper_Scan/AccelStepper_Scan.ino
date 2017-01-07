#include <AccelStepper.h>

#define HOME_BUTTON 8
#define FLIP_DIR_BUTTON 9
#define PAUSE_RUN_SWITCH 10
#define VELOCITY_POT A0
#define DISTANCE_POT A1
#define LIMIT_REAR A2
#define LIMIT_FRONT A3
#define NUM_LIMIT_READINGS 10

#define MOTOR_DIR 2
#define MOTOR_STEP 3

#define MAX_DISTANCE 15000
// Define our maximum and minimum speed in steps per second (scale pot to these)
#define  MAX_SPEED 800
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

// Limit Sensors
int limitRearReadings[NUM_LIMIT_READINGS];      // the readings from the analog input
int limitRearReadIndex = 0;              // the index of the current reading
int limitRearTotal = 0;                  // the running total
int limitRearAverage = 0; // the average
boolean overshoot = false;


void setup()
{

  stepper.setMaxSpeed(MAX_SPEED);
  //  stepper.setAcceleration(800);

  pinMode(HOME_BUTTON, INPUT_PULLUP);
  pinMode(FLIP_DIR_BUTTON, INPUT_PULLUP);
  pinMode(PAUSE_RUN_SWITCH, INPUT_PULLUP);
  pinMode(VELOCITY_POT, INPUT);
  pinMode(DISTANCE_POT, INPUT);

  // init limit smoothings
  for (int thisReading = 0; thisReading < NUM_LIMIT_READINGS; thisReading++) {
    limitRearReadings[thisReading] = 0;
  }

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

void checkLimits() {
  static int analog_read_counter = 0;
  // subtract the last reading:
  limitRearTotal = limitRearTotal - limitRearReadings[limitRearReadIndex];
  // read from the sensor:
  limitRearReadings[limitRearReadIndex] = analogRead(LIMIT_REAR);
  // add the reading to the total:
  limitRearTotal = limitRearTotal + limitRearReadings[limitRearReadIndex];
  // advance to the next position in the array:
  limitRearReadIndex = limitRearReadIndex + 1;

  // if we're at the end of the array...
  if (limitRearReadIndex >= NUM_LIMIT_READINGS) {
    // ...wrap around to the beginning:
    limitRearReadIndex = 0;
  }

  // calculate the average:
  limitRearAverage = limitRearTotal / NUM_LIMIT_READINGS;
  Serial.print("Limit Reading = ");
  Serial.print(limitRearAverage);
  Serial.print(" | analog poll = ");
  Serial.println(analog_read_counter);

  // Analog Read Counter provides debounce
  if (limitRearAverage < 920 && analog_read_counter <= 0) {
    Serial.println("Time to turn around!");
    sign = -sign;
    analog_read_counter = 20;
  }

  analog_read_counter--;
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
    current_speed = sign * ((velocity_val / 1023.0) * (MAX_SPEED - MIN_SPEED)) + MIN_SPEED;

    checkLimits();
    //    Serial.print("target = ");
    //    Serial.print(stepper.targetPosition());
    //    Serial.print(" | current = ");
    //    Serial.print(stepper.currentPosition());
    //    Serial.print(" | distance to go = ");
    //    Serial.println(stepper.distanceToGo());
    // Give the stepper a chance to step if it needs to
    if (pos != distance_val) {
      stepper.moveTo(distance_val);
      pos = distance_val;
    }

    // Update the stepper to run at this new speed
    stepper.setSpeed(current_speed);
  }

  stepper.runSpeed();
}

