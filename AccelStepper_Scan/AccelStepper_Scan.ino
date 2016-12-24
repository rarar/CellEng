#include <AccelStepper.h>

#define HOME_BUTTON 8
#define FLIP_DIR_BUTTON 9
#define PAUSE_RUN_SWITCH 10
#define MAN_AUTO_SWITCH 11
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
int sign = 1;
int pos = MAX_DISTANCE;

unsigned long lastFlipDebounceTime = 0;
unsigned long lastHomeDebounceTime = 0;
unsigned long debounceDelay = 1;

// Direction Variables
int flipDirState;
int lastFlipDirState = LOW;

// Home Button Variables
int homeState;
int lastHomeState = LOW;

boolean isManual = true;

void setup()
{

  stepper.setMaxSpeed(MAX_SPEED);
  //  stepper.setAcceleration(800);

  pinMode(HOME_BUTTON, INPUT_PULLUP);
  pinMode(FLIP_DIR_BUTTON, INPUT_PULLUP);
  pinMode(PAUSE_RUN_SWITCH, INPUT_PULLUP);
  pinMode(MAN_AUTO_SWITCH, INPUT_PULLUP);
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
        sign = -1 * sign;
        //        Serial.print("new pos = "); Serial.println(sign*pos);
        stepper.moveTo(sign * pos);
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
        stepper.setCurrentPosition(0);
        stepper.moveTo(sign * pos);
        delay(1000); // Force a pause as user feedback
      }
    }
  }
  lastHomeState = reading;
}

void loop()
{
  // Check if we're powered on
  if (digitalRead(PAUSE_RUN_SWITCH)) return;

  static float current_speed = 0.0;
  static int analog_read_counter = 1000;
  static int velocity_val = 0;
  static int distance_val = 0;

  updateDirection();
  updateHomeState();


  if (digitalRead(MAN_AUTO_SWITCH) == LOW) {
    if (analog_read_counter > 0) {
      analog_read_counter--;
    } else {
      analog_read_counter = 3000;
      // now read the pot (from 0 to 1023)
      velocity_val = analogRead(VELOCITY_POT);
      distance_val = map(analogRead(DISTANCE_POT), 0, 1023, 0, MAX_DISTANCE);
      if (distance_val != pos) {
        pos = distance_val;
        stepper.stop();
        stepper.moveTo(sign * pos);
      }
      current_speed = ((velocity_val / 1023.0) * (MAX_SPEED - MIN_SPEED)) + MIN_SPEED;
      stepper.setSpeed(current_speed);
    }
    //    Serial.println("auto mode");
    if (stepper.distanceToGo() == 0) {
      //      Serial.print("new pos = ");
      //      Serial.println(pos);
      sign = -1 * sign;
      stepper.moveTo(sign * pos);
      stepper.setSpeed(current_speed);
      //      Serial.print("current speed = ");
      //      Serial.println(current_speed);
    }
        Serial.print("stepper position = ");
        Serial.print(stepper.currentPosition());
        Serial.print(" ");
        Serial.println(stepper.targetPosition());
    stepper.runSpeedToPosition();
  } else if (digitalRead(MAN_AUTO_SWITCH) == HIGH) {

    // MANUAL MODE HERE
    // flip direction still works

    int localD = analogRead(DISTANCE_POT);
    localD = map(localD, 0, 1023, 0, MAX_DISTANCE);
    int localV = analogRead(VELOCITY_POT);
    localV = ((localV / 1023.0) * (MAX_SPEED - MIN_SPEED)) + MIN_SPEED;
    stepper.moveTo(sign * localD);
    stepper.setSpeed(localV);
    stepper.runSpeedToPosition();
  }

}
