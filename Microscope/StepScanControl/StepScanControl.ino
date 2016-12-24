#include <Stepper.h>

#define HOME_BUTTON 8
#define FLIP_DIR_BUTTON 9
#define PAUSE_RUN_SWITCH 10
#define MAN_AUTO_SWITCH 11
#define VELOCITY_POT A0
#define DISTANCE_POT A1

#define MOTOR_DIR 2
#define MOTOR_STEP 3

#define MAX_DISTANCE 15325
#define MAX_DV 1

int stepCounter = 0;
int stepping = false;

int distance = MAX_DISTANCE;

int cPeriod, tick;


unsigned long lastFlipDebounceTime = 0;
unsigned long lastHomeDebounceTime = 0;
unsigned long debounceDelay = 1;


// Direction Variables
int flipDirState;
int lastFlipDirState = LOW;
boolean inReverse = false;

// Home Button Variables
int homeState;
int lastHomeState = LOW;

boolean autoMode = true;

boolean isOff = true;


void setup()
{
  // Set up the controls
  pinMode(HOME_BUTTON, INPUT);
  pinMode(FLIP_DIR_BUTTON, INPUT);
  pinMode(PAUSE_RUN_SWITCH, INPUT);
  pinMode(MAN_AUTO_SWITCH, INPUT);
  pinMode(VELOCITY_POT, INPUT);
  pinMode(DISTANCE_POT, INPUT);

  digitalWrite(HOME_BUTTON, HIGH);
  digitalWrite(FLIP_DIR_BUTTON, HIGH);
  digitalWrite(PAUSE_RUN_SWITCH, HIGH);
  digitalWrite(MAN_AUTO_SWITCH, HIGH);

  // Set up the motor
  pinMode(MOTOR_DIR, OUTPUT);
  pinMode(MOTOR_STEP, OUTPUT);
  digitalWrite(MOTOR_DIR, LOW);
  digitalWrite(MOTOR_STEP, LOW);

  Serial.begin(115200);
}

//helper function to flip direction
void flipDirection(boolean dir) {
  inReverse = dir;
  cPeriod = 0;
  digitalWrite(MOTOR_DIR, inReverse);
}

/* TO DO: WRITE ONE FUCTION TO DEBOUNCE BUTTONS */

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
        flipDirection(!inReverse);
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
        stepCounter = 0;
        flipDirection(false);
        delay(1000); // Force a pause as user feedback
      }
    }
  }
  lastHomeState = reading;
}


void updateDistance() {
  distance = map(analogRead(DISTANCE_POT), 0, 1024, 0, MAX_DISTANCE);
}

void runStepper(int period) {

  digitalWrite(MOTOR_STEP, HIGH);
  delay(1);
  digitalWrite(MOTOR_STEP, LOW);
  delay(1);

}

void loop()
{

  updateDirection();
  updateHomeState();
  updateDistance();


  // SWITCHES
  isOff = digitalRead(PAUSE_RUN_SWITCH);
  autoMode = digitalRead(MAN_AUTO_SWITCH);

  // POTS
  int period = analogRead(VELOCITY_POT);

  if (autoMode == LOW) {
    //Serial.println("Running Manual");
  } else if (autoMode == HIGH) {
    //Serial.println("Running Auto");
  }


  if (isOff == LOW) {
    stepping = true;
  } else if (isOff == HIGH) {
    stepping = false;
  }

  if (stepping == true) {

    runStepper(period);


    if (!inReverse) {
      stepCounter += 1;
    } else {
      stepCounter -= 1;
    }

    Serial.print("step count = "); Serial.print(stepCounter); Serial.print(" || distance = "); Serial.print(distance);
    Serial.print(" || period = "); Serial.println(cPeriod);

    if (stepCounter >= distance || stepCounter <= 0) {
      flipDirection(!inReverse);
    }
  }

}
