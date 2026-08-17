/***************************************************
  The purpose of this code is to provide a UART
  interface to a collection of servos for an InMoov
  robot.

  The serial connection accepts commands that consist
  of a servo abbreviation, and a value. The value is
  either an amount between 0 and 100, to represent the
  percentage of the servo to adjust, or a relative
  percentage to change the servo, from -100 to +100.

  Servo - Description
  ------------------------------------
  HH    - Head Horizontal (Rotation)
  HV    - Head Vertical (Neck)
  HM    - Mouth

  RB    - Right Bicep Twist
  RW    - Right Wrist Twist

  RT    - Right Thumb
  RI    - Right Index
  RM    - Right Middle
  RR    - Right Ring
  RP    - Right Pinky

  LW    - Left Wrist Twist

  LT    - Left Thumb
  LI    - Left Index
  LM    - Left Middle
  LR    - Left Ring
  LP    - Left Pinky

  DP    - Default Positions. Takes no value, e.g. "DP"

  Every servo (pin, degree range, calibrated pulse range,
  and whether it's inverted) is defined once in the SERVOS
  table below. Adding, recalibrating, or enabling a servo
  means editing one row there - nothing else in this file
  needs to change.

  Error - Description
  ------------------------------------
  E100  - Unsupported Servo (Servo code not found)
  E101  - Missing value (no digits found after the servo code)
  E102  - Servo not enabled (defined in the table, but not wired up/calibrated)
  E103  - Malformed command (shorter than a two-character servo code)

****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "AdaHat.h"

// Most uncalibrated servos support PWM pulse lengths from 150 to 600.
#define SERVO_UNCALIBRATED_DEFAULT_MIN 150
#define SERVO_UNCALIBRATED_DEFAULT_MAX 600

// Pins on the Ada Hat for the servos (0-15 = board 0, 16-31 = board 1)
#define PIN_HEAD_TWIST 0
#define PIN_HEAD_MOUTH 1
#define PIN_HEAD_VERT 8

#define PIN_RIGHT_BICEP_TWIST 2

#define PIN_RIGHT_THUMB 3
#define PIN_RIGHT_INDEX 4
#define PIN_RIGHT_MIDDLE 5
#define PIN_RIGHT_RING 6
#define PIN_RIGHT_PINKY 7

#define PIN_RIGHT_WRIST 9

// LEFT ARM PINS
#define PIN_LEFT_THUMB 15
#define PIN_LEFT_INDEX 16
#define PIN_LEFT_MIDDLE 17
#define PIN_LEFT_RING 18
#define PIN_LEFT_PINKY 19

#define PIN_LEFT_WRIST 20

struct ServoConfig {
  const char code[3];  // two-letter UART command code
  uint8_t pin;         // PCA9685 channel
  int minDeg;          // degrees at command value 0
  int maxDeg;          // degrees at command value 100
  int minPulse;        // calibrated PWM pulse length (out of 4096) at 0 degrees
  int maxPulse;        // calibrated PWM pulse length (out of 4096) at 180 degrees
  bool inverted;       // true if command value 0 should be this servo's far/open end
  bool enabled;        // false = not wired up / not calibrated yet - setup() skips it
                        //         and commands targeting it return E102 instead of moving it
};

// clang-format off
ServoConfig SERVOS[] = {
  // code            pin                   minDeg maxDeg  minPulse                        maxPulse                        inverted enabled
  { "HH", PIN_HEAD_TWIST,                  40,    100,    180,                            580,                            false,   true  },  // Head Horizontal (Rotation)
  { "HV", PIN_HEAD_VERT,                   20,    90,     140,                            550,                            false,   true  },  // Head Vertical (Neck) - 90 until camera cable routing allows the true mechanical max of 120
  { "HM", PIN_HEAD_MOUTH,                  45,    90,     135,                            550,                            false,   true  },  // Mouth

  { "RB", PIN_RIGHT_BICEP_TWIST,           45,    120,    180,                            580,                            false,   true  },  // Right Bicep Twist
  { "RT", PIN_RIGHT_THUMB,                 10,    100,    SERVO_UNCALIBRATED_DEFAULT_MIN, SERVO_UNCALIBRATED_DEFAULT_MAX, false,   true  },  // Right Thumb
  { "RI", PIN_RIGHT_INDEX,                 0,     80,     SERVO_UNCALIBRATED_DEFAULT_MIN, SERVO_UNCALIBRATED_DEFAULT_MAX, false,   true  },  // Right Index
  { "RM", PIN_RIGHT_MIDDLE,                40,    140,    SERVO_UNCALIBRATED_DEFAULT_MIN, SERVO_UNCALIBRATED_DEFAULT_MAX, true,    true  },  // Right Middle - mounted backwards, so 0% must map to the "open" pulse
  { "RR", PIN_RIGHT_RING,                  20,    165,    SERVO_UNCALIBRATED_DEFAULT_MIN, SERVO_UNCALIBRATED_DEFAULT_MAX, false,   true  },  // Right Ring
  { "RP", PIN_RIGHT_PINKY,                 45,    165,    SERVO_UNCALIBRATED_DEFAULT_MIN, SERVO_UNCALIBRATED_DEFAULT_MAX, false,   true  },  // Right Pinky
  { "RW", PIN_RIGHT_WRIST,                 40,    120,    SERVO_UNCALIBRATED_DEFAULT_MIN, SERVO_UNCALIBRATED_DEFAULT_MAX, false,   true  },  // Right Wrist Twist

  // Left arm: same pins/ranges the original sketch already listed, but it never
  // called setupServo() for them, so they sat frozen. Left "enabled: false" here
  // until you've confirmed on hardware that each one is wired and moves correctly -
  // flip it to true one servo at a time as you verify it.
  { "LT", PIN_LEFT_THUMB,                  10,    100,    SERVO_UNCALIBRATED_DEFAULT_MIN, SERVO_UNCALIBRATED_DEFAULT_MAX, false,   false },  // Left Thumb
  { "LI", PIN_LEFT_INDEX,                  0,     80,     SERVO_UNCALIBRATED_DEFAULT_MIN, SERVO_UNCALIBRATED_DEFAULT_MAX, false,   false },  // Left Index
  { "LM", PIN_LEFT_MIDDLE,                 40,    140,    SERVO_UNCALIBRATED_DEFAULT_MIN, SERVO_UNCALIBRATED_DEFAULT_MAX, true,    false },  // Left Middle - inverted like its right-hand counterpart (per the original's commented-out PIN_INVERT swap); verify before enabling
  { "LR", PIN_LEFT_RING,                   20,    165,    SERVO_UNCALIBRATED_DEFAULT_MIN, SERVO_UNCALIBRATED_DEFAULT_MAX, false,   false },  // Left Ring
  { "LP", PIN_LEFT_PINKY,                  45,    165,    SERVO_UNCALIBRATED_DEFAULT_MIN, SERVO_UNCALIBRATED_DEFAULT_MAX, false,   false },  // Left Pinky
  { "LW", PIN_LEFT_WRIST,                  40,    120,    SERVO_UNCALIBRATED_DEFAULT_MIN, SERVO_UNCALIBRATED_DEFAULT_MAX, false,   false },  // Left Wrist Twist
};
const int SERVO_COUNT = sizeof(SERVOS) / sizeof(SERVOS[0]);
// clang-format on

AdaHat hat(0);

void setup() {
  Serial.begin(115200);
  Serial.println("M:InMoov Arduino Online");

  hat.setup();
  for (int i = 0; i < SERVO_COUNT; i++) {
    if (SERVOS[i].enabled) {
      hat.setupServo(SERVOS[i].pin, SERVOS[i].minPulse, SERVOS[i].maxPulse);
    }
  }
}

const byte numChars = 32;
char commands[numChars];
boolean newData = false;

void loop() {
  recvWithEndMarker();
  processCommands();
  hat.turnOffIdleServos();
}

void recvWithEndMarker() {
  static byte ndx = 0;
  const char endMarker = '\n';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (rc != endMarker) {
      commands[ndx] = rc;
      ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1;
      }

    } else {
      commands[ndx] = '\0';  // terminate the string
      ndx = 0;
      newData = true;
    }
  }
}

// Looks up a servo by its two-letter UART code. Returns NULL if not found.
ServoConfig* findServo(const char* code) {
  for (int i = 0; i < SERVO_COUNT; i++) {
    if (strncmp(SERVOS[i].code, code, 2) == 0) {
      return &SERVOS[i];
    }
  }
  return NULL;
}

// Known commands, separated by spaces
// Commands follow this pattern: <Servo><Value>, or just "DP" for default positions.
// Servo : one of the two character abbreviations above.
// Value : either an absolute value, or a +/- value to adjust.
//
// examples:
//    HH90 : set Head Horizontal servo to 90 percent.
//    HH+10 : set Head Horizontal servo to 10 percent greater than its current value
//    DP : move every enabled servo to its default (50%) position
//
void processCommands() {
  if (!newData) {
    return;
  }

  Serial.print("C:");
  Serial.println(commands);

  char* command = strtok(commands, " \n");
  while (command != NULL) {
    // Every command starts with a two-character servo code. A shorter token
    // can't be valid, and reading command[1] or scanning from command + 2
    // would run past this token's terminator into stale buffer bytes.
    if (strlen(command) < 2) {
      Serial.print("E103: Malformed command: ");
      Serial.println(command);
      command = strtok(NULL, " ");
      continue;
    }

    char code[3] = { command[0], command[1], '\0' };

    if (strcmp(code, "DP") == 0) {
      Serial.println("M:Received command - DP (default positions)");
      setDefaultPositions();
    } else {
      // value_str points at the first digit/sign after the 2-letter code.
      // A command with no digits at all (e.g. malformed input) has no
      // value to parse, so bail out instead of dereferencing NULL.
      char* value_str = strpbrk(command + 2, "+-0123456789");
      if (value_str == NULL) {
        Serial.print("E101: Missing value for command: ");
        Serial.println(command);
      } else {
        bool relative = (value_str[0] == '+' || value_str[0] == '-');
        // Clamp at parse time so a garbage value can't wrap when narrowed to
        // int (e.g. atol("65536") -> 0, which would read as "go to minimum").
        int value = (int)constrain(atol(value_str), -100L, 100L);

        Serial.print("M:Received command - ");
        Serial.print(code);
        if (relative) {
          Serial.print(" relative");
        }
        Serial.print(" value ");
        Serial.print(value, DEC);
        Serial.println("%");

        sendCommand(code, value, relative);
      }
    }

    // Find the next command in input string
    command = strtok(NULL, " ");
  }

  newData = false;
}

void setDefaultPositions() {
  for (int i = 0; i < SERVO_COUNT; i++) {
    if (SERVOS[i].enabled) {
      hat.setServoDegrees(SERVOS[i].pin, getCalculatedDegreesFromPercentage(50, SERVOS[i].minDeg, SERVOS[i].maxDeg));
    }
  }
}

void sendCommand(const char* code, int value, bool relative) {
  ServoConfig* servo = findServo(code);
  if (servo == NULL) {
    Serial.print("E100: Unsupported Servo: ");
    Serial.println(code);
    return;
  }
  if (!servo->enabled) {
    Serial.print("E102: Servo not enabled: ");
    Serial.println(code);
    return;
  }

  if (!relative) {
    // Absolute 0-100 value: inverting means flipping which end is 0%.
    int commandValue = servo->inverted ? 100 - value : value;
    hat.setServoDegrees(servo->pin, getCalculatedDegreesFromPercentage(commandValue, servo->minDeg, servo->maxDeg));
  } else {
    // Relative delta: inverting means reversing direction, NOT "100 - delta"
    // (the original bug - a "+10" nudge became a ~90% jump on inverted servos).
    int delta = servo->inverted ? -value : value;
    if (hat.getServoDegrees(servo->pin) == -1) {  // has not been set before, relative will be from center
      hat.setServoDegrees(servo->pin, getCalculatedDegreesFromPercentage(50, servo->minDeg, servo->maxDeg));
    }
    hat.setServoDegrees(servo->pin, getCalculatedRelativeDegrees(delta, servo->minDeg, servo->maxDeg, hat.getServoDegrees(servo->pin)));
  }
}

// Takes a command percentage and a range, and returns a safe range
int getCalculatedDegreesFromPercentage(int commandValue, int minimum, int maximum) {
  commandValue = constrain(commandValue, 0, 100);
  return map(commandValue, 0, 100, minimum, maximum);
}

// Takes a command percentage and a range and a current value, and returns a safe degrees
int getCalculatedRelativeDegrees(int commandValue, int minimum, int maximum, int current) {
  int current_percent = map(current, minimum, maximum, 0, 100);
  int new_percent = constrain(current_percent + commandValue, 0, 100);
  return map(new_percent, 0, 100, minimum, maximum);
}
