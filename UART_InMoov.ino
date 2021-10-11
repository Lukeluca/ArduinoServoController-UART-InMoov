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

  This class is responsible for making sure the
  amounts do not cause the servos to exceed their
  limits.

  This code also returns errors and messages on the
  return serial connection.

  Error - Description
  ------------------------------------
  E100  - Unsupported Servo (Servo not found)

  
****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "AdaHat.h"

// These are servo-specific values, which then correlate with the zero (min) & 180 (max) degrees
#define SERVO_HEAD_TWIST_MIN  180 // this is the 'minimum' pulse length count (out of 4096)
#define SERVO_HEAD_TWIST_MAX  580 // this is the 'maximum' pulse length count (out of 4096)

#define SERVO_HEAD_VERT_MIN  140 // this is the 'minimum' pulse length count (out of 4096)
#define SERVO_HEAD_VERT_MAX  550 // this is the 'maximum' pulse length count (out of 4096)

#define SERVO_HEAD_MOUTH_MIN 135
#define SERVO_HEAD_MOUTH_MAX 550

#define SERVO_RIGHT_BICEP_TWIST_MIN 180
#define SERVO_RIGHT_BICEP_TWIST_MAX 580

// most servos support PWMs from 150 to 600
#define SERVO_UNCALIBRATED_DEFAULT_MIN 150
#define SERVO_UNCALIBRATED_DEFAULT_MAX 600

// Pins on the Ada Hat for the servos
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

#define PIN_INVERT PIN_RIGHT_MIDDLE

// LEFT ARM PINS
#define PIN_LEFT_THUMB 15
#define PIN_LEFT_INDEX 16
#define PIN_LEFT_MIDDLE 17
#define PIN_LEFT_RING 18
#define PIN_LEFT_PINKY 19

#define PIN_LEFT_WRIST 20

//#define PIN_INVERT PIN_LEFT_MIDDLE



// The minimum and maximum servo values (in the 0-180 range) for each servo
// The UART commands will turn the 0-100 range into this range
#define HEAD_TWIST_MIN 40 // to the right
#define HEAD_TWIST_MAX 100 // to the left

#define HEAD_VERT_MIN 20 // down
#define HEAD_VERT_MAX 90 // up // 120 is true max once camera cable will not be a limiter

#define HEAD_MOUTH_MIN 45
#define HEAD_MOUTH_MAX 90 

#define RIGHT_BICEP_TWIST_MIN 45
#define RIGHT_BICEP_TWIST_MAX 120

#define RIGHT_THUMB_MIN 10
#define RIGHT_THUMB_MAX 100

#define RIGHT_INDEX_MIN 0
#define RIGHT_INDEX_MAX 80

#define RIGHT_MIDDLE_MIN 40 // some issues with this servo
#define RIGHT_MIDDLE_MAX 140

#define RIGHT_RING_MIN 20
#define RIGHT_RING_MAX 165

#define RIGHT_PINKY_MIN 45
#define RIGHT_PINKY_MAX 165

#define RIGHT_WRIST_MIN 40
#define RIGHT_WRIST_MAX 120

//LEFT
#define LEFT_THUMB_MIN 10
#define LEFT_THUMB_MAX 100

#define LEFT_INDEX_MIN 0
#define LEFT_INDEX_MAX 80

#define LEFT_MIDDLE_MIN 40 // some issues with this servo
#define LEFT_MIDDLE_MAX 140

#define LEFT_RING_MIN 20
#define LEFT_RING_MAX 165

#define LEFT_PINKY_MIN 45
#define LEFT_PINKY_MAX 165

#define LEFT_WRIST_MIN 40
#define LEFT_WRIST_MAX 120

AdaHat hat(0);

void setup() {
  // Turn the Serial Protocol ON
  Serial.begin(115200);
  Serial.println("M:InMoov Arduino Online");

  hat.setup();
  hat.setupServo(PIN_HEAD_TWIST,        SERVO_HEAD_TWIST_MIN,           SERVO_HEAD_TWIST_MAX);
  hat.setupServo(PIN_HEAD_VERT,          SERVO_HEAD_VERT_MIN,           SERVO_HEAD_VERT_MAX);
  hat.setupServo(PIN_HEAD_MOUTH,        SERVO_HEAD_MOUTH_MIN,           SERVO_HEAD_MOUTH_MAX);
  hat.setupServo(PIN_RIGHT_BICEP_TWIST, SERVO_RIGHT_BICEP_TWIST_MIN,    SERVO_RIGHT_BICEP_TWIST_MAX);
  hat.setupServo(PIN_RIGHT_THUMB,       SERVO_UNCALIBRATED_DEFAULT_MIN, SERVO_UNCALIBRATED_DEFAULT_MAX);
  hat.setupServo(PIN_RIGHT_INDEX,       SERVO_UNCALIBRATED_DEFAULT_MIN, SERVO_UNCALIBRATED_DEFAULT_MAX);
  hat.setupServo(PIN_RIGHT_MIDDLE,      SERVO_UNCALIBRATED_DEFAULT_MIN, SERVO_UNCALIBRATED_DEFAULT_MAX);
  hat.setupServo(PIN_RIGHT_RING,        SERVO_UNCALIBRATED_DEFAULT_MIN, SERVO_UNCALIBRATED_DEFAULT_MAX);
  hat.setupServo(PIN_RIGHT_PINKY,       SERVO_UNCALIBRATED_DEFAULT_MIN, SERVO_UNCALIBRATED_DEFAULT_MAX);
  hat.setupServo(PIN_RIGHT_WRIST,       SERVO_UNCALIBRATED_DEFAULT_MIN, SERVO_UNCALIBRATED_DEFAULT_MAX);
  //TODO: Setup left arm
}


const byte numChars = 32;
char commands[numChars];
boolean newData = false;

void loop() {
  recvWithEndMarker();
  processCommands();
  turnOffIdleServos();
}


void recvWithEndMarker() {
  static byte ndx = 0;
  char endMarker = '\n';
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
      commands[ndx] = '\0'; // terminate the string
      ndx = 0;
      newData = true;
    }
  }
}

void turnOffIdleServos() {
  hat.turnOffIdleServos();
}


// Known commands, separated by spaces
// Commands follow this pattern: <Servo><Value>
// Servo : one of the two character abbreviations below.
// Value : either an absolute value, or a +/- value to adjust.
// 
// examples:
//    HH90 : set Head Horizontal servo to 90 percent.
//    HH+10 : set Head Horizontal servo to 10 degrees greater than it's current value
//

void processCommands() {
  if (newData == true) {

    // Sending commands back
    Serial.print("C:");
    Serial.println(commands);
    
    char* command = strtok(commands," \n");
    while (command != NULL)
    {
        //Serial.print("C:");
        //Serial.println(command);
      
        char servo[3];
        servo[0] = command[0];
        servo[1] = command[1];
        servo[2] = '\0';

        // Split the command in two values, servo & value
        char* value_str = strpbrk(command, "+-1234567890");
        bool relative = false;
        long value = 0;
        if (value_str[0] == '+' || value_str[0] == '-') {
          // Relative adjustment
          relative = true;
        }
        value = atol(value_str);

        // Return commands to acknowledge
        Serial.print("M:Received command - ");
        Serial.print(servo);
        if (relative == true) {
          Serial.print(" relative");
        }
        Serial.print(" value ");
        Serial.print(value, DEC);
        Serial.println("%");

        
        if (strcmp(servo,"DP") == 0) {
          setDefaultPositions();
        } else {
          sendCommand(servo, value, relative);
        }
        
        // Find the next command in input string
        command = strtok(NULL, " ");
    }
    
    
    newData = false;
  }
}

void setDefaultPositions() {
  int pinAmount = 10;
  int pins[pinAmount] = { 
    PIN_HEAD_TWIST,
    PIN_HEAD_VERT,
    PIN_HEAD_MOUTH,
    PIN_RIGHT_BICEP_TWIST,
    PIN_RIGHT_THUMB,
    PIN_RIGHT_INDEX,
    PIN_RIGHT_MIDDLE,
    PIN_RIGHT_RING,
    PIN_RIGHT_PINKY,
    PIN_RIGHT_WRIST
//    PIN_LEFT_THUMB,
//    PIN_LEFT_INDEX,
//    PIN_LEFT_MIDDLE,
//    PIN_LEFT_RING,
//    PIN_LEFT_PINKY,
//    PIN_LEFT_WRIST
  }; // TODO: Include left when ready

  
  for (int i = 0; i < sizeof(pins) -1; i++) {
    int servo_min = getServoMin(pins[i]);
    int servo_max = getServoMax(pins[i]);
    hat.setServoDegrees(pins[i], getCalculatedDegreesFromPercentage(50, servo_min, servo_max));
  }
}

void sendCommand(char servo[], int value, bool relative) {
  int control_pin = getControlPin(servo);
  if (control_pin > -1) {
    int servo_min = getServoMin(control_pin);
    int servo_max = getServoMax(control_pin);
    // middle servo is inverted, where normal max would close, not open
    if (control_pin == PIN_INVERT) {
      value = 100-value;
    }
    if (!relative) {
      hat.setServoDegrees(control_pin, getCalculatedDegreesFromPercentage(value, servo_min, servo_max));
    } else {
      if (hat.getServoDegrees(control_pin) == -1) { // has not been set before, relative will be from center
        hat.setServoDegrees(control_pin, getCalculatedDegreesFromPercentage(50, servo_min, servo_max));
      }
      hat.setServoDegrees(control_pin, getCalculatedRelativeDegrees(value, servo_min, servo_max, hat.getServoDegrees(control_pin)));
    }
    
  } else {
    Serial.print("E100: Unsupported Servo: ");
    Serial.println(servo);
  }
}

// Takes a command percentage and a range, and returns a safe range
int getCalculatedDegreesFromPercentage(int commandValue, int minimum, int maximum) {
  if (commandValue > 100) {
    commandValue = 100;
  }
  if (commandValue < 0) {
    commandValue = 0;
  }
  
  return map(commandValue, 0, 100, minimum, maximum);
}

// Takes a command percentage and a range and a current value, and returns a safe degrees
int getCalculatedRelativeDegrees(int commandValue, int minimum, int maximum, int current) {
  int current_percent = map(current, minimum, maximum, 0, 100);
  
  int new_percent = current_percent + commandValue;
  
  if (new_percent > 100) {
    new_percent = 100;
  }
  if (new_percent < 0) {
    new_percent = 0;
  }
  
  return map(new_percent, 0, 100, minimum, maximum);
}

// returns the control pin based on the servo command
// -1 is error
int getControlPin(char servo[]) {
  if (strcmp(servo,"HH") == 0) return PIN_HEAD_TWIST;
  if (strcmp(servo,"HV") == 0) return PIN_HEAD_VERT;
  if (strcmp(servo,"HM") == 0) return PIN_HEAD_MOUTH;

  if (strcmp(servo,"RB") == 0) return PIN_RIGHT_BICEP_TWIST;
  if (strcmp(servo,"RT") == 0) return PIN_RIGHT_THUMB;
  
  if (strcmp(servo,"RI") == 0) return PIN_RIGHT_INDEX;
  if (strcmp(servo,"RM") == 0) return PIN_RIGHT_MIDDLE;
  if (strcmp(servo,"RR") == 0) return PIN_RIGHT_RING;
  if (strcmp(servo,"RP") == 0) return PIN_RIGHT_PINKY;

  if (strcmp(servo,"RW") == 0) return PIN_RIGHT_WRIST;

//  if (strcmp(servo,"LB") == 0) return PIN_LEFT_BICEP_TWIST; // Not available
  if (strcmp(servo,"LT") == 0) return PIN_LEFT_THUMB;
  
  if (strcmp(servo,"LI") == 0) return PIN_LEFT_INDEX;
  if (strcmp(servo,"LM") == 0) return PIN_LEFT_MIDDLE;
  if (strcmp(servo,"LR") == 0) return PIN_LEFT_RING;
  if (strcmp(servo,"LP") == 0) return PIN_LEFT_PINKY;

  if (strcmp(servo,"LW") == 0) return PIN_LEFT_WRIST;


  // error, none found
  return -1;
}

int getServoMin(int servoPin) {
  switch (servoPin) {
    case PIN_HEAD_TWIST:    return HEAD_TWIST_MIN; break;
    case PIN_HEAD_MOUTH:    return HEAD_MOUTH_MIN; break;
    case PIN_HEAD_VERT:     return HEAD_VERT_MIN; break;

    case PIN_RIGHT_BICEP_TWIST: return RIGHT_BICEP_TWIST_MIN; break;

    case PIN_RIGHT_THUMB:   return RIGHT_THUMB_MIN; break;
    case PIN_RIGHT_INDEX:   return RIGHT_INDEX_MIN;  break;
    case PIN_RIGHT_MIDDLE:  return RIGHT_MIDDLE_MIN; break;
    case PIN_RIGHT_RING:    return RIGHT_RING_MIN;   break;
    case PIN_RIGHT_PINKY:   return RIGHT_PINKY_MIN;  break;

    case PIN_RIGHT_WRIST:   return RIGHT_WRIST_MIN;  break;

    case PIN_LEFT_THUMB:    return LEFT_THUMB_MIN; break;
    case PIN_LEFT_INDEX:   return LEFT_INDEX_MIN;  break;
    case PIN_LEFT_MIDDLE:  return LEFT_MIDDLE_MIN; break;
    case PIN_LEFT_RING:    return LEFT_RING_MIN;   break;
    case PIN_LEFT_PINKY:   return LEFT_PINKY_MIN;  break;

    case PIN_LEFT_WRIST:   return LEFT_WRIST_MIN;  break;
  }
  
  return 90; // safe default, hopefully
}

int getServoMax(int servoPin) {
  switch (servoPin) {
    case PIN_HEAD_TWIST:    return HEAD_TWIST_MAX; break;
    case PIN_HEAD_MOUTH:    return HEAD_MOUTH_MAX; break;
    case PIN_HEAD_VERT:     return HEAD_VERT_MAX;  break;

    case PIN_RIGHT_BICEP_TWIST: return RIGHT_BICEP_TWIST_MAX; break;

    case PIN_RIGHT_THUMB:   return RIGHT_THUMB_MAX; break;
    case PIN_RIGHT_INDEX:   return RIGHT_INDEX_MAX;  break;
    case PIN_RIGHT_MIDDLE:  return RIGHT_MIDDLE_MAX; break;
    case PIN_RIGHT_RING:    return RIGHT_RING_MAX;   break;
    case PIN_RIGHT_PINKY:   return RIGHT_PINKY_MAX;  break;

    case PIN_RIGHT_WRIST:   return RIGHT_WRIST_MAX;  break;

    
    case PIN_LEFT_THUMB:   return  LEFT_THUMB_MAX; break;
    case PIN_LEFT_INDEX:   return  LEFT_INDEX_MAX;  break;
    case PIN_LEFT_MIDDLE:  return LEFT_MIDDLE_MAX; break;
    case PIN_LEFT_RING:    return LEFT_RING_MAX;   break;
    case PIN_LEFT_PINKY:   return LEFT_PINKY_MAX;  break;

    case PIN_LEFT_WRIST:   return LEFT_WRIST_MAX;  break;
  }
  
  return 90; // safe default, hopefully
}
