/*************************************************** 
  This code provides an abstraction for an Adafruit
  16-channel PWM & Servo driver hat.
  
  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  This class helps keep track of the minimums and maximums
  for each servo, and then does the calculations.

  MAJOR ASSUMPTION: All servos have a 0-180 range
****************************************************/

#include "Wire.h"
#include "Adafruit_PWMServoDriver.h"
#include "AdaHat.h"

AdaHat::AdaHat(int boardPosition)
{
  // TODO use the zero-based board position to change the driver initialization
}

void AdaHat::setup() {

  // called this way, it uses the default address 0x40
  //Adafruit_PWMServoDriver _pwm = Adafruit_PWMServoDriver();
  // you can also call it with a different address you want
  _pwm0 = Adafruit_PWMServoDriver(0x40);
  _pwm1 = Adafruit_PWMServoDriver(0x41);
  // you can also call it with a different address and I2C interface
  //Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(&Wire, 0x40);

  _pwm0.begin();
  _pwm0.setPWMFreq(60);  // Analog servos run at ~60 Hz updates


  _pwm1.begin();
  _pwm1.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  delay(10);
}


void AdaHat::setupServo(int position, int min, int max) {

  if (position == -1) {
    return;
  }

  _servoMins[position] = min;
  _servoMaxs[position] = max;
  
}

void AdaHat::setServoDegrees(int position, int degrees) {

  int currentMin = _servoMins[position];
  int currentMax = _servoMaxs[position];

  _servoPoss[position] = degrees; // setting last position
  _lastUpdated[position] = millis();

  int pulseLength = map(degrees, 0, 180, currentMin, currentMax);

  if (position < 16) {
    _pwm0.setPWM(position, 0, pulseLength);
  } 
  if (position >= 16) {
    _pwm1.setPWM(position-16, 0, pulseLength);
  }
  
}

int AdaHat::getServoDegrees(int position) {
  return _servoPoss[position];
}

void AdaHat::turnOffIdleServos() {
  for (int pin = 0; pin < 16; pin++) {
    if (millis() > _lastUpdated[pin] + 5000) {
      // turn off pin
      if (pin < 16) {
        _pwm0.setPWM(pin, 0, 4096);
      }
      if (pin >= 16) {
        _pwm1.setPWM(pin-16, 0, 4096);
      }
    }
  }
}
