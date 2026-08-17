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


// Guards every array access below - these arrays are indexed by raw channel
// number, so an out-of-range pin would otherwise corrupt adjacent memory.
bool AdaHat::isValidPosition(int position) {
  return position >= 0 && position < ADAHAT_CHANNELS;
}

void AdaHat::setupServo(int position, int min, int max) {

  if (!isValidPosition(position)) {
    return;
  }

  _servoMins[position] = min;
  _servoMaxs[position] = max;
  _registered[position] = true;

}

void AdaHat::setServoDegrees(int position, int degrees) {

  if (!isValidPosition(position)) {
    return;
  }

  int currentMin = _servoMins[position];
  int currentMax = _servoMaxs[position];

  _servoPoss[position] = degrees; // setting last position
  _lastUpdated[position] = millis();
  _isOff[position] = false;

  int pulseLength = map(degrees, 0, 180, currentMin, currentMax);

  if (position < 16) {
    _pwm0.setPWM(position, 0, pulseLength);
  } else {
    _pwm1.setPWM(position-16, 0, pulseLength);
  }

}

int AdaHat::getServoDegrees(int position) {
  if (!isValidPosition(position)) {
    return -1;
  }
  return _servoPoss[position];
}

void AdaHat::turnOffIdleServos() {
  // Covers both boards - this used to stop at channel 16, so the "pin >= 16"
  // branch was dead code and the second board's servos never idled out.
  for (int pin = 0; pin < ADAHAT_CHANNELS; pin++) {
    // Skip channels with no servo on them, and ones already powered down.
    // Without this the loop re-sent a shutoff to all 32 channels on every
    // pass, flooding the I2C bus (and hitting board 1 even when it isn't
    // plugged in) rather than sending one shutoff per idle period.
    if (!_registered[pin] || _isOff[pin]) {
      continue;
    }

    // Subtract rather than add, so this still behaves correctly when millis()
    // rolls over (~49 days of uptime).
    if (millis() - _lastUpdated[pin] >= ADAHAT_IDLE_TIMEOUT_MS) {
      if (pin < 16) {
        _pwm0.setPWM(pin, 0, 4096);
      } else {
        _pwm1.setPWM(pin-16, 0, 4096);
      }
      _isOff[pin] = true;
    }
  }
}
