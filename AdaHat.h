
#ifndef AdaHat_h
#define AdaHat_h


#include "Wire.h"
#include "Adafruit_PWMServoDriver.h"


class AdaHat {

  private:
    Adafruit_PWMServoDriver _pwm0;
    Adafruit_PWMServoDriver _pwm1;

    // safe defaults if something else goes wrong, aka 90 degrees
    int _servoMins[32] = { 375, 375, 375, 375, 375, 375, 375, 375, 375, 375, 375, 375, 375, 375, 375, 375,
                           375, 375, 375, 375, 375, 375, 375, 375, 375, 375, 375, 375, 375, 375, 375, 375 };
    int _servoMaxs[32] = { 375, 375, 375, 375, 375, 375, 375, 375, 375, 375, 375, 375, 375, 375, 375, 375,
                           375, 375, 375, 375, 375, 375, 375, 375, 375, 375, 375, 375, 375, 375, 375, 375 };

    // last sent servo positions (in degrees)
    int _servoPoss[32] = { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
                           -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 };
    unsigned long _lastUpdated[32] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  public:
    AdaHat(int boardPosition);
    void setup();
    void setupServo(int position, int min, int max);
    void setServoDegrees(int position, int degrees);
    int getServoDegrees(int position);
    void turnOffIdleServos();
};

#endif
