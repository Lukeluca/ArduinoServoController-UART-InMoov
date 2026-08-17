
#ifndef AdaHat_h
#define AdaHat_h


#include "Wire.h"
#include "Adafruit_PWMServoDriver.h"

// Channels across both PWM boards: 0-15 on board 0, 16-31 on board 1.
#define ADAHAT_CHANNELS 32

// How long a servo can sit untouched before it is powered down, to stop it
// buzzing and drawing current while holding a position nobody asked for.
#define ADAHAT_IDLE_TIMEOUT_MS 5000


class AdaHat {

  private:
    Adafruit_PWMServoDriver _pwm0;
    Adafruit_PWMServoDriver _pwm1;

    // safe defaults if something else goes wrong, aka 90 degrees
    int _servoMins[ADAHAT_CHANNELS] = { 375, 375, 375, 375, 375, 375, 375, 375, 375, 375, 375, 375, 375, 375, 375, 375,
                                        375, 375, 375, 375, 375, 375, 375, 375, 375, 375, 375, 375, 375, 375, 375, 375 };
    int _servoMaxs[ADAHAT_CHANNELS] = { 375, 375, 375, 375, 375, 375, 375, 375, 375, 375, 375, 375, 375, 375, 375, 375,
                                        375, 375, 375, 375, 375, 375, 375, 375, 375, 375, 375, 375, 375, 375, 375, 375 };

    // last sent servo positions (in degrees)
    int _servoPoss[ADAHAT_CHANNELS] = { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
                                        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 };
    unsigned long _lastUpdated[ADAHAT_CHANNELS] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    // Channels setupServo() has been called for. Only these are ever driven or
    // idled off, so unused channels generate no I2C traffic at all.
    bool _registered[ADAHAT_CHANNELS] = { false };

    // Whether the channel's PWM output is currently off. Latching this means
    // turnOffIdleServos() sends one shutoff per idle period instead of
    // re-sending it on every single pass of the main loop.
    bool _isOff[ADAHAT_CHANNELS] = { false };

    bool isValidPosition(int position);

  public:
    AdaHat(int boardPosition);
    void setup();
    void setupServo(int position, int min, int max);
    void setServoDegrees(int position, int degrees);
    int getServoDegrees(int position);
    void turnOffIdleServos();
};

#endif
