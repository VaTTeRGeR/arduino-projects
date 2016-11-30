#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define SERVOMIN    204// 1ms
#define SERVOMAX    408// 2ms
#define SERVOFREQ   50 // HZ

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {
  pwm.begin();
  pwm.setPWMFreq(SERVOFREQ);
}

void loop() {
    pwm.setPWM(0, 0, mapAngle(45));
    delay(100);
}

int mapAngle(int angle){
  angle = min(angle, 135);
  angle = max(angle, 45);
  return map(angle, 45, 135, SERVOMIN, SERVOMAX);
}
