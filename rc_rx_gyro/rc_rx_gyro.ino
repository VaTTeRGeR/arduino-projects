#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define SERVOMIN    205// 1ms
#define SERVOMAX    409// 2ms
#define SERVOFREQ   50 // HZ

#define THROTTLE_HEADER 0

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {
  pinMode(13, OUTPUT);
  
  pwm.begin();
  pwm.setPWMFreq(SERVOFREQ);
  setThrottle(0);
}

void loop() {
  while(true){
    for(int i = 0; i <= 90; i++) {
      pwm.setPWM(15, 0, mapAngle(45+i));
      delay(20);
    }
    for(int i = 0; i <= 90; i++) {
      pwm.setPWM(15, 0, mapAngle(135-i));
      delay(20);
    }
  }
}

void setThrottle(int percent){
  pwm.setPWM(THROTTLE_HEADER, 0, mapAngle(map(percent, 0, 100, 45, 135)));
}

int mapAngle(int angle){
  angle = min(angle, 135);
  angle = max(angle, 45);
  return map(angle, 45, 135, SERVOMIN, SERVOMAX);
}
