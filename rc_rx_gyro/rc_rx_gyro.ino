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
}

void loop() {
  digitalWrite(13, HIGH);
  setThrottle(100);
  
  delay(7000);
  
  digitalWrite(13, LOW);
  setThrottle(0);
  
  while(true){
    for(int i = 0; i <= 25; i++) {
      setThrottle(i);
      delay(100);
    }
    for(int i = 0; i <= 25; i++) {
      setThrottle(25-i);
      delay(100);
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
