#import<Servo.h>

Servo pwm;

int oldValue = -1;

void setup() {
  pinMode(A0, INPUT);
  pinMode(13, OUTPUT);

  pwm.attach(10);
  pwm.write(45);
}

void loop() {
  int in = 1023-analogRead(A0);

  int angle = map(in, 512,1023,45,135);
  angle = max(45, angle);
  angle = min(135, angle);
  pwm.write(angle);
  analogWrite(13, angle);
}
