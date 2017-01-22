#include "quaternion.h"

Quaternion q0;
Quaternion q1;

Vector3 v0;

void setup() {
  pinMode(13, OUTPUT);
  
  vset(v0, 1,1,0);
  vnor(v0);
  qidt(q0);
  qsetFromAxis(q1, v0, 15);
  Serial.begin(115200);
}

void loop() {
  unsigned long t0 = micros();
  for(int i = 0; i < 10000; i++) {
    qmul(q0,q1);
  }
  Serial.println(micros()-t0);
  
  digitalWrite(13, HIGH);
  delay(17);
  digitalWrite(13, LOW);
}
