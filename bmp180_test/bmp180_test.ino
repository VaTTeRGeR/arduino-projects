#include <SFE_BMP180.h>

int led = 13;

SFE_BMP180 sensor;

double t,p,p0,a0;

// the setup routine runs once when you press reset:
void setup() {                
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);
  sensor.begin();
  
  double pSum = 0;

  for(byte b = 0; b < 64; b++) {
  delay(sensor.startTemperature());
    sensor.getTemperature(t);
    delay(sensor.startPressure(3));
    sensor.getPressure(p0, t);
    pSum += p0;
  }

  p = p0 = pSum / 64;
  a0 = sensor.altitude(p0,p0);
}

// the loop routine runs over and over again forever:
void loop() {
  delay(sensor.startTemperature());
  sensor.getTemperature(t);
  delay(sensor.startPressure(3));

  double pN;
  sensor.getPressure(pN, t);

  p = 0.9*p + 0.1*pN;

  double da = abs(sensor.altitude(p,p0) - a0);
  
  if(da <= 0.5) {
    digitalWrite(led, HIGH);
  } else {
    digitalWrite(led, LOW);
  }
}
