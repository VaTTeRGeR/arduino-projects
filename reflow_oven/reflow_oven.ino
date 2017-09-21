#include <elapsedMillis.h>

elapsedMillis sinceSSRMeasure;
elapsedMillis sinceOvenMeasure;
elapsedMillis sinceSerialSend;

float ssr_temperature  = 0;
float oven_temperature = 0;

boolean oven_ntc_disconnected = false;
boolean ssr_ntc_disconnected = false;

boolean oven_ntc_shorted = false;
boolean ssr_ntc_shorted = false;

boolean ssr_on = false;

void setup() {
  pinMode(13, OUTPUT);
  pinMode(5 , OUTPUT);

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  
  Serial.begin(57600);
}

void loop() {
  if(sinceSSRMeasure > 1000) {
    measureSSRTemperature();
    sinceSSRMeasure = 0;
  }
  
  if(sinceOvenMeasure > 100) {
    measureOvenTemperature();
    sinceOvenMeasure = 0;
  }
  
  if(sinceSerialSend > 200) {
    Serial.print(oven_temperature,2);
    Serial.print(",");
    Serial.println(ssr_temperature,2);
    sinceSerialSend = 0;
  }
}

void measureSSRTemperature() {
  const float vcc = 3.33;
  const float rbias = 18.12;//kOhm
  const float B_NTC = 3950.0;

  const float vt = ((float)analogRead(A0)) / 1023.0 * vcc;
  const float resistance  = rbias * (vcc/vt - 1.0);

  ssr_temperature = (log(resistance/100.0/*kOhm*/)/log(2.718281))/B_NTC + 1.0/298.15;
  ssr_temperature = 1.0/ssr_temperature;
  ssr_temperature -= 273.15;

  if(ssr_temperature < -250.0) {
    ssr_ntc_disconnected = true;
    ssr_ntc_shorted = false;
  } else if(ssr_temperature > 600.0) {
    ssr_ntc_disconnected = false;
    ssr_ntc_shorted = true;
  } else {
    ssr_ntc_disconnected = false;
    ssr_ntc_shorted = false;
  }
}

void measureOvenTemperature() {
  const float vcc   = 3.33;
  const float rbias = 1.787;//kOhm
  const float B_NTC = 4160.0;

  const float vt = ((float)analogRead(A1)) / 1023.0 * vcc;
  const float resistance  = rbias * (vcc/vt - 1.0);

  oven_temperature = (log(resistance/100.0/*kOhm*/)/log(2.718281))/B_NTC + 1.0/298.15;
  oven_temperature = 1.0/oven_temperature;
  oven_temperature -= 273.15;

  if(oven_temperature < -250.0) {
    oven_ntc_disconnected = true;
    oven_ntc_shorted = false;
  } else if(oven_temperature > 600.0) {
    oven_ntc_disconnected = false;
    oven_ntc_shorted = true;
  } else {
    oven_ntc_disconnected = false;
    oven_ntc_shorted = false;
  }
}
