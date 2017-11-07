#include <elapsedMillis.h>

elapsedMillis sinceSSRMeasure;
elapsedMillis sinceSSRPWM;
elapsedMillis sinceOvenMeasure;
elapsedMillis sinceSerialSend;

elapsedMillis sinceStateUpdate;
elapsedMillis sincePID;

elapsedMillis sinceBuzzer;

float ssr_temperature  = 0;
float oven_temperature = 0;

boolean oven_ntc_disconnected = false;
boolean ssr_ntc_disconnected  = false;

boolean oven_ntc_shorted = false;
boolean ssr_ntc_shorted  = false;

boolean  ssr_on            = false;
uint32_t ssr_pwm_dutycycle = 0;
uint32_t ssr_pwm_period    = 800;

float   oven_rise_per_second            = 1.25; //degrees/second for long heat burst
float   oven_responsetime               = 10.0; //time until pwm change affects temperature
float   oven_precutofftime              = oven_rise_per_second*oven_responsetime; //time until pwm change affects temperature
float   oven_temperature_target         = 0;
boolean oven_temperature_target_reached = false;

float error_old = 0.0;
float error_derivative_old = 0.0;
float error_time_last_change = 0.0;

#define STATE_IDLE     0
#define STATE_TO_SOAK  1
#define STATE_SOAK     2
#define STATE_REFLOW   3
#define STATE_COOL     4

int state = STATE_IDLE;


void setup() {
  pinMode(3 , OUTPUT);
  pinMode(5 , OUTPUT);
  pinMode(13, OUTPUT);

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  
  Serial.begin(57600);
  Serial.setTimeout(2);
}

void loop() {
  if(sinceBuzzer > 1000) {
    tone(3,2048,100);
    sinceBuzzer = 0;
  }
  
  if(sinceSSRMeasure > 1000) {
    measureSSRTemperature();
    sinceSSRMeasure = 0;
  }
  
  if(sinceOvenMeasure > 100) {
    measureOvenTemperature();
    sinceOvenMeasure = 0;
  }
  
  if(sinceSerialSend > 1000) {
    Serial.print("Oven: ");
    Serial.print(oven_temperature,2);
    Serial.print(", SSR: ");
    Serial.print(ssr_temperature,2);
    Serial.print(", Oven PWM: ");
    Serial.println(ssr_pwm_dutycycle);
      
    sinceSerialSend = 0;
  }


  uint32_t pid_interval = 1000;
  //PID START
  if(sincePID > pid_interval) {
    sincePID = 0;
    
    const float kP = 7.5;
    const float kI = 0.0;
    const float kD = 80.0;
  
    int baseDutyCycle = (getHoldDutyCyclefromTemperature(oven_temperature_target) * 75) / 100;
    
    float error = oven_temperature_target - oven_temperature;
    float error_derivative = (error - error_old)*(1000.0/(float)pid_interval);

    //if not enough change do not update derivative error 
    if(abs(error_derivative) < 0.01 && error_time_last_change < 5.0) { 
      error_time_last_change += ((float)pid_interval)/1000.0;
      error_derivative = error_derivative_old;
    } else { //if enough change and error has been accumulated
      error_derivative_old = error_derivative = error_derivative/error_time_last_change;
      error_time_last_change = 1.0;
    }
    
    error_old = error;
    
    Serial.print("P: ");
    Serial.print(kP * error, 2);
    Serial.print(", I: 0.00, D: ");
    Serial.println(kD * error_derivative, 2);
    
    float output = kP * error /*+ kI * integral(error)*/ + kD * error_derivative + 0.5;
    
    
    int dutyCycle = (baseDutyCycle + (int)output);
    setDutyCycle(dutyCycle);
  }
  //PID END

  if(sinceSSRPWM > ssr_pwm_period) {
    sinceSSRPWM = 0;
    if(!ssr_on && ssr_pwm_dutycycle > 0) {
      digitalWrite(5, ssr_on = HIGH);
    }
  } else if(sinceSSRPWM > (ssr_pwm_period * ssr_pwm_dutycycle) / 100) {
    if(ssr_on)
      digitalWrite(5, ssr_on = LOW);
  }

  if(Serial.available()){
    setTemperatureTarget((float)Serial.parseInt());
    Serial.print("Oven target temperature: ");
    Serial.println(oven_temperature_target);
  }

  /*if(sinceStateUpdate > 1000) {
    switch(state) {
      case STATE_IDLE:
        
      break;
      
      case STATE_TO_SOAK:
        
      break;
  
      case STATE_SOAK:
        
      break;
  
      case STATE_REFLOW:
        
      break;
  
      case STATE_COOL:
        
      break;
  
      default:
      break;
    };
  }*/

  //Add better safeguard!
  /*while(oven_ntc_disconnected || oven_ntc_shorted) {
    digitalWrite(5, ssr_on = LOW);
    if(oven_ntc_shorted)
      Serial.println("Thermistor shorted!");
    if(oven_ntc_disconnected)
      Serial.println("Thermistor disconnected!");
    delay(1000);
    measureOvenTemperature();
  }*/
}

uint32_t getHoldDutyCyclefromTemperature(float temperature){
  //https://www.mycurvefit.com/
  //  T , PWM
  //----------
  //  0 , 50
  //  15, 100
  //  25, 150
  //  35, 200
  //  45, 220
  temperature = constrain(temperature, 20, 270);
  return   (uint32_t)(
         +  0.5 //rounding
         -  1.776357/100000000000000.0
         +  0.3646995     * temperature
         - (0.0008960402  * temperature)* temperature
         - (0.00001052457 * temperature)*(temperature  *  temperature)
         + (5.131143/100000000.0 *(temperature * temperature))*(temperature*temperature)
         );
}

void setTemperatureTarget(float temperature) {
  oven_temperature_target = temperature;
  oven_temperature_target_reached = isTemperatureReached();
}

boolean isTemperatureReached() {
  return oven_temperature_target < oven_temperature;
}

void setDutyCycle(int dutycycle) {
  ssr_pwm_dutycycle = (uint32_t)constrain(dutycycle,0,100);
}

void measureSSRTemperature() {
  const float vcc = 3.33;
  const float rbias = 18.12;//kOhm
  const float B_NTC = 3950.0;

  const int   A0_value = analogRead(A0);
  const float vt = ((float)(A0_value == 0 ? 1 : A0_value)) / 1023.0 * vcc;
  const float resistance  = rbias * (vcc/vt - 1.0);

  ssr_temperature = (log(resistance/100.0/*kOhm*/)/log(2.718281))/B_NTC + 1.0/298.15;
  ssr_temperature = 1.0/ssr_temperature;
  ssr_temperature -= 273.15;

  if(ssr_temperature < -273.0) {
    ssr_ntc_disconnected = false;
    ssr_ntc_shorted = true;
  } else if(ssr_temperature < -20.0) {
    ssr_ntc_disconnected = true;
    ssr_ntc_shorted = false;
  } else {
    ssr_ntc_disconnected = false;
    ssr_ntc_shorted = false;
  }
}

void measureOvenTemperature() {
  const float vcc   = 3.33;
  const float rbias = 1.787;//kOhm
  const float B_NTC = 4160.0;

  const int   A1_value = analogRead(A1);
  const float vt = ((float)(A1_value == 0 ? 1 : A1_value)) / 1023.0 * vcc;
  const float resistance  = rbias * (vcc/vt - 1.0);

  oven_temperature = (log(resistance/100.0/*kOhm*/)/log(2.718281))/B_NTC + 1.0/298.15;
  oven_temperature = 1.0/oven_temperature;
  oven_temperature -= 273.15;

  if(oven_temperature < -273.0) {
    oven_ntc_disconnected = false;
    oven_ntc_shorted = true;
  } else if(oven_temperature < -20.0) {
    oven_ntc_disconnected = true;
    oven_ntc_shorted = false;
  } else {
    oven_ntc_disconnected = false;
    oven_ntc_shorted = false;
  }
}
