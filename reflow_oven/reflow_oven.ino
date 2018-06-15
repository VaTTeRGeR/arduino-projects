#include <elapsedMillis.h>
#include <SPI.h>

#include <U8g2lib.h>

elapsedMillis sinceDraw;
elapsedMillis sinceGraphUpdate;

elapsedMillis sinceBuzzer;

elapsedMillis sinceInput;

elapsedMillis sinceSSRMeasure;
elapsedMillis sinceSSRPWM;
elapsedMillis sinceOvenMeasure;
elapsedMillis sinceSerialSend;

elapsedMillis sinceStateUpdate;
elapsedMillis sincePID;

float ssr_temperature  = 0;
float oven_temperature = 0;

boolean oven_ntc_disconnected = false;
boolean ssr_ntc_disconnected  = false;

boolean oven_ntc_shorted = false;
boolean ssr_ntc_shorted  = false;

boolean  ssr_on            = false;
uint32_t ssr_pwm_dutycycle = 0;
uint32_t ssr_pwm_period    = 800;

float   oven_temperature_target         = 0.0;
boolean oven_temperature_target_reached = false;

float error_old = 0.0;
float error_derivative_old = 0.0;
float error_time_last_change = 0.0;

float error_integral = 0.0;

#define STATE_IDLE     0
#define STATE_TO_SOAK  1
#define STATE_SOAK     2
#define STATE_REFLOW   3
#define STATE_COOL     4

int state = STATE_IDLE;

U8G2_ST7920_128X64_2_HW_SPI u8g2(U8G2_R2, /* CS=*/ 10, /* reset=*/ U8X8_PIN_NONE);

boolean buzzer_on = false;

uint16_t button = 1023;
uint16_t joy_x  = 512;
uint16_t joy_y  = 512;

const uint8_t temps_size = 110;
uint8_t temps[temps_size];
uint8_t temps_head = 0;

void setup() {
  pinMode(3 , OUTPUT); //buzzer control
  pinMode(5 , OUTPUT); //ssr control

  pinMode(A0, INPUT); //temp_ssr
  pinMode(A1, INPUT); //temp_oven

  pinMode(16, INPUT); //button Pressed   = 0
  pinMode(17, INPUT); //joy_x  UpperLeft = 0
  pinMode(27, INPUT); //joy_y  UpperLeft = 0
  
  Serial.begin(57600);
  Serial.setTimeout(2);

  u8g2.begin();
  u8g2.setFont(u8g2_font_micro_tr);//height of 6 pixels

  for(uint8_t i = 0; i<temps_size; i++){
    temps[i] = 0;
  }
}

uint8_t temp_dummy = 0;

void loop() {
  uint16_t button = analogRead(16);
  uint16_t joy_x  = analogRead(17);
  uint16_t joy_y  = analogRead(27);
  
  if(sinceGraphUpdate > 500) {
    sinceGraphUpdate = 0;
    
    if(temps_head == temps_size-1) {
      temps_head = 0;
    } else {
      temps_head++;
    }

    temps[temps_head] = (uint8_t)constrain(oven_temperature,50.0,250.0);
  }

  if(sinceDraw > 250) {
    sinceDraw = 0;
    

    //u8g2.clearDisplay();

    uint32_t t = millis();
    
    u8g2.firstPage();
    do {
      u8g2.setCursor(0,6);
      u8g2.print("T:");
      u8g2.print(oven_temperature,0);
      u8g2.print("/");
      u8g2.print(oven_temperature_target,0);

      u8g2.setCursor(5*12,6);
      u8g2.print("T_SSR:");
      u8g2.print(ssr_temperature,0);

      u8g2.drawFrame(96,0,32,8);
      u8g2.drawBox(98,2,(28*ssr_pwm_dutycycle)/100,4);
      
      u8g2.drawHLine(0,7,128);
      u8g2.drawHLine(0,7*3,128);
      
      u8g2.setCursor(0,7*4);
      u8g2.print("250");

      u8g2.setCursor(0,64);
      u8g2.print("50");
      
      u8g2.drawFrame(128-(temps_size+2),7*3,temps_size+2,64-7*3);

      uint8_t temps_index;
      if(temps_head == temps_size - 1) {
        temps_index = 0;
      } else {
        temps_index = temps_head + 1;
      }

      uint8_t temps_drawn = 0;
      
      while(temps_drawn < temps_size) {
        u8g2.drawPixel(temps_drawn+128-(temps_size+2)+1, 62 - ((temps[temps_index]-50)*(64-7*3-2))/200);

        temps_index++;
        if(temps_index == temps_size) {
          temps_index = 0;
        }
        
        temps_drawn++;
      }
            
    } while (u8g2.nextPage());

    Serial.print("t_render: ");
    Serial.println(millis()-t);

  }
  
  if(sinceBuzzer > 500 && buzzer_on) {
    tone(3,2048,100);
    sinceBuzzer = 0;
  }

  if(sinceInput > 250) {
    sinceInput = 0;

    if(button < 512) {
      tone(3,1536,5);
    }
    
    if(joy_x < 256) {
      tone(3,1536,5);
      oven_temperature_target -= 5.0;
    }
    
    if(joy_x > 768) {
      tone(3,1536,5);
      oven_temperature_target += 5.0;
    }
    
    if(joy_y < 256) {
      tone(3,1536,5);
      oven_temperature_target += 10.0;
    }
    
    if(joy_y > 768) {
      tone(3,1536,5);
      oven_temperature_target -= 10.0;
    }
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


  const uint32_t pid_interval = 1000;
  //PID START
  if(sincePID > pid_interval) {
    sincePID = 0;
    
    const float kP = 20;//7.5;
    const float kI = 25;//0.0;
    const float kD = 0;//80.0;
  
    int baseDutyCycle = (getHoldDutyCyclefromTemperature(oven_temperature_target) * 75) / 100;
    
    float error = oven_temperature_target - oven_temperature;
    error_integral = error_integral + error*(1000.0/(float)pid_interval);
    error_integral = constrain(error_integral, -5.0, 5.0);
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
    Serial.print(", I: ");
    Serial.print(kI * error_integral, 2);
    Serial.print(", D: ");
    Serial.println(kD * error_derivative, 2);
    
    float output = kP * error + kI * error_integral + kD * error_derivative + 0.5;
    
    
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
  //  A curve fit to experimentally gained values.
  //  https://www.mycurvefit.com/
  //  PWM , T
  // ---------
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
  const float B_NTC = 3930.0;//B20/60

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
