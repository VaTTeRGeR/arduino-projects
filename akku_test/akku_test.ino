#include <elapsedMillis.h>

#include <SPI.h>

#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

#define RELAIS 6

Adafruit_PCD8544 pcd = Adafruit_PCD8544(9,8,7);

elapsedMillis since_refresh;

float offset = 2.494; //Voltage out when relais on, 2.515 if off
float amphours  = 0.0;
float watthours = 0.0;
float temperature = 0.0;

const uint32_t  measure_period = 1000;

float time_total = 0.0;

bool finished = false;
float voltage_avg = 0.0;

void setup() {
  pcd.begin();
  pcd.setContrast(61);

  pinMode(A0, INPUT);
  pinMode(A6, INPUT);
  pinMode(A7, INPUT);

  voltage_avg = readVoltage();

  if(voltage_avg < 13.0) {
    finished = true;
    relaisOff();
  } else {
    finished = false;
    relaisOn();
  }
}

void loop() {
  lcdBiasByTemperature();
  if(finished) {
      pcd.clearDisplay();
      
      pcd.setTextSize(1);
      pcd.setCursor(0,0);
      
      pcd.println("FERTIG!");
      
      pcd.print("AH: ");
      pcd.println(amphours,2);
  
      pcd.print("WH: ");
      pcd.println(watthours,2);
  
      pcd.print("Zeit: ");
      pcd.print((uint8_t)(time_total));
      pcd.print(":");
      uint8_t minutes = (uint8_t)(fmod(time_total,1.0)*60.0);
      if(minutes < 10) {
        pcd.print(0);
      }
      pcd.println(minutes);

      pcd.print("T/B:");
      pcd.print(temperature,1);
      pcd.print("/");
      pcd.print(60+max(0, map(temperature,20,10,0,5)));
  
      pcd.display();

      delay(1000);
  } else {
    if(since_refresh >= measure_period) {
      since_refresh -= measure_period;
  
      float voltage = readVoltage();
      float current = readCurrent();
  
      float time_hours = ((float)measure_period)/(1000.0*60.0*60.0);
      time_total += time_hours;
      
      amphours  += current*time_hours;
      watthours += (current*voltage)*time_hours;

      voltage_avg = voltage_avg * 0.5 + voltage * 0.5;

      if(voltage_avg < 11.5) {
        finished = true;
        relaisOff();
      }
  
      pcd.clearDisplay();
  
      pcd.setTextSize(1);
      pcd.setCursor(0,0);
      
      pcd.print("Volt: ");
      pcd.println(voltage,2);
      
      pcd.print("Strom: ");
      pcd.println(current,2);
  
      pcd.print("AH: ");
      pcd.println(max(0.0, amphours),2);
  
      pcd.print("WH: ");
      pcd.println(max(0.0, watthours),2);
  
      pcd.print("Zeit: ");
      pcd.print((uint8_t)(time_total));
      pcd.print(":");
      uint8_t minutes = (uint8_t)(fmod(time_total,1.0)*60.0);
      if(minutes < 10) {
        pcd.print(0);
      }
      pcd.print(minutes);
  
      pcd.display();
    }
  }
}

void lcdBiasByTemperature() {
  float vt = ((float)analogRead(A1))*(3.29/1023);
  temperature = ((3.29*9.85)/(3.29-vt))-9.85;
  
  temperature = (log(temperature/10.0)/log(2.718281))/4300.0 + 1.0/298.15;
  temperature = 1.0/temperature;
  
  temperature -= 273.15;

  pcd.setContrast(60+max(0, map(temperature,20,10,0,5)));
}

void relaisOn() {
  pinMode(RELAIS, OUTPUT);
  digitalWrite(RELAIS, LOW);
}

void relaisOff() {
  pinMode(RELAIS, INPUT);
  digitalWrite(RELAIS, LOW);
}

float readVoltage() {
  return ((float)analogRead(A6))/1023.0*3.3/(4.61/(46.56+4.61));
}

float readCurrentVoltage() {
  return ((float)analogRead(A7))/1023.0*3.3/(22.0/(12.0+22.0));
}

float readCurrent() {
  return abs(offset - readCurrentVoltage()*(2.494/2.48))*10.0;
}
