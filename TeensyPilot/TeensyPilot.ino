#include <i2c_t3.h>
#include <SFE_BMP180.h>

//*BMP180*/

#define TASK_SETUP    0
#define TASK_ACTIVE   1
#define TASK_INACTIVE 2

SFE_BMP180 bmp180;

double t, p, p0, a0, height_bmp180;

byte bmp180_task_t = TASK_INACTIVE;
byte bmp180_task_p = TASK_INACTIVE;

unsigned int  bmp180_wait_t;
elapsedMillis bmp180_wait_elapsed;

//*MPU6050*/

#include <mpu6050.h>

MPU6050 mpu6050(100);

//*SERVO*/

#include <Servo.h>


//EMAX Servo
const  int8_t offset_emax   = +35;
const uint8_t servomid_emax = (uint8_t)(offset_emax + 90);
  
const uint8_t servomin_emax = servomid_emax - 35;
const uint8_t servomax_emax = servomid_emax + 35;
  
//ES 07 Servo
const  int8_t offset_es07   = -7;
const uint8_t servomid_es07 = (uint8_t)(offset_es07 + 90);
  
const uint8_t servomin_es07 = servomid_es07 - 45;
const uint8_t servomax_es07 = servomid_es07 + 45;
  

Servo servo0;
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;  

elapsedMillis sinceServoWrite;

//*NEO6M*/

#include <neo6m.h>

NEO6M gps;
elapsedMillis sincePrint;
  
//*RFM69*/

#include <RFM69.h>
#include <RFM69registers.h>

RFM69 rfm69(15, 2, true, 2);
bool  rfm69_initialized = false;

PacketTransmitter packetTransmitter;
PacketPlane       packetPlane;

elapsedMillis sinceCalibration;
elapsedMillis sinceLastMessage;

//*BATTERY VOLTAGE*/

float battery_voltage = 0.0;

void setup() {
  delay(10); //Wait for all sensors to boot

  //Serial.begin(115200);

  setup_rfm69();

  mpu6050.setup();

  setup_bmp180();

  gps.setup();

  servo0.attach(35);
  servo1.attach(36);
  servo2.attach(37);
  servo3.attach(38);
  servo4.attach(30);
  servo5.attach(29);
  servo6.attach(16);

  pinMode(A15, INPUT);
  analogReference(0);
  analogReadRes(12);
}

void loop() {
  update_rfm69();
  
  mpu6050.update();

  gps.update();

  update_bmp180();

  if(sinceServoWrite > 10) {
    sinceServoWrite = 0;
    servo0.write(map(packetTransmitter.ch0, 0, 255, servomin_emax, servomax_emax));// Throttle
    servo1.write(map(packetTransmitter.ch1 + 16, 255, 0, servomin_emax, servomax_emax));// Rudder
    
    servo2.write(map(packetTransmitter.ch2, 255, 0, servomin_emax, servomax_emax));// Aileron Left?
    servo3.write(map(packetTransmitter.ch2, 255, 0, servomin_emax, servomax_emax));// Aileron Right?
    
    servo4.write(map(packetTransmitter.ch3 + 16, 0, 255, servomin_emax, servomax_emax));// Elevator

    servo5.write(map(packetTransmitter.ch4, 0, 255, servomin_es07, servomax_es07));// Camera
    servo6.write(map(packetTransmitter.ch4, 255, 0, servomin_es07, servomax_es07));// Camera Inverted
  
    battery_voltage = 0.95*battery_voltage + 0.05*(((float)analogRead(A15))*17.97/4096.0*1.0214);
  }

  if (sincePrint > 250) {
    sincePrint = 0;
    /*Serial.print("ch0=");
    Serial.print(packetTransmitter.ch0);
    Serial.print("(thr), ch1=");
    Serial.print(packetTransmitter.ch1);
    Serial.print("(rud), ch2=");
    Serial.print(packetTransmitter.ch2);
    Serial.print("(ail), ch3=");
    Serial.print(packetTransmitter.ch3);
    Serial.println("(ele)");*/
    /*Serial.print("w=");
    Serial.print(gps.ANGLE);
    Serial.print(" / w_home=");
    Serial.print(gps.HOME_ANGLE);
    Serial.print(" / w_diff=");
    Serial.print(gps.HOME_ANGLE_D);
    Serial.print(" / dist=");
    Serial.print(gps.HOME_DIST);
    Serial.print(" / gesch=");
    Serial.print(gps.SPEED);
    Serial.print(" / acc=");
    Serial.print(gps.H_ACCURACY);
    Serial.print(" / voltage=");
    Serial.print(battery_voltage);
    Serial.println("v");*/
  }
}

/* ------- */
/* BMP 180 */
/* ------- */

void setup_bmp180() {
  bmp180.begin();

  double pSum = 0;

  for (byte i = 0; i < 16; i++) {
    delay(bmp180.startTemperature());
    bmp180.getTemperature(t);
    delay(bmp180.startPressure(3));
    bmp180.getPressure(p0, t);
    pSum += p0;
  }

  p = p0 = pSum / 16.0;

  a0 = bmp180.altitude(p0, p0);

  /*Serial.print("A0: ");
  Serial.print(a0);
  Serial.println("m");

  Serial.print("T0: ");
  Serial.print(t);
  Serial.println("c");*/

  bmp180_wait_t = 0;
  bmp180_task_t = TASK_SETUP;
  bmp180_task_p = TASK_INACTIVE;
}

void update_bmp180() {
  if (bmp180_task_t == TASK_SETUP) {

    bmp180_wait_t = bmp180.startTemperature();
    if (bmp180_wait_t == 0) {
      //Serial.println("Error while starting temp measurement");
    }

    bmp180_task_t = TASK_ACTIVE;

    bmp180_wait_elapsed = 0;

  } else if (bmp180_task_t == TASK_ACTIVE && bmp180_wait_elapsed >= bmp180_wait_t ) {


    bmp180.getTemperature(t);
    bmp180_wait_t = bmp180.startPressure(3);

    if (bmp180_wait_t == 0) {
      //Serial.println("Error while starting pressure measurement");
    }

    bmp180_task_t = TASK_INACTIVE;
    bmp180_task_p = TASK_ACTIVE;

    bmp180_wait_elapsed = 0;
  }

  if (bmp180_task_p == TASK_ACTIVE && bmp180_wait_elapsed >= bmp180_wait_t ) {
    double pN;
    bmp180.getPressure(pN, t);

    p = 0.9 * p + 0.1 * pN;

    height_bmp180 = bmp180.altitude(p, p0) - a0;

    /*Serial.print("A: ");
      Serial.print(alt,1);
      Serial.println("m");*/

    bmp180_task_t = TASK_SETUP;
    bmp180_task_p = TASK_INACTIVE;
  }
}

void setup_rfm69() {
  if (rfm69.initialize(RF69_433MHZ, RX_RFM69, NETWORK_RFM69)) {
    if (rfm69.readReg(REG_SYNCVALUE2) == NETWORK_RFM69) {
      rfm69_initialized = true;

      rfm69.setHighPower();
      rfm69.setPowerLevel(31);

      rfm69.writeReg(REG_BITRATEMSB, RF_BITRATEMSB_19200);
      rfm69.writeReg(REG_BITRATELSB, RF_BITRATELSB_19200);

      sinceCalibration = 0;
      sinceLastMessage = 0;
    }
  }
}

void update_rfm69() {
  if (rfm69_initialized) {
    if(rfm69.receiveDone()) {
      if(rfm69.DATALEN == sizeof(PacketTransmitter)) {
        sinceLastMessage = 0;
        
        packetTransmitter = *(PacketTransmitter*)rfm69.DATA;

        packetPlane.rssi          = (int8_t)(rfm69.RSSI);
        
        packetPlane.voltage       = (uint16_t)(constrain(battery_voltage, 0.0, 65.0)*1000.0);
        
        packetPlane.distance      = (uint16_t)constrain(gps.HOME_DIST, 0.0, 500000.0);
        packetPlane.height        = (uint16_t)constrain(height_bmp180, 0.0, 5000.0);
        packetPlane.speed         = (uint8_t)constrain(gps.SPEED, 0.0, 200.0);
        packetPlane.home_angle_d  = (int8_t)(gps.HOME_ANGLE_D/2.0);

        packetPlane.pitch         = (int8_t)mpu6050.PITCH;
        packetPlane.roll          = (int8_t)mpu6050.ROLL;


        //uint32_t t_rfm = micros();
        
        rfm69.send(TX_RFM69, (const void*)(&packetPlane), sizeof(PacketPlane));
        
        /*Serial.print("Sending took ");
        Serial.print((micros() - t_rfm));
        Serial.println("us");*/
        
        if(sinceCalibration > 10000) {
          sinceCalibration = 0;

          rfm69.rcCalibration();
        }
        
        rfm69.receiveDone();
      }
    }
    if(sinceLastMessage >= 500) {
      
      sinceLastMessage = 500;
      
      packetTransmitter.ch0 = 16;
      packetTransmitter.ch1 = 127+16;
      packetTransmitter.ch2 = 127+16;
      packetTransmitter.ch3 = 127;
    }
  }
}
