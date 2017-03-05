#include <RFM69.h>
#include <RFM69registers.h>

#include <i2c_t3.h>
#include <SFE_BMP180.h>

#define TASK_SETUP    0
#define TASK_ACTIVE   1
#define TASK_INACTIVE 2

//*BMP180*/

SFE_BMP180 bmp180;

double t,p,p0,a0;

byte bmp180_task_t = TASK_INACTIVE;
byte bmp180_task_p = TASK_INACTIVE;

unsigned int  bmp180_wait_t;
elapsedMillis bmp180_wait_elapsed;

//*MPU6050*/

#include <mpu6050.h>

MPU6050 mpu6050(100);

//*SERVO*/

#include <Servo.h>

float offset = 33.0;

Servo servo0;
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;

//*NEO6M*/

#include <neo6m.h>

NEO6M gps;

//*RFM69*/

#define NETWORK_RFM69 100

#define TX_RFM69 1
#define RX_RFM69 2

#define TX_PER_SECOND 30

#define SIGNAL_WARN_LB -85
#define SIGNAL_WARN_UB -110
#define SIGNAL_CHECK_SPACING 15

RFM69 rfm69(15, 2, true, 0);
bool  rfm69_initialized = false;

void setup() {
  delay(10); //Wait for all sensors to boot

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
}

void loop(){
  mpu6050.update();

  gps.update();
  
  update_bmp180();
  
  servo0.write(-mpu6050.ROLL + offset + 90);
  servo1.write( mpu6050.PITCH + offset + 90);
  servo2.write(offset + 90);
  servo3.write(offset + 90);
  servo4.write(offset + 90);
  servo5.write(offset + 90);
  servo6.write(offset + 90);
}

/* ------- */
/* BMP 180 */
/* ------- */

void setup_bmp180(){
  bmp180.begin();
  
  double pSum = 0;

  for(byte i = 0; i < 16; i++) {
    delay(bmp180.startTemperature());
    bmp180.getTemperature(t);
    delay(bmp180.startPressure(3));
    bmp180.getPressure(p0, t);
    pSum += p0;
  }

  p = p0 = pSum / 16.0;
  
  a0 = bmp180.altitude(p0,p0);
  
  Serial.print("A0: ");
  Serial.print(a0);
  Serial.println("m");

  Serial.print("T0: ");
  Serial.print(t);
  Serial.println("c");
  
  bmp180_wait_t = 0;
  bmp180_task_t = TASK_SETUP;
  bmp180_task_p = TASK_INACTIVE;
}

void update_bmp180(){
  if(bmp180_task_t == TASK_SETUP) {

    bmp180_wait_t = bmp180.startTemperature();
    if(bmp180_wait_t == 0) {
      Serial.println("Error while starting temp measurement");
    }
    
    bmp180_task_t = TASK_ACTIVE;

    bmp180_wait_elapsed = 0;
  
  } else if(bmp180_task_t == TASK_ACTIVE && bmp180_wait_elapsed >= bmp180_wait_t ) {
    
    
    bmp180.getTemperature(t);
    bmp180_wait_t = bmp180.startPressure(3);

    if(bmp180_wait_t == 0) {
      Serial.println("Error while starting pressure measurement");
    }
    
    bmp180_task_t = TASK_INACTIVE;
    bmp180_task_p = TASK_ACTIVE;
    
    bmp180_wait_elapsed = 0;
  }
  
  if(bmp180_task_p == TASK_ACTIVE && bmp180_wait_elapsed >= bmp180_wait_t ) {
    double pN;
    bmp180.getPressure(pN, t);
  
    p = 0.9*p + 0.1*pN;
  
    double alt = bmp180.altitude(p,p0)-a0;

    /*Serial.print("A: ");
    Serial.print(alt,1);
    Serial.println("m");*/

    bmp180_task_t = TASK_SETUP;
    bmp180_task_p = TASK_INACTIVE;
  }
}

void setup_rfm69() {
  if(rfm69.initialize(RF69_433MHZ, RX_RFM69, NETWORK_RFM69)) {
    if(rfm69.readReg(REG_SYNCVALUE2) == NETWORK_RFM69) {
      rfm69_initialized = true;

      rfm69.setHighPower();
      rfm69.setPowerLevel(29);
  
      rfm69.writeReg(REG_BITRATEMSB, RF_BITRATEMSB_57600);
      rfm69.writeReg(REG_BITRATELSB, RF_BITRATELSB_57600);
    }
  }
}
