 //#include <avr/wdt.h>

/* --- */

#include <SPI.h>
#include <RFM69.h>
#include <RFM69registers.h>
#include "radio_constants.h"
#include "radio_packets.h"

/* --- */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "servo_constants.h"

/* --- */

#include "mpu6050_constants.h"

/* --- */

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

/* --- */

RFM69       radio;

PacketRSSI  packetRSSI;
Packet      packet;

boolean     radio_ready = false;


/* --- */


long    gyro_x_cal, gyro_y_cal, gyro_z_cal;

int     gyro_x, gyro_y, gyro_z;
long    acc_x, acc_y, acc_z;

long    acc_total_vector;
long    still_length_acc;

float   angle_pitch, angle_roll;
float   angle_roll_acc, angle_pitch_acc;
int     angle_pitch_buffer, angle_roll_buffer;

float   angle_pitch_output, angle_roll_output;

int     temperature;
long    loop_timer;
boolean gyro_angles_set;

/* - SETUP - */

void setup() {
  //wdt_enable(WDTO_1S);
  
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  for(int i = 0; i<10;i++){
    digitalWrite(13, HIGH);
    delay(25);
    digitalWrite(13, LOW);
    delay(25);
  }

  //setupPWM();
  
  //setup_mpu_6050();
  
  //calibrate_mpu_6050();

  setupRadio();
  
  //wdt_reset();
  //wdt_enable(WDTO_15MS);
  
  loop_timer = micros();
}

void loop() {
  //updateMPU();
  
  updateRadio();
  
  while(micros() - loop_timer < 1000000/SAMPLERATE);
  loop_timer = micros();

  //wdt_reset();
}

/* --- */

void setupRadio(){
  if(radio.initialize(RF69_433MHZ, RX_RFM69, NETWORK_RFM69)) {
    if(radio.readReg(REG_SYNCVALUE2) == NETWORK_RFM69) {
      radio_ready = true;
    
      radio.setPowerLevel(0);
  
      radio.writeReg(REG_BITRATEMSB, RF_BITRATEMSB_9600);
      radio.writeReg(REG_BITRATELSB, RF_BITRATELSB_9600);
    }
  }
}

void updateRadio(){
  if(radio_ready && radio.receiveDone()) {
    if(radio.DATALEN == sizeof(Packet)) {
      packet = *(Packet*)radio.DATA;
      if(packet.flags == 0xAB) {
        packetRSSI.rssi = radio.RSSI;
        radio.send(TX_RFM69, (const void*)(&packetRSSI), sizeof(packetRSSI));
      } else {
      }
    }
  }
}

/* --- */

void setupPWM(){
  pwm.begin();
  pwm.setPWMFreq(SERVOFREQ);
  setThrottle(0, 0);
}

void setThrottle(byte pin, unsigned char percent){
  percent = min(percent, 100);
  percent = max(percent, 0);
  pwm.setPWM(pin, 0, map(percent, 0, 100, ESCMIN, ESCMAX));
}

void setAngle(byte pin, char angle){
  angle = min(angle, 45);
  angle = max(angle, -45);
  pwm.setPWM(pin, 0, map(angle, -45, 45, SERVOMIN, SERVOMAX));
}

/* --- */

void updateMPU(){

  read_mpu_6050_data();                                                //Read the raw acc and gyro data from the MPU-6050

  gyro_x -= gyro_x_cal;                                                //Subtract the offset calibration value from the raw gyro_x value
  gyro_y -= gyro_y_cal;                                                //Subtract the offset calibration value from the raw gyro_y value
  gyro_z -= gyro_z_cal;                                                //Subtract the offset calibration value from the raw gyro_z value

  //float gyroToAngleChange = 1.0/((float)SAMPLERATE)/65.5;            // for 500 deg/s
  float gyroToAngleChange = 1.0/((float)SAMPLERATE)/16.375;            // for 2000 deg/s

  //Gyro angle calculations
  angle_pitch += ((float)gyro_x) * gyroToAngleChange;                  //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll += ((float)gyro_y) * gyroToAngleChange;                   //Calculate the traveled roll angle and add this to the angle_roll variable
  
  float gyrozScl = gyroToAngleChange * (PI / 180.0);                   //The Arduino sin function is in radians
  angle_pitch += angle_roll * sin(gyro_z * gyrozScl);                  //If the IMU has yawed transfer the roll angle to the pitch angel
  angle_roll -= angle_pitch * sin(gyro_z * gyrozScl);                  //If the IMU has yawed transfer the pitch angle to the roll angel
  
  //Accelerometer angle calculations
  acc_total_vector = sqrt( (acc_x*acc_x) + (acc_y*acc_y) + (acc_z*acc_z) ); //Calculate the total accelerometer vector
  //57.296 = 1.0 / (3.142 / 180.0) The Arduino asin function is in radians
  angle_pitch_acc = asin((float)acc_y / acc_total_vector)*  57.296;    //Calculate the pitch angle
  angle_roll_acc = asin((float)acc_x / acc_total_vector) * -57.296;    //Calculate the roll angle
  
  //Offset
  //angle_pitch_acc -= 0.0;                                              //Accelerometer calibration value for pitch
  //angle_roll_acc -= 0.0;                                               //Accelerometer calibration value for roll

  if(!gyro_angles_set){                                                //If the IMU is not already started
    angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle 
    angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle 
    gyro_angles_set = true;                                            //Set the IMU started flag
  } else if((((float)still_length_acc) * 1.025) > ((float)acc_total_vector)
         && (((float)still_length_acc) * 0.975) < ((float)acc_total_vector)) {
    angle_pitch = angle_pitch * 0.98 + angle_pitch_acc * 0.02;       //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_roll = angle_roll * 0.98 + angle_roll_acc * 0.02;          //Correct the drift of the gyro roll angle with the accelerometer roll angle
  }

  
  //To dampen the pitch and roll angles a complementary filter is used
  //angle_pitch_output = angle_pitch_output * 0.75 + angle_pitch * 0.25;   //Take 75% of the output pitch value and add 25% of the raw pitch value
  //angle_roll_output = angle_roll_output * 0.75 + angle_roll * 0.25;      //Take 75% of the output roll value and add 25% of the raw roll value

  angle_pitch_output = angle_pitch;
  angle_roll_output = angle_roll;
  
  setAngle(15, angle_roll_output/2);
  setAngle(11, angle_pitch_output/2);
  setAngle(7, 45);
  setAngle(6, 0);
  setAngle(5, -45);
}

void read_mpu_6050_data(){                                             //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68,14);                                           //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);                                        //Wait until all the bytes are received
  
  //READS IN THIS ORDER: AX, AY, AZ | GX, GY, GZ
  
  acc_z = (Wire.read()<<8|Wire.read());                                  //Add the low and high byte to the acc_x variable
  acc_y = (Wire.read()<<8|Wire.read());                                  //Add the low and high byte to the acc_y variable
  acc_x = -(Wire.read()<<8|Wire.read());                                  //Add the low and high byte to the acc_z variable
  temperature = Wire.read()<<8|Wire.read();                            //Add the low and high byte to the temperature variable
  gyro_z = (Wire.read()<<8|Wire.read());                                 //Add the low and high byte to the gyro_x variable
  gyro_y = (Wire.read()<<8|Wire.read());                                 //Add the low and high byte to the gyro_y variable
  gyro_x = -(Wire.read()<<8|Wire.read());                                 //Add the low and high byte to the gyro_z variable

}

void setup_mpu_6050(){
  Wire.begin();                                                        //Start I2C as master
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x10);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the gyro (2000dps full scale)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x18);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
}

void calibrate_mpu_6050(){
  
  int cal_rounds = 100;
  
  for (int cal_int = 0; cal_int < cal_rounds ; cal_int ++){                  //Run this code 2000 times
    read_mpu_6050_data();                                              //Read the raw acc and gyro data from the MPU-6050
    
    gyro_x_cal += gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z;                                              //Add the gyro z-axis offset to the gyro_z_cal variable

    still_length_acc += sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));

    delay(5);
    
    //wdt_reset();
  }
  
  gyro_x_cal /= cal_rounds;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyro_y_cal /= cal_rounds;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gyro_z_cal /= cal_rounds;                                                  //Divide the gyro_z_cal variable by 2000 to get the avarage offset

  still_length_acc /= cal_rounds;
}

/* --- */

void wait(long t_wait) {
  for(long t_now = millis(); millis() < t_now + t_wait; ){}
}

