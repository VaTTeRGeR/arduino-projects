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
#include "math3d.h"

/* --- */

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

/* --- */

RFM69       radio;

PacketRSSI  packetRSSI;
Packet      packet;

boolean     radio_ready = false;


/* --- */

unsigned long   arm_counter;
boolean         armed;


signed long    gyro_x_cal, gyro_y_cal, gyro_z_cal;

signed long    gyro_x, gyro_y, gyro_z;
signed long    acc_x, acc_y, acc_z;

signed long    acc_total_vector;
signed long    still_length_acc;

float   angle_pitch, angle_roll;
float   angle_roll_acc, angle_pitch_acc;
int     angle_pitch_buffer, angle_roll_buffer;

float   angle_pitch_output, angle_roll_output;

int     temperature;

long    loop_timer;

boolean gyro_angles_set;

/* - SETUP - */

void setup() {

  Serial.begin(115200);

  //wdt_enable(WDTO_1S);
  
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  for(int i = 0; i<10;i++){
    digitalWrite(13, HIGH);
    delay(25);
    digitalWrite(13, LOW);
    delay(25);
  }

  setupPWM();
  
  setup_mpu_6050();
  
  calibrate_mpu_6050();

  //setupRadio();
  
  //wdt_reset();
  //wdt_enable(WDTO_15MS);
  
  loop_timer = micros();
  
  arm_counter = millis();
  armed = false;
}

void loop() {
  
  read_mpu_6050_data();

  gyro_x -= gyro_x_cal;
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;

  Serial.write(64);
  Serial.write(63);
  Serial.write(62);

  Serial.write((byte *)(&gyro_x), 4);
  Serial.write((byte *)(&gyro_y), 4);
  Serial.write((byte *)(&gyro_z), 4);

  Serial.write((byte *)(&acc_x), 4);
  Serial.write((byte *)(&acc_y), 4);
  Serial.write((byte *)(&acc_z), 4);

  /*Serial.print(acc_x);
  Serial.print(',');
  Serial.print(acc_y);
  Serial.print(',');
  Serial.println(acc_z);
  Serial.flush();*/
  
  //updateMPU();
  
  //updateRadio();
  
  while(micros() - loop_timer < 1000000/SAMPLERATE);
  loop_timer = micros();

  //wdt_reset();
}

/* --- */

void setupRadio(){
  if(radio.initialize(RF69_433MHZ, RX_RFM69, NETWORK_RFM69)) {
    if(radio.readReg(REG_SYNCVALUE2) == NETWORK_RFM69) {
      radio_ready = true;
    
      radio.setPowerLevel(17);
      radio.setHighPower();
  
      radio.writeReg(REG_BITRATEMSB, RF_BITRATEMSB_19200);
      radio.writeReg(REG_BITRATELSB, RF_BITRATELSB_19200);
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
  setThrottle(THROTTLE_HEADER, 0);
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
    angle_pitch = angle_pitch * 0.95 + angle_pitch_acc * 0.05;       //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_roll = angle_roll * 0.95 + angle_roll_acc * 0.05;          //Correct the drift of the gyro roll angle with the accelerometer roll angle
  }

  
  //To dampen the pitch and roll angles a complementary filter is used
  //angle_pitch_output = angle_pitch_output * 0.75 + angle_pitch * 0.25;   //Take 75% of the output pitch value and add 25% of the raw pitch value
  //angle_roll_output = angle_roll_output * 0.75 + angle_roll * 0.25;      //Take 75% of the output roll value and add 25% of the raw roll value

  angle_pitch_output = angle_pitch;
  angle_roll_output = angle_roll;
  
  Serial.print("AA");
  Serial.write((byte)angle_roll_output);
  Serial.write((byte)angle_pitch_output);
  
  setAngle(15, angle_roll_output/2.0);
  setAngle(11, angle_pitch_output/2.0);
  setAngle(7, 45);
  setAngle(6, 0);
  setAngle(5, -45);
  
  if(abs(angle_roll_output) > 45 || abs(angle_pitch_output) > 45) {
    digitalWrite(13, HIGH);
  } else {
    digitalWrite(13, LOW);
  }
  
  if(!armed && millis() - arm_counter > 8000) {
    armed = true;
  }
  
  if(armed) {
    setThrottle(THROTTLE_HEADER, abs(angle_roll_output * 1.5));
  }
}

Quat rq;
Vec3 Vector3Z = Vector(0,0,1);

void updateMPUX() {

  unsigned long t0 = micros();
  
  read_mpu_6050_data();                                                //Read the raw acc and gyro data from the MPU-6050

  gyro_x -= gyro_x_cal;                                                //Subtract the offset calibration value from the raw gyro_x value
  gyro_y -= gyro_y_cal;                                                //Subtract the offset calibration value from the raw gyro_y value
  gyro_z -= gyro_z_cal;                                                //Subtract the offset calibration value from the raw gyro_z value

  float gscl = 1.0 / 16.375 * Q_DEGRAD;            // for 2000 deg/s setting (in rad/s)
  float ascl = 1.0 / 2048.0;                     // for 16 g setting

  Vec3 GyroVec = Vector(((float)gyro_x) * gscl, ((float)gyro_y) * gscl, ((float)gyro_z) * gscl);

  Vec3 Accel_Body = Vector(acc_x * ascl, acc_y * ascl, acc_z * ascl);

  Vec3 Accel_World = Rotate(rq, Accel_Body); 

  Vec3 correction_World = CrossProd(Accel_World, Vector3Z);

  Vec3 correction_Body = Rotate(correction_World, rq);

  GyroVec = Sum(GyroVec, correction_Body);

  Quat incrementalRotation = Quaternion(GyroVec, 1000000/SAMPLERATE);

  rq = Mul(incrementalRotation, rq);

  Vec3 YPR = YawPitchRoll(rq);
  angle_roll_output = -YPR.z * Q_RADDEG;
  angle_pitch_output = -YPR.y * Q_RADDEG;

  Serial.println(micros()-t0);
  
  setAngle(15, angle_pitch_output/2.0);
  setAngle(11, angle_roll_output/2.0);
  setAngle(7, 45);
  setAngle(6, 0);
  setAngle(5, -45);

  if(abs(angle_roll_output) > 45 || abs(angle_pitch_output) > 45) {
    digitalWrite(13, HIGH);
  } else {
    digitalWrite(13, LOW);
  }
  
  //Serial.print(angle_roll_output,3);
  //Serial.print(',');
  //Serial.print(angle_pitch_output,3);
  //Serial.println();
}

void read_mpu_6050_data(){                                             //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050

  Wire.requestFrom(0x68, 14);                                           //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);                                        //Wait until all the bytes are received
  
  //READS IN THIS ORDER: AX, AY, AZ | GX, GY, GZ
  
  acc_x = (Wire.read()<<8|Wire.read());                                  //Add the low and high byte to the acc_x variable
  acc_y = (Wire.read()<<8|Wire.read());                                  //Add the low and high byte to the acc_y variable
  acc_z = (Wire.read()<<8|Wire.read());                                  //Add the low and high byte to the acc_z variable

  temperature = Wire.read()<<8|Wire.read();                            //Add the low and high byte to the temperature variable
  
  gyro_x = (Wire.read()<<8|Wire.read());                                 //Add the low and high byte to the gyro_x variable
  gyro_y = (Wire.read()<<8|Wire.read());                                 //Add the low and high byte to the gyro_y variable
  gyro_z = (Wire.read()<<8|Wire.read());                                 //Add the low and high byte to the gyro_z variable

  Wire.endTransmission();                                              //End the transmission

  resetPointerMPU();
}

void setup_mpu_6050(){
  
  Wire.begin();
  Wire.setClock(400000UL);
  
  writeToMPU( 25, byte((8000 / 100) - 1)); //  sample rate divider: sample rate = mstrClock / (1 +  divider)
  writeToMPU( 26, 0);            //  DLPF set.  (0 = 8kHz master clock else 1kHz master clock)
  writeToMPU( 27, 0x18);         //  2000 deg/s gyro
  writeToMPU( 28, 0x10);         //  16g accelerometer
  writeToMPU( 31, B00000000);    //  no motion detect
  writeToMPU( 35, B00000000);    //  no FIFO
  writeToMPU( 36, B00000000);    //  no mstr I2C
  writeToMPU( 55, B01110000);    //  configure interrupt  -- on when data ready, off on data read
  writeToMPU( 56, B00000001);    //  interrupt on
  writeToMPU(106, B00000000);    //  no silly stuff
  writeToMPU(107, B00000001);    //  no sleep and clock off gyro_X
  writeToMPU(108, B00000000);    //  no goofball sleep mode

  resetPointerMPU();
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

void writeToMPU(byte address, byte val) { // *** I2C Write Function ***
  Wire.beginTransmission(0x68); //start transmission to device 
  Wire.write(address);      // send register address
  Wire.write(val);        // send value to write
  Wire.endTransmission();     //end transmission
}

void resetPointerMPU() { // *** I2C Write Function ***
  Wire.beginTransmission(0x68); //start transmission to device 
  Wire.write(0x3B);        // send value to write
  Wire.endTransmission();     //end transmission
}

/* --- */

void wait(long t_wait) {
  for(long t_now = millis(); millis() < t_now + t_wait; ){}
}

