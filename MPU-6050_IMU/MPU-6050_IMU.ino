#include <i2c_t3.h>
#include <SFE_BMP180.h>

#include "gps.h"

#define SAMPLERATE  100

#define Q_RADDEG    57.29577
#define Q_DEGRAD    0.01745329

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

signed long    gyro_x_cal, gyro_y_cal, gyro_z_cal;

signed long    gyro_x, gyro_y, gyro_z;
signed long    acc_x, acc_y, acc_z;

signed long    acc_total_vector;
signed long    still_length_acc;

float   angle_pitch, angle_roll;
float   angle_pitch_output, angle_roll_output;
float   angle_roll_acc, angle_pitch_acc;

int     temperature;

byte          mpu6050_task;
elapsedMillis mpu6050_wait_elapsed;

boolean gyro_angles_set;

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

void setup() {
  delay(10); //Wait for all sensors to boot
  
  //Serial.begin(115200);                                                 //Use only for debugging

  Wire.begin(I2C_MASTER, 0, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);    //Start I2C as master
  
  pinMode(13, OUTPUT);                                                  //Set output 13 (LED) as output
  
  setup_mpu_6050();                                                     //Setup the registers of the MPU-6050 (500dfs / +/-8g) and start the gyro
  
  setup_bmp180();                                                     //Setup the registers of the MPU-6050 (500dfs / +/-8g) and start the gyro

  setup_gps();
  
  servo0.attach(35);
  servo1.attach(36);
  servo2.attach(37);
  servo3.attach(38);
  servo4.attach(30);
  servo5.attach(29);
  servo6.attach(16);
}

void loop(){
  update_mpu6050();
  update_bmp180();
  update_gps();
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

/* -------- */
/* MPU 6050 */
/* -------- */

void setup_mpu_6050(){
  writeToMPU(107, B00000001);    //  no sleep and clock off gyro_X
  writeToMPU(108, B00000000);    //  no goofball sleep mode
  
  writeToMPU( 25, byte((8000 / 200) - 1)); //  sample rate divider: sample rate = mstrClock / (1 +  divider)
  writeToMPU( 26, B00000000);            //  DLPF set.  (0 = 8kHz master clock else 1kHz master clock)
  writeToMPU( 27, 0x18);         //  2000 deg/s gyro
  writeToMPU( 28, 0x10);         //  16g accelerometer
  writeToMPU( 31, B00000000);    //  no motion detect
  writeToMPU( 35, B00000000);    //  no FIFO
  writeToMPU( 36, B00000000);    //  no mstr I2C
  writeToMPU( 55, B01110000);    //  configure interrupt
  writeToMPU( 56, B00000000);    //  no interrupt
  writeToMPU(106, B00000000);    //  no silly stuff

  digitalWrite(13, HIGH);
  calibrate_mpu_6050();
  digitalWrite(13, LOW);

  mpu6050_task = TASK_ACTIVE;
  mpu6050_wait_elapsed = 0;
}

void update_mpu6050(){
  if(mpu6050_task == TASK_ACTIVE && mpu6050_wait_elapsed >= 1000/SAMPLERATE) {

    mpu6050_wait_elapsed -= 1000/SAMPLERATE;
    
    read_mpu_6050_data();
  
    gyro_x -= gyro_x_cal;
    gyro_y -= gyro_y_cal;
    gyro_z -= gyro_z_cal;
  
    //float gyroToAngleChange = 1.0/((float)SAMPLERATE)/65.5;            // for 500 deg/s
    float gyroToAngleChange = 1.0/((float)SAMPLERATE)/16.375;            // for 2000 deg/s
  
    //Gyro angle calculations
    angle_pitch += ((float)gyro_x) * gyroToAngleChange;                  //Calculate the traveled pitch angle and add this to the angle_pitch variable
    angle_roll += ((float)gyro_y) * gyroToAngleChange;                   //Calculate the traveled roll angle and add this to the angle_roll variable
    
    float gyrozScl = gyroToAngleChange * Q_DEGRAD;                       //The Arduino sin function is in radians
    angle_pitch += angle_roll * sin(gyro_z * gyrozScl);                  //If the IMU has yawed transfer the roll angle to the pitch angel
    angle_roll -= angle_pitch * sin(gyro_z * gyrozScl);                  //If the IMU has yawed transfer the pitch angle to the roll angel
    
    //Accelerometer angle calculations
    acc_total_vector = sqrt( (acc_x*acc_x) + (acc_y*acc_y) + (acc_z*acc_z) ); //Calculate the total accelerometer vector
    angle_pitch_acc = asin((float)acc_y / acc_total_vector)*  Q_RADDEG;    //Calculate the pitch angle
    angle_roll_acc = asin((float)acc_x / acc_total_vector) * -Q_RADDEG;    //Calculate the roll angle
    
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
    //angle_pitch_output = angle_pitch_output * 0.5 + angle_pitch * 0.5;   //Take 50% of the output pitch value and add 50% of the raw pitch value
    //angle_roll_output = angle_roll_output * 0.5 + angle_roll * 0.5;      //Take 50% of the output roll value and add 50% of the raw roll value
  
    //To dampen the pitch and roll angles a complementary filter is used
    angle_pitch_output = angle_pitch;
    angle_roll_output = angle_roll;
  
    servo0.write(-angle_pitch_output + offset + 90);
    servo1.write(angle_pitch_output + offset + 90);
    servo2.write(offset + 90);
    servo3.write(offset + 90);
    servo4.write(offset + 90);
    servo5.write(offset + 90);
    servo6.write(offset + 90);
  
    if(abs(abs(angle_pitch_output)-45.0)<2.5 || abs(abs(angle_roll_output)-45.0)<2.5) {
      digitalWrite(13, HIGH);
    } else {
      digitalWrite(13, LOW);
    }
  }
}

void read_mpu_6050_data(){                                             //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(59);                                                    // send register pointer
  Wire.endTransmission();                                              //End the transmission

  Wire.requestFrom(0x68, 14, true);                                          //Request 14 bytes from the MPU-6050

  while(Wire.available()<14);
    
  //READS IN THIS ORDER: AX, AY, AZ | TEMP | GX, GY, GZ
  
  acc_x = (int16_t)(Wire.read()<<8|Wire.read());                                  //Add the low and high byte to the acc_x variable
  acc_y = (int16_t)(Wire.read()<<8|Wire.read());                                  //Add the low and high byte to the acc_y variable
  acc_z = (int16_t)(Wire.read()<<8|Wire.read());                                  //Add the low and high byte to the acc_z variable

  temperature = (int16_t)(Wire.read()<<8|Wire.read());                            //Add the low and high byte to the temperature variable
  
  gyro_x = (int16_t)(Wire.read()<<8|Wire.read());                                 //Add the low and high byte to the gyro_x variable
  gyro_y = (int16_t)(Wire.read()<<8|Wire.read());                                 //Add the low and high byte to the gyro_y variable
  gyro_z = (int16_t)(Wire.read()<<8|Wire.read());                                 //Add the low and high byte to the gyro_z variable

}

void calibrate_mpu_6050(){
  
  signed long cal_rounds = 0;
  unsigned long t_cal_start = micros();
  
  while((micros() - t_cal_start < 500000 && cal_rounds < 2048) || cal_rounds == 0) {                  //Run this code for 250 ms or 2000 cycles
    read_mpu_6050_data();                                              //Read the raw acc and gyro data from the MPU-6050
    
    gyro_x_cal += gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z;                                              //Add the gyro z-axis offset to the gyro_z_cal variable

    still_length_acc += sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));

    cal_rounds++;
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
