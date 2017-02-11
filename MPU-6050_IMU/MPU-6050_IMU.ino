#include <PWMServo.h>

#include <i2c_t3.h>

#define SAMPLERATE  100

#define Q_RADDEG    57.29577
#define Q_DEGRAD    0.01745329

signed long    gyro_x_cal, gyro_y_cal, gyro_z_cal;

signed long    gyro_x, gyro_y, gyro_z;
signed long    acc_x, acc_y, acc_z;

signed long    acc_total_vector;
signed long    still_length_acc;

float   angle_pitch, angle_roll;
float   angle_roll_acc, angle_pitch_acc;

float   angle_pitch_output, angle_roll_output;

int     temperature;

long    loop_timer;

boolean gyro_angles_set;

PWMServo servo;

void setup() {
  //Serial.begin(115200);                                               //Use only for debugging
  
  pinMode(13, OUTPUT);                                                 //Set output 13 (LED) as output
  
  setup_mpu_6050();                                          //Setup the registers of the MPU-6050 (500dfs / +/-8g) and start the gyro

  digitalWrite(13, HIGH);                                              //Set digital output 13 high to indicate startup

  calibrate_mpu_6050();

  servo.attach(35);
  
  digitalWrite(13, LOW);                                               //All done, turn the LED off
  
  loop_timer = micros();                                               //Reset the loop timer
}

void loop(){
  read_mpu_6050_data();

  gyro_x -= gyro_x_cal;
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;

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

  if(abs(abs(angle_pitch_output)-45)<5 || abs(abs(angle_roll_output)-45)<5) {
    digitalWrite(13, HIGH);
  } else {
    digitalWrite(13, LOW);
  }

  servo.write(map(angle_pitch_output, -90, 90, 0, 180));

  /*Serial.print(acc_x);
  Serial.print(",");
  Serial.print(acc_y);
  Serial.print(",");
  Serial.print(acc_z);
  Serial.print("-");
  Serial.print(gyro_x);
  Serial.print(",");
  Serial.print(gyro_y);
  Serial.print(",");
  Serial.println(gyro_z);*/

  while(micros() - loop_timer < 1000000/SAMPLERATE);                                 //Wait until the loop_timer reaches target before starting the next loop
  loop_timer = micros();                                               //Reset the loop timer
}


void read_mpu_6050_data(){                                             //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050

  Wire.requestFrom(0x68, 14);                                           //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);                                        //Wait until all the bytes are received
  
  //READS IN THIS ORDER: AX, AY, AZ | GX, GY, GZ
  
  acc_x = (int16_t)(Wire.read()<<8|Wire.read());                                  //Add the low and high byte to the acc_x variable
  acc_y = (int16_t)(Wire.read()<<8|Wire.read());                                  //Add the low and high byte to the acc_y variable
  acc_z = (int16_t)(Wire.read()<<8|Wire.read());                                  //Add the low and high byte to the acc_z variable

  temperature = (int16_t)(Wire.read()<<8|Wire.read());                            //Add the low and high byte to the temperature variable
  
  gyro_x = (int16_t)(Wire.read()<<8|Wire.read());                                 //Add the low and high byte to the gyro_x variable
  gyro_y = (int16_t)(Wire.read()<<8|Wire.read());                                 //Add the low and high byte to the gyro_y variable
  gyro_z = (int16_t)(Wire.read()<<8|Wire.read());                                 //Add the low and high byte to the gyro_z variable

  Wire.endTransmission();                                              //End the transmission

  resetPointerMPU();
}

void setup_mpu_6050(){

  Wire.begin(I2C_MASTER, 0, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000); //Start I2C as master

  delay(200);

  writeToMPU(107, B00000001);    //  no sleep and clock off gyro_X
  writeToMPU(108, B00000000);    //  no goofball sleep mode
  
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

