#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
#include <RFM69registers.h>

#include <SPI.h>

#define SERVOMIN    205// 1ms
#define SERVOMAX    409// 2ms
#define SERVOFREQ   50 // HZ

#define THROTTLE_HEADER 0

#define TX_RFM69 1
#define RX_RFM69 2

#define NETWORK_RFM69 100

#define TX_PER_SECOND 30

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

RFM69 radio;

typedef struct {
  uint8_t x_left;
  uint8_t y_left;
  uint8_t x_right;
  uint8_t y_right;
  uint8_t flags;
} Packet;

Packet packet;

typedef struct {
  signed int rssi;
} PacketRSSI;

PacketRSSI packetRSSI;

boolean radio_ready = false;

void setup() {
  pinMode(13, OUTPUT);
  
  pwm.begin();
  pwm.setPWMFreq(SERVOFREQ);
  
  setThrottle(0);
  
  if(radio.initialize(RF69_433MHZ, RX_RFM69, NETWORK_RFM69)) {
    if(radio.readReg(REG_SYNCVALUE2) == NETWORK_RFM69) {
      radio_ready = true;
    
      radio.setPowerLevel(28);
      radio.setHighPower();
  
      radio.writeReg(REG_BITRATEMSB, RF_BITRATEMSB_9600);
      radio.writeReg(REG_BITRATELSB, RF_BITRATELSB_9600);
    }
  }
}

void loop() {
  updateRadio();
}

int count = 0;

void updateRadio(){
  if(radio.receiveDone() && radio_ready) {
    if(radio.DATALEN == sizeof(Packet)) {
      
      packet = *(Packet*)radio.DATA;
      if(packet.flags == 0xAA) {
        count = (count+1)%5;
        if(count == 0) {
          packetRSSI.rssi = radio.RSSI;
          radio.send(TX_RFM69, (const void*)(&packetRSSI), sizeof(packetRSSI));
        }
      }
    }
  }
}

void setThrottle(int percent){
  pwm.setPWM(THROTTLE_HEADER, 0, mapAngle(map(percent, 0, 100, 45, 135)));
}

int mapAngle(int angle){
  angle = min(angle, 135);
  angle = max(angle, 45);
  return map(angle, 45, 135, SERVOMIN, SERVOMAX);
}

int mapAngleLogical(int8_t angle){
  angle = min(angle, 45);
  angle = max(angle, -45);
  return map(angle, -45, 45, SERVOMIN, SERVOMAX);
}

void wait(long t_wait) {
  for(long t_now = millis(); millis() < t_now + t_wait; ){}
}

