#define DEG_RAD 0.0174533

#include <elapsedMillis.h>

#include <SPI.h>

/* --- */

#include <RFM69.h>
#include <RFM69registers.h>

/* --- */

#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

#define OFF_X 70
#define OFF_Y 35

/* --- */

#include "radio_constants.h"
#include "radio_joysticks.h"

uint16_t xLeft, yLeft, xRight, yRight, xJoy;

/* --- */

RFM69                         rfm69;

PacketTransmitter             packetTransmitter;
PacketPlane                   packetPlane;

Adafruit_PCD8544              pcd = Adafruit_PCD8544(7, 8, 9);

/* --- */

bool                          rfm69_initialized = false; // true if rfm69 is initialized successfully

uint32_t                      t_last_ack = 0; // timestamp of last rssi packet arrival

int8_t lastRssi = 0;

elapsedMillis sinceDisplay;
elapsedMillis sinceSend;
elapsedMillis sinceCalibration;

/* --- */

byte contrast = 42;

void setup() {

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);

  pinMode(4, INPUT);
  pinMode(A6, INPUT);
  pinMode(A7, INPUT);

  digitalWrite(LED_G, LOW);
  digitalWrite(LED_R, HIGH);
  
  
  pcd.begin();
  
  pcd.setContrast(contrast);
  
  pcd.display();
  
  delay(50);
  
  pcd.clearDisplay();
  
  if(rfm69.initialize(RF69_433MHZ, TX_RFM69, NETWORK_RFM69)) {
    if(rfm69.readReg(REG_SYNCVALUE2) == NETWORK_RFM69) {
      rfm69_initialized = true;
    
      rfm69.setHighPower();
      rfm69.setPowerLevel(31);
  
      rfm69.writeReg(REG_BITRATEMSB, RF_BITRATEMSB_19200);
      rfm69.writeReg(REG_BITRATELSB, RF_BITRATELSB_19200);
    }
  }
  
  xLeft  = getXLeft();
  yLeft  = getYLeft();
  xRight = getXRight();
  yRight = getYRight();
  xJoy   = getJX();
}


void loop() {
  if(rfm69_initialized) {
    if(rfm69.receiveDone()) {
      if(rfm69.DATALEN == sizeof(PacketPlane)) {
        
        packetPlane = *(PacketPlane*)rfm69.DATA;

        t_last_ack = millis();
        
        if(packetPlane.rssi <= SIGNAL_WARN_LB) {
          int8_t rssi = packetPlane.rssi;
          rssi = max(SIGNAL_WARN_UB, rssi);
          rssi = min(SIGNAL_WARN_LB, rssi);
          rssi = map(rssi, SIGNAL_WARN_LB, SIGNAL_WARN_UB, 0, 255);
          analogWrite(LED_R, rssi);
        } else {
          analogWrite(LED_R, 0);
        }
 
        if(sinceCalibration > 10000) {
          sinceCalibration = 0;
          rfm69.rcCalibration();
        }
      }
    }
    digitalWrite(LED_G, HIGH);

    if(sinceSend > 33) {
      sinceSend = 0;

      xLeft  = xLeft /2 + getXLeft() /2;
      yLeft  = yLeft /2 + getYLeft() /2;
      xRight = xRight/2 + getXRight()/2;
      yRight = yRight/2 + getYRight()/2;
      xJoy   = xJoy  *8/10 + getJX() *2/10;

      packetTransmitter.ch0 = yLeft  >> 2;//throttle
      packetTransmitter.ch1 = xLeft  >> 2;//rudder
      packetTransmitter.ch2 = xRight >> 2;//aileron
      packetTransmitter.ch3 = yRight >> 2;//elevator
      packetTransmitter.ch4 = xJoy   >> 2;//camera pan

      rfm69.send(RX_RFM69, (const void*)(&packetTransmitter), sizeof(PacketTransmitter));
    }

    digitalWrite(LED_G, LOW);

  } else {
    digitalWrite(LED_R, HIGH);
    digitalWrite(LED_G, LOW);
  }
  
  if(sinceDisplay > 50) {
    sinceDisplay = 0;
    
    pcd.clearDisplay();
  
    pcd.setTextSize(1);
    pcd.setCursor(0,0);
  
    if(rfm69_initialized) {
      if(millis() - t_last_ack >= 100) {
        pcd.print("[SL-]");
      } else {
        pcd.print("[OK+]");
      }
    } else {
      pcd.print("[ERR]");
    }
    pcd.print(" SS:");
    int8_t rssi = packetPlane.rssi;
    if(rssi < -15) {
      lastRssi = rssi;
    }
    pcd.println(lastRssi);
    
    pcd.print("D/H:");
    if(packetPlane.distance < 1000) {
      pcd.print(packetPlane.distance);
      pcd.print("m/");
    } else {
      pcd.print(((float)packetPlane.distance)/1000.0,2);
      pcd.print("km/");
    }
    pcd.println(packetPlane.home_angle_d*2);
    
    pcd.print("A/V:");
    pcd.print(packetPlane.height);
    pcd.print("m/");
    pcd.print(packetPlane.speed);
    pcd.println("kmh");
  
    pcd.print("BAT:");
    pcd.print(((float)packetPlane.voltage)/4000.0);
    pcd.println("v");

    pcd.print("P/R:");
    pcd.print(packetPlane.pitch);
    pcd.print("/");
    pcd.println(packetPlane.roll);

    int8_t lx,ly;
    lx = (int8_t)(7.0 * cos(-((float)packetPlane.roll)*DEG_RAD));
    ly = (int8_t)(7.0 * sin(-((float)packetPlane.roll)*DEG_RAD));
  
    pcd.drawLine(OFF_X-lx, OFF_Y-ly, OFF_X+lx, OFF_Y+ly, BLACK);

    pcd.drawFastVLine(82, 30, 10, BLACK);
    pcd.drawFastHLine(81, OFF_Y + constrain(-packetPlane.pitch,-6,6), 3, BLACK);
    
    if(getJY() > 768)
      contrast++;
    else if(getJY() < 256)
      contrast--;
  
    contrast = min(45, contrast);
    contrast = max(35, contrast);
  
    noInterrupts();
  
      pcd.setContrast(contrast);
      pcd.display();
  
    interrupts();
  }
}
