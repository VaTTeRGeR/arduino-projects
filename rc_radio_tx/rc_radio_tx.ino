#include <elapsedMillis.h>

#include <SPI.h>

/* --- */

#include <RFM69.h>
#include <RFM69registers.h>

/* --- */

#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

/* --- */

#include "radio_constants.h"
#include "radio_joysticks.h"

/* --- */

RFM69                         rfm69;

PacketTransmitter             packetTransmitter;
PacketPlane                   packetPlane;

Adafruit_PCD8544              pcd = Adafruit_PCD8544(7, 8, 9);

/* --- */

bool                          rfm69_initialized = false; // true if rfm69 is initialized successfully

uint32_t                      t_last_ack = 0; // timestamp of last rssi packet arrival

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
      }
    }
    digitalWrite(LED_G, HIGH);

    if(sinceSend > 33) {
      sinceSend = 0;
      
      packetTransmitter.ch0 = getYLeft()  >> 2;
      packetTransmitter.ch1 = getXLeft()  >> 2;
      packetTransmitter.ch2 = getXRight() >> 2;
      packetTransmitter.ch3 = getYRight() >> 2;
      packetTransmitter.ch4 = getJX()     >> 2;
  
      rfm69.send(RX_RFM69, (const void*)(&packetTransmitter), sizeof(PacketTransmitter));

      if(sinceCalibration > 20000) {
        sinceCalibration = 0;
        rfm69.rcCalibration();
      }
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
    pcd.println(packetPlane.rssi);
    
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
