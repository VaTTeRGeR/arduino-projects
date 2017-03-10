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

RFM69                         radio;

PacketTransmitter             packetTransmitter;
PacketPlane                   packetPlane;

Adafruit_PCD8544              pcd = Adafruit_PCD8544(7, 8, 9);

/* --- */

bool                          radio_initialized = false; // true if radio is initialized successfully

uint32_t                      t_last_ack = 0; // timestamp of last rssi packet arrival

elapsedMillis sinceDisplay;
elapsedMillis sinceSend;

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
  
  if(radio.initialize(RF69_433MHZ, TX_RFM69, NETWORK_RFM69)) {
    if(radio.readReg(REG_SYNCVALUE2) == NETWORK_RFM69) {
      radio_initialized = true;
    
      radio.setHighPower();
      radio.setPowerLevel(4);
  
      radio.writeReg(REG_BITRATEMSB, RF_BITRATEMSB_57600);
      radio.writeReg(REG_BITRATELSB, RF_BITRATELSB_57600);
    }
  }
}


void loop() {
  if(radio_initialized) {
    if(radio.receiveDone()) {
      if(radio.DATALEN == sizeof(PacketPlane)) {
        
        packetPlane = *(PacketPlane*)radio.DATA;

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
      
      packetTransmitter.ch0 = getYLeft() >> 2;
      packetTransmitter.ch1 = getXLeft() >> 2;
      packetTransmitter.ch2 = getXRight() >> 2;
      packetTransmitter.ch3 = getYRight() >> 2;
  
      packetTransmitter.flags = 0;
  
      radio.send(RX_RFM69, (const void*)(&packetTransmitter), sizeof(PacketTransmitter));
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
  
    if(radio_initialized) {
      if(millis() - t_last_ack >= 100) {
        pcd.print("[-]");
      } else {
        pcd.print("[O]");
      }
    } else {
      pcd.print("[E]");
    }
    pcd.print(" SS:");
    pcd.println(constrain(packetPlane.rssi, -20, -120));
    
    pcd.print("D/A:");
    pcd.print(packetPlane.distance);
    pcd.print("/");
    pcd.println(packetPlane.home_angle_d<<1);
    
    pcd.print("H:");
    pcd.println(packetPlane.height);
  
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
