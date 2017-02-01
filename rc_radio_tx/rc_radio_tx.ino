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
#include "radio_packets.h"

/* --- */

RFM69                         radio;
Adafruit_PCD8544              pcd = Adafruit_PCD8544(7, 8, 9);

Packet                        packet;
PacketRSSI                    packetRSSI;

/* --- */

boolean                       radio_initialized = false; // true if radio is initialized successfully

unsigned long                 t_last_rssi = 0; // timestamp of last rssi packet arrival

unsigned long                 t_last_update = 0; // time needed to complete the previous loop

int                           counter_rssi = 0;

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
  
  delay(500);
  
  pcd.clearDisplay();
  
  if(radio.initialize(RF69_433MHZ, TX_RFM69, NETWORK_RFM69)) {
    if(radio.readReg(REG_SYNCVALUE2) == NETWORK_RFM69) {
      radio_initialized = true;
    
      radio.setHighPower();
      radio.setPowerLevel(17);
  
      radio.writeReg(REG_BITRATEMSB, RF_BITRATEMSB_57600);
      radio.writeReg(REG_BITRATELSB, RF_BITRATELSB_57600);
    }
  }
}


void loop() {
  unsigned long t_update = millis();
    
  if(radio_initialized){
    if(radio.receiveDone()) {
      if(radio.DATALEN == sizeof(packetRSSI)) {
        packetRSSI = *(PacketRSSI*)radio.DATA;
        t_last_rssi = millis();
        if(packetRSSI.rssi <= SIGNAL_WARN_LB) {
          signed int rssi = packetRSSI.rssi;
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
    
    packet.x_left = getXLeft() >> 2;
    packet.y_left = getYLeft() >> 2;
    packet.x_right = getXRight() >> 2;
    packet.y_right = getYRight() >> 2;

    if(counter_rssi <= 0) {
      packet.flags = 0xAB;
      counter_rssi = SIGNAL_CHECK_SPACING;
    } else {
      packet.flags = 0xAA;
      counter_rssi--;
    }
    
    radio.send(RX_RFM69, (const void*)(&packet), sizeof(packet));

    digitalWrite(LED_G, LOW);

    radio.receiveDone();
  } else {
    digitalWrite(LED_R, HIGH);
    digitalWrite(LED_G, LOW);
  }

  if(counter_rssi != SIGNAL_CHECK_SPACING) {
    pcd.clearDisplay();
    
    pcd.drawFastVLine(41, 0, 48, BLACK);
    pcd.drawFastVLine(42, 0, 48, BLACK);
    
    pcd.drawRect(0, 0, 84 ,48, BLACK);
  
    pcd.setTextSize(1);
    pcd.setCursor(2,2);

    if(radio_initialized) {
      if(millis() - t_last_rssi >= (((SIGNAL_CHECK_SPACING) * 3) >> 1) * t_last_update) {
        pcd.println("[ ]");
      } else {
        pcd.println("[+]");
      }
    } else {
      pcd.println("");
    }
  
    int pxl = ((getXLeft()*41)/1023);
    int pyl = (((1023L - getYLeft())*47)/1023);
  
    int pxr = ((getXRight()*41)/1023);
    int pyr = (((1023 - getYRight())*47)/1023);
  
    //int pxr = ((getJX()*41)/1023);
    //int pyr = (((1023 - getJY())*47)/1023);
  
    pcd.drawFastVLine(pxl, 0, 48, BLACK);
    pcd.drawFastHLine(0, pyl, 42, BLACK);
  
    pcd.drawRect(pxl-2, pyl-2, 5, 5, BLACK);
    
    if(!getJB()) {
      pcd.drawFastVLine(pxr + 42, 0, 48, BLACK);
      pcd.drawFastHLine(42, pyr, 42, BLACK);
  
      pcd.drawRect(pxr-2 + 42, pyr-2, 5, 5, BLACK);
    }
    
    pcd.println(t_last_update);
    pcd.println(packetRSSI.rssi);
    pcd.println(((millis()-t_last_rssi)>>2)<<2);
  
  
  
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
    
  } else {
    radio.receiveDone();
  }


  unsigned long t_used = t_last_update = millis() - t_update;
  
  wait(max(0, (1000/TX_PER_SECOND) - t_used));
}

// UTIL METHOD SECTION

void wait(long t_wait) {
  long t_now = millis();
  while(millis() < t_now + t_wait);
}
