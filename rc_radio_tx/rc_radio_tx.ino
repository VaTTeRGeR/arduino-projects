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

/* --- */

byte contrast = 40;

void setup() {

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);

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

  if(false /*radio.initialize(RF69_433MHZ, TX_RFM69, NETWORK_RFM69)*/) {
    if(radio.readReg(REG_SYNCVALUE2) == NETWORK_RFM69) {
      radio_initialized = true;
    
      radio.setHighPower();
      radio.setPowerLevel(18);
  
      radio.writeReg(REG_BITRATEMSB, RF_BITRATEMSB_9600);
      radio.writeReg(REG_BITRATELSB, RF_BITRATELSB_9600);
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
    packet.flags = 0xAA;
    
    radio.send(RX_RFM69, (const void*)(&packet), sizeof(packet));

    digitalWrite(LED_G, LOW);

    radio.receiveDone();
    
    if(millis() - t_last_rssi >= SIGNAL_WARN_TIME) {
      t_last_rssi = millis() - SIGNAL_WARN_TIME;
      digitalWrite(LED_R, HIGH);
      wait(10);
      digitalWrite(LED_R, LOW);
      wait(10);
      digitalWrite(LED_R, HIGH);
      wait(10);
      digitalWrite(LED_R, LOW);
      wait(10);
    }
    
  } else {
    //blinkLed(20, LED_R);
  }
  
  pcd.clearDisplay();
  
  pcd.drawFastVLine(41, 0, 48, BLACK);
  pcd.drawFastVLine(42, 0, 48, BLACK);
  
  pcd.drawRect(0, 0, 84 ,48, BLACK);

  pcd.setTextSize(1);
  pcd.setCursor(2,2);
  pcd.println(radio_initialized ? "ROK" : "ERR");

  int pxl = ((getXLeft()*41)/1023);
  int pyl = (((1023L - getYLeft())*47)/1023);

  //int pxr = ((getXRight()*41)/1023);
  //int pyr = (((1023 - getYRight())*47)/1023);

  int pxr = ((getJX()*41)/1023);
  int pyr = (((1023 - getJY())*47)/1023);

  pcd.drawFastVLine(pxl, 0, 48, BLACK);
  pcd.drawFastHLine(0, pyl, 42, BLACK);

  pcd.drawRect(pxl-2, pyl-2, 5, 5, BLACK);
  
  if(!getJB()) {
    pcd.drawFastVLine(pxr + 42, 0, 48, BLACK);
    pcd.drawFastHLine(42, pyr, 42, BLACK);

    pcd.drawRect(pxr-2 + 42, pyr-2, 5, 5, BLACK);
  }
  
  pcd.println(t_last_update);
  pcd.println(contrast);

  if(getJY() > 768)
    pcd.setContrast(contrast++);
  else if(getJY() < 256)
    pcd.setContrast(contrast--);

  contrast = min(127, contrast);
  contrast = max(0, contrast);
  
  pcd.display();

  unsigned long t_used = t_last_update = millis() - t_update;
  
  wait(max(0, (1000/TX_PER_SECOND) - t_used));
}

// UTIL METHOD SECTION

void blinkLed(unsigned int DELAY_MS, unsigned int LED_PIN) {
    digitalWrite(LED_PIN, HIGH);

    wait((2*DELAY_MS)/10);
    
    digitalWrite(LED_PIN, LOW);

    wait((8*DELAY_MS)/10);
}

void wait(long t_wait) {
  long t_now = millis();
  while(millis() < t_now + t_wait){}
}
