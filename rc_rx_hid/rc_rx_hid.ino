/* --- */

#include <SPI.h>
#include <RFM69.h>
#include <RFM69registers.h>
#include "radio_constants.h"
#include "radio_packets.h"

RFM69         radio;

PacketRSSI    packetRSSI;
Packet        packet;

boolean       radio_ready = false;

/* --- */

#include "Joystick.h"

/* --- */

unsigned long loop_timer;

/* - SETUP - */

void setup() {

  //Serial.begin(115200);

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  for(int i = 0; i<10;i++){
    digitalWrite(13, HIGH);
    delay(25);
    digitalWrite(13, LOW);
    delay(25);
  }

  setupRadio();
  
  Joystick.begin(false);
    
  loop_timer = micros();
}

void loop() {
  updateRadio();
  
  Joystick.setXAxis(packet.x_right - 127);
  Joystick.setYAxis(256 - packet.y_right - 127);

  Joystick.setRudder(packet.x_left);
  Joystick.setThrottle(256 - packet.y_left);

  Joystick.sendState();
  
  //while(micros() - loop_timer < 1000000L/120L);
  //loop_timer = micros();
}

/* --- */

void setupRadio(){
  if(radio.initialize(RF69_433MHZ, RX_RFM69, NETWORK_RFM69)) {
    if(radio.readReg(REG_SYNCVALUE2) == NETWORK_RFM69) {
      radio_ready = true;
    
      radio.setPowerLevel(17);
      //radio.setHighPower();
  
      radio.writeReg(REG_BITRATEMSB, RF_BITRATEMSB_57600);
      radio.writeReg(REG_BITRATELSB, RF_BITRATELSB_57600);
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
