#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
#include <RFM69registers.h>
#include <SPI.h>

#define LED_R 5
#define LED_G 6
#define LED_DEFAULT 13

#define BITN 0x00
#define BIT0 0x01
#define BIT1 0x02
#define BIT2 0x04
#define BIT3 0x08
#define BIT4 0x10
#define BIT5 0x20
#define BIT6 0x40
#define BIT7 0x80

#define TX_RFM69 1
#define RX_RFM69 2

#define NETWORK_RFM69 100

#define TX_PER_SECOND 30

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
  pinMode(LED_DEFAULT, OUTPUT);
  
  if(radio.initialize(RF69_433MHZ, RX_RFM69, NETWORK_RFM69)) {
    if(radio.readReg(REG_SYNCVALUE2) == NETWORK_RFM69) {
      radio_ready = true;
    
      radio.setPowerLevel(31);
  
      radio.writeReg(REG_BITRATEMSB, RF_BITRATEMSB_9600);
      radio.writeReg(REG_BITRATELSB, RF_BITRATELSB_9600);
    }
  }
}

int count = 0;

void loop() {
}

void idleLoop() {
}

// UTIL METHOD SECTION

void blinkLed(unsigned int DELAY_MS, unsigned int LED_PIN) {
    digitalWrite(LED_PIN, HIGH);

    wait((2*DELAY_MS)/10);
    
    digitalWrite(LED_PIN, LOW);

    wait((8*DELAY_MS)/10);
}

void wait(long t_wait) {
  for(unsigned long t_now = millis(); millis() < t_now + t_wait; ){}
}

void waitIdle(long t_wait) {
  unsigned long t_now = millis();
  while(millis() < t_now + t_wait){
    idleLoop();
  }
}
