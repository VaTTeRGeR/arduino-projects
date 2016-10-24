#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
#include <RFM69registers.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>

#define LED 13

RFM69 radio;

typedef struct {
  int number;
} Packet;

Packet data;


void setup() {
  pinMode(13, OUTPUT);

  Blink(1000);
  Blink(1000);
  Blink(1000);
  
  if(radio.initialize(RF69_433MHZ, 2, 100)) {
    Blink(100);
    Blink(100);
    Blink(100);
    Blink(100);
    Blink(100);
    Blink(100);
    Blink(100);
    Blink(100);
    Blink(100);
    Blink(100);
  }
  radio.setPowerLevel(31);
  radio.writeReg(REG_BITRATEMSB, RF_BITRATEMSB_1200);
  radio.writeReg(REG_BITRATELSB, RF_BITRATELSB_1200);

  
  Blink(1000);
  Blink(1000);
  Blink(1000);
}

void loop() {
    if(radio.receiveDone()) {
      if(radio.DATALEN != sizeof(Packet)) {
        Blink(50);
        Blink(50);
        Blink(50);
        Blink(50);
        Blink(50);
      } else {
        data = *(Packet*)radio.DATA;
        if(data.number == 1337) {
          Blink(250);
          Blink(250);
        } else {
          Blink(50);
        }
      }
    }
}

void Blink(byte DELAY_MS) {
    digitalWrite(LED, HIGH);
    delay(DELAY_MS);
    digitalWrite(LED, LOW);
    delay(DELAY_MS);
}
