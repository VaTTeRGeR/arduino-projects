#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
#include <RFM69registers.h>
#include <SPI.h>

#define LED 5

RFM69 radio;

typedef struct {
  int number;
} Packet;

Packet data;


//RF_BITRATEMSB_1200            0x68
//RF_BITRATELSB_1200            0x2B
void setup() {
  pinMode(LED, OUTPUT);
  
  Blink(1000);
  Blink(1000);
  Blink(1000);
  
  if(radio.initialize(RF69_433MHZ, 1, 100)) {
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
  radio.setHighPower();
  radio.setPowerLevel(6);
  radio.writeReg(REG_BITRATEMSB, RF_BITRATEMSB_1200);
  radio.writeReg(REG_BITRATELSB, RF_BITRATELSB_1200);

  
  Blink(1000);
  Blink(1000);
  Blink(1000);
  
  data.number = 133;//7;
}

void loop() {
    radio.send(2, (const void*)(&data), sizeof(data));
    Blink(map(analogRead(A0), 0, 1023, 10, 500));
}

void Blink(unsigned int DELAY_MS) {
    digitalWrite(LED, HIGH);
    delay(DELAY_MS);
    digitalWrite(LED, LOW);
    delay(DELAY_MS);
}
